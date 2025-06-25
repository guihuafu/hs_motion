#include "HS_AutoMove.h"
#include <iostream>
#include "HS_Int_Factory.h"
#include "HS_Kinematics.h"


HS_AutoMove::HS_AutoMove(MotionPara *para,double dCycle,int iInterMultCnt)
{
    this->mMotionPara = para;
    m_pHS_BasicPara = HS_BasicPara::GetInstance();
    m_pHS_BasicPara->SetPara(para);
    m_pHS_BasicPara->SetCycle(dCycle,iInterMultCnt);
    m_tFilterControl = &m_pHS_BasicPara->m_tSysFilterPara.tAutoControl;

	m_HS_GroupKin = new HS_GroupKin();

    m_iInterMultCnt = iInterMultCnt;
	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		m_HS_SetPosCheck[iGroup] = new HS_SetPosCheck(iGroup);
	}

    m_HS_Kinematics = m_HS_GroupKin->GetKinematicsByNum(0);

    m_HS_Int_JointPreH = new HS_Int_Factory(MP_Joint,m_HS_GroupKin); 
    m_HS_Int_LinePreH = new HS_Int_Factory(MP_Line,m_HS_GroupKin);;
    m_HS_Int_SLinePreH = new HS_Int_Factory(MP_BLine,m_HS_GroupKin);
    m_HS_Int_CirclePreH = new HS_Int_Factory(MP_Arc,m_HS_GroupKin);

    m_HS_Kinematics->InitPara();

    for(int i = 0;i < 2;i++)
    {
		for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
		{
			m_HS_Int_Joint[i][iGroup] = new HS_Int_Factory(MP_Joint,m_HS_GroupKin); 
			m_HS_Int_Line[i][iGroup] = new HS_Int_Factory(MP_Line,m_HS_GroupKin);
			m_HS_Int_SLine[i][iGroup] = new HS_Int_Factory(MP_BLine,m_HS_GroupKin);
			m_HS_Int_Circle[i][iGroup] = new HS_Int_Factory(MP_Arc,m_HS_GroupKin);
		}
    }
    m_HS_FilterHandle = new HS_FilterHandle(dCycle);
    m_HS_SpaceFilter = new HS_FilterHandle(dCycle);

    m_iAutoFilterType = 0;
    if(m_tFilterControl->bFilterOpenFlag)
    {
        m_HS_FilterHandle->Filer_SetPara(m_tFilterControl->tFilterPara);
        m_iAutoFilterType = m_pHS_BasicPara->m_tSysFilterPara.iAutoFilterType;
    }
    Reset();
}

HS_AutoMove::~HS_AutoMove()
{
	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		delete m_HS_SetPosCheck[iGroup];
	}
	delete m_HS_GroupKin;
    delete m_HS_FilterHandle;
    delete m_HS_SpaceFilter;
    for(int i = 0;i < 2;i++)
    {
		for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
		{
			delete m_HS_Int_Joint[i][iGroup];
			m_HS_Int_Joint[i][iGroup] = NULL;
			delete m_HS_Int_Line[i][iGroup];
			delete m_HS_Int_Circle[i][iGroup];
			delete m_HS_Int_SLine[i][iGroup];
		}
    }

    delete m_HS_Int_JointPreH;
    m_HS_Int_JointPreH = NULL;
    delete m_HS_Int_LinePreH;
    m_HS_Int_LinePreH = NULL;
    delete m_HS_Int_SLinePreH;
    m_HS_Int_SLinePreH = NULL;
    delete m_HS_Int_CirclePreH;
    m_HS_Int_CirclePreH = NULL;
}

void HS_AutoMove::execReset()
{
    Reset();
}

void HS_AutoMove::InitPara()
{
    //第一段运动，复位参数
    if(m_tFilterControl->bFilterOpenFlag)
    {
        m_HS_FilterHandle->Filer_SetPara(m_tFilterControl->tFilterPara);
        m_iAutoFilterType = m_pHS_BasicPara->m_tSysFilterPara.iAutoFilterType;
    }
	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		m_HS_SetPosCheck[iGroup]->ResetCheck();
	}
    m_HS_FilterHandle->ResetFilter();
    m_bSLFiterFlag = false;
    m_bSetPosErrorFlag = false;
	for(int i = 0;i < 2;i++)
	{
		for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
		{
			m_HS_Int_Joint[i][iGroup]->ResetData(); 
			m_HS_Int_Line[i][iGroup]->ResetData();
			m_HS_Int_SLine[i][iGroup]->ResetData();
			m_HS_Int_Circle[i][iGroup]->ResetData();
		}
	}
	m_eStopState = S_NONE;
    LOG_ALGO("Init Para!");
}

void HS_AutoMove::RestartInit()
{
	if(m_tFilterControl->bFilterOpenFlag)
	{
		m_HS_FilterHandle->Filer_SetPara(m_tFilterControl->tFilterPara);
		m_iAutoFilterType = m_pHS_BasicPara->m_tSysFilterPara.iAutoFilterType;
	}
	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		m_HS_SetPosCheck[iGroup]->ResetCheck();
	}
	m_HS_FilterHandle->ResetFilter();
	m_bSLFiterFlag = false;
	m_bSetPosErrorFlag = false;
	m_eStopState = S_NONE;
	LOG_ALGO("Restart Init!");
}

void HS_AutoMove::Reset()
{
    m_bPlanFlag = false;
    m_bSmoothPreFlag = false;
    m_bSmoothNextFlag = false;
    m_bFilterDoneFlag = false;
    m_bSetPosErrorFlag = false;
	m_bSmoothJointFlag = false;
	m_bMoveOnFlag = false;
    m_iIntIndex = 0;
	m_iBaseGroup = 0;
	m_bGroupPlanFlag = false;
	m_bSmoothAheadFlag = false;
	m_bRestartAddFlag = false;
	m_iCurId = -1;
	m_iSLGroup = 0;
    memset(&m_tMoveDoneJPos,0,sizeof(m_tMoveDoneJPos));
    memset(m_dSFilterRJPos,0,sizeof(double)*MaxAxisNum);

    m_iJointBuffIndex = 0;
    memset(m_iJointBuff,-1,sizeof(m_iJointBuff));

	for(int i = 0;i < MAXGROUPNUM;i++)
		m_iGroupId[i] = i;
}

int HS_AutoMove::execPrintKeyInfo()
{
	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		m_HS_SetPosCheck[iGroup]->ResetCheck();
	}
    m_HS_FilterHandle->ResetFilter();
    //return m_HS_Kinematics->PrintKeyInfo();
	return 0;
}

/************************************************
函数功能：多轴组预处理规划算法
参    数：
		 tGroupMotionData----多轴组输入运行段数据
         tTrajData-----------规划参数数据缓存
		 iIndex--------------索引行号
返 回 值：错误码
*************************************************/
int HS_AutoMove::execPrehandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex)
{
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	if (gettimeofday(&l_beginTime_Auto, NULL) == -1)
		return -1;
#endif

	int iErrorId = 0;

	HS_Int_Factory *pHS_Int_Factory;

	GroupIdSort(tGroupMotionData.tHS_GroupRel);

	LOG_ALGO("-------------------------------------------------------------------------");
	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(tGroupMotionData.tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		if(tGroupMotionData.bStartMove)
		{
			HS_Kinematics *pHS_Kinematics = m_HS_GroupKin->GetKinematicsByNum(iGroup);

			LOG_ALGO("---------------------------------Group:%d--------------------------------",iGroup);
			pHS_Kinematics->PrintKeyInfo();
			LOG_ALGO("-------------------------------------------------------------------------");
		}

		switch(tGroupMotionData.tBaseMoveData[iGroup].eTrajType)
		{
		case MP_Joint:
			pHS_Int_Factory = m_HS_Int_JointPreH; 
			break;
		case MP_Line:
			if(tGroupMotionData.tBaseMoveData[iGroup].iCntType == 1)
				pHS_Int_Factory = m_HS_Int_SLinePreH;
			else
				pHS_Int_Factory = m_HS_Int_LinePreH; 
			break;
		case MP_Arc:
			pHS_Int_Factory = m_HS_Int_CirclePreH; 
			break;
		case MP_BLine:
			pHS_Int_Factory = m_HS_Int_SLinePreH;
			break;
		default:
			LOG_ALGO("Error Move Type:%d;index:%d;Group:%d!",tGroupMotionData.tBaseMoveData[iGroup].eTrajType,iIndex,iGroup);
			return E_MOVE_MOVETYPE;
			break;
		}

		iErrorId = pHS_Int_Factory->PreHandle(tGroupMotionData,tTrajData,iIndex,iGroup); 
		if(tGroupMotionData.tBaseMoveData[iGroup].eTrajType == MP_Line&&tGroupMotionData.tBaseMoveData[iGroup].iCntType == 1)
		{
			tTrajData[iIndex].tMotionData.tBaseMoveData[iGroup].eTrajType = MP_BLine;
		}
		if(iErrorId != 0)
			return iErrorId;
	}

#ifdef _LINUX_
	if(gettimeofday(&l_endTime_Auto, NULL) == -1)
		return -1;
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
	LOG_ALGO("Time_Prehandle = %.3lf\n",de_us_Auto);
#endif
	return iErrorId;
} 

int HS_AutoMove::execPlanMove(GroupTrajData *tTrajData,int iIndex,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	int iErrorId = 0;

	if(tTrajData[iIndex].tMotionData.bStartMove)
	{
		InitPara();
	}

	m_tHS_GroupRel = tTrajData[iIndex].tMotionData.tHS_GroupRel;

	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		m_HS_Int_FactoryPre[iGroup] = m_HS_Int_Factory[iGroup];
	}
	m_iIntIndex = (m_iIntIndex + 1)%2;

	m_iCurId = iIndex;
	m_tTrajData = tTrajData;

	int iPlanGroupNum = 0;

	GroupIdSort(m_tHS_GroupRel);

	LOG_ALGO("-------------------------------------------------------------------------");
	int iRepeatCnt = 0;
	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;		

		iPlanGroupNum++;
		switch(tTrajData[iIndex].tMotionData.tBaseMoveData[iGroup].eTrajType)
		{
		case MP_Joint:
			m_HS_Int_Factory[iGroup] = m_HS_Int_Joint[m_iIntIndex][iGroup]; 
			break;
		case MP_Line:
			m_HS_Int_Factory[iGroup] = m_HS_Int_Line[m_iIntIndex][iGroup]; 
			break;
		case MP_Arc:
			m_HS_Int_Factory[iGroup] = m_HS_Int_Circle[m_iIntIndex][iGroup]; 
			break;
		case MP_BLine:
			m_HS_Int_Factory[iGroup] = m_HS_Int_SLine[m_iIntIndex][iGroup];
			break;
		default:
			LOG_ALGO("Error Move Type:%d;index:%d!",tTrajData[iIndex].tMotionData.tBaseMoveData[iGroup].eTrajType,iIndex);
			return E_MOVE_MOVETYPE;
			break;
		}

		int iRet = m_HS_Int_Factory[iGroup]->Plan(tTrajData,iIndex,iGroup,dRatio,tHS_GroupJPos);

		m_iSLGroup = iGroup;
		if(iRet != W_REPEAT)
			m_iBaseGroup = iGroup;

		//多轴组反馈值处理策略，任一为报警，则报警，全部为W_REPEAT才为W_REPEAT	
		if(iRet != 0&&iRet != W_REPEAT)
			iErrorId = iRet;

		if(iRet == W_REPEAT)
			iRepeatCnt++;
	}	

	tTrajData[iIndex].tMotionData.iLineNum = -1;

	if(iRepeatCnt == iPlanGroupNum)
		iErrorId = W_REPEAT;

	if(iPlanGroupNum > 1)
	{
		GroupSyncHandle();
		m_bGroupPlanFlag = true;
	}
	else
		m_bGroupPlanFlag = false;

	if(iErrorId == 0)
	{
		m_bPlanFlag = true;
		if(tTrajData[iIndex].tMotionData.dCnt > Eps)
		{
			m_bSmoothNextFlag = true;
		}
		else
			m_bSmoothNextFlag = false;
		m_bFilterDoneFlag = false;
	}
	else if(iErrorId == W_REPEAT)
	{
		//警告可以继续运动，但与前后段的平滑不生效
		m_bPlanFlag = true;
		m_bSmoothPreFlag = false;
		m_bSmoothNextFlag = false;
		m_bFilterDoneFlag = false;
	}
	else 
	{
		if(tTrajData[iIndex].tMotionData.bStartMove)
			return iErrorId;
		else
		{
			//继续运行前一段运动,等待滤波结束
			for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
			{
				m_HS_Int_Factory[iGroup] = m_HS_Int_FactoryPre[iGroup];
			}
		}
	}    
	m_bMoveOnFlag = true;
	m_bSmoothAheadFlag = false;
	m_bRestartAddFlag = false;

	DynLinePlanFilterStart(tHS_GroupJPos);
	return iErrorId;
}
/************************************************
函数功能：直线动态规划的空间滤波处理
参    数：		
返 回 值：报警码
*************************************************/
int HS_AutoMove::DynLinePlanFilterStart(HS_GroupJPos &tHS_GroupJPos)
{
	if(m_HS_Int_Factory[m_iSLGroup]->GetSLPlanFlag()&&!m_bSLFiterFlag)
	{
		m_HS_Kinematics = m_HS_GroupKin->GetKinematicsByNum(m_iSLGroup);
		m_bSLFiterFlag = true;
		m_bSLFilterInitFlag = true;
		FilterPara tFilterPara;
		tFilterPara.eFilterType = TYPE_FIR;
		tFilterPara.dFre = 20;
		tFilterPara.iAxisNum = 9;
		tFilterPara.iGrade = 9;
		m_HS_SpaceFilter->Filer_SetPara(tFilterPara);
		memcpy(m_dSFilterRJPos,tHS_GroupJPos.dJPos[m_iSLGroup],sizeof(double)*MaxAxisNum);
		LOG_ALGO("SLine Filter Open!");
	}
	//m_bSLFiterFlag = false;
	return 0;
}

int HS_AutoMove::GroupSyncHandle()
{
	//获取
	SyncPara tSyncPara[MAXGROUPNUM] = {SyncPara()};
	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;
		tSyncPara[iGroup] = m_HS_Int_Factory[iGroup]->GetSyncPara();
	}

	//同步
	double dMaxKDV = 0;	
	double dKDV[MAXGROUPNUM][MaxAxisNum] = {0};
	double dMaxTAcc = 0;
	double dMaxTDec = 0;
	double dMaxTSmooth[2] = {0};

	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		for(int i = 0;i < MaxAxisNum;i++)
		{			
			dKDV[iGroup][i] = 0;
			//1.计算与速度的比值
			if(fabs(tSyncPara[iGroup].tVelPlan.dEVel[i]) > Eps)
				dKDV[iGroup][i] = fabs(tSyncPara[iGroup].tPreHandle.dSetDis[i]/tSyncPara[iGroup].tVelPlan.dEVel[i]);
			//2.求解最大值		
			dMaxKDV = Max(dKDV[iGroup][i],dMaxKDV);			
		}	

		dMaxTAcc = Max(dMaxTAcc,tSyncPara[iGroup].tVelPlan.dTime[TACC]);
		dMaxTDec = Max(dMaxTDec,tSyncPara[iGroup].tVelPlan.dTime[TDEC]);

		dMaxTSmooth[PRE] = Max(dMaxTSmooth[PRE],tSyncPara[iGroup].tVelPlan.dTSmooth[PRE]);
		dMaxTSmooth[NEX] = Max(dMaxTSmooth[NEX],tSyncPara[iGroup].tVelPlan.dTSmooth[NEX]);
	}

	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		if(dMaxKDV > Eps)
		{
			for(int i = 0;i < MaxAxisNum;i++)
				tSyncPara[iGroup].tVelPlan.dEVel[i] = dKDV[iGroup][i]*tSyncPara[iGroup].tVelPlan.dEVel[i]/dMaxKDV;
		}

		tSyncPara[iGroup].tVelPlan.dTime[TACC] = dMaxTAcc;
		tSyncPara[iGroup].tVelPlan.dTime[TDEC] = dMaxTDec;
		tSyncPara[iGroup].tVelPlan.dTime[TCON] = dMaxKDV - dMaxTAcc/2 - dMaxTDec/2;
		tSyncPara[iGroup].tVelPlan.dTime[TALL] = dMaxTAcc + dMaxTDec + tSyncPara[iGroup].tVelPlan.dTime[TCON];

		tSyncPara[iGroup].tVelPlan.dTSmooth[PRE] = dMaxTSmooth[PRE];
		tSyncPara[iGroup].tVelPlan.dTSmooth[NEX] = dMaxTSmooth[NEX];
	}

	//下发
	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		m_HS_Int_Factory[iGroup]->SetSyncParaPlan(tSyncPara[iGroup]);
	}

	return 0;
}

int HS_AutoMove::execStopPlan()
{
    if(!m_bPlanFlag)
        return 0;
    LOG_ALGO("Stop Cmd!");

	if(m_bRestartAddFlag)
	{
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			int iRet = m_HS_Int_ReStartAdd[iGroup]->Stop();
			m_eStopState = S_STOP;			
		}
	}
	else if(m_bGroupPlanFlag||m_bSmoothPreFlag)
	{
		double dTStop = 0;
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			double dT = m_HS_Int_Factory[iGroup]->CalcStopTime();
			dTStop = Max(dT,dTStop);

			if(m_bSmoothPreFlag)
			{
				dT = m_HS_Int_FactoryPre[iGroup]->CalcStopTime();
				dTStop = Max(dT,dTStop);
			}					
		}

		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			int iRet = m_HS_Int_Factory[iGroup]->StopPlanByTime(dTStop);

			if(m_bSmoothPreFlag)
			{
				LOG_ALGO("Stop Move Pre!");
				m_HS_Int_FactoryPre[iGroup]->StopPlanByTime(dTStop);
			}
			m_eStopState = S_STOP;			
		}
	}
	else
	{
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			int iRet = m_HS_Int_Factory[iGroup]->Stop();

			if(iRet == 1)
			{
				LOG_ALGO("Small Line Stop Cmd OK!");
				m_eStopState = S_SLSTOP;
			}
			else
			{
				if(m_bSmoothPreFlag)
				{
					LOG_ALGO("Stop Move Pre!");
					m_HS_Int_FactoryPre[iGroup]->Stop();
				}
				m_eStopState = S_STOP;
			}
		}
	}
    return 0;
}

int HS_AutoMove::execStopRestartPlan(double dRatio,HS_GroupJPos &tRealJPos)
{
	int iErrorId = 0;
	m_dRestartRatio = dRatio;
	m_tRestartRealJPos = tRealJPos;

	RestartInit();
    LOG_ALGO("Stop Restart Plan!");

	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		LOG_ALGO("Group %d,RealJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",iGroup,
			tRealJPos.dJPos[iGroup][0],tRealJPos.dJPos[iGroup][1],tRealJPos.dJPos[iGroup][2],tRealJPos.dJPos[iGroup][3],tRealJPos.dJPos[iGroup][4],
			tRealJPos.dJPos[iGroup][5],tRealJPos.dJPos[iGroup][6],tRealJPos.dJPos[iGroup][7],tRealJPos.dJPos[iGroup][8]);

		if(!RepeatPosCheck(tRealJPos.dJPos[iGroup],m_tMoveDoneJPos.dJPos[iGroup]))
		{
			LOG_ALGO("Error Restart JPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",
				m_tMoveDoneJPos.dJPos[iGroup][0],m_tMoveDoneJPos.dJPos[iGroup][1],m_tMoveDoneJPos.dJPos[iGroup][2],m_tMoveDoneJPos.dJPos[iGroup][3],
				m_tMoveDoneJPos.dJPos[iGroup][4],m_tMoveDoneJPos.dJPos[iGroup][5],m_tMoveDoneJPos.dJPos[iGroup][6],m_tMoveDoneJPos.dJPos[iGroup][7],
				m_tMoveDoneJPos.dJPos[iGroup][8]);

			m_bPlanFlag = false;
			return ERROR_RESTART_EJPOS;
		}
	}

	iErrorId = WeaveRestartAddMove(dRatio,tRealJPos);
	if(iErrorId != 0) return iErrorId;

	if(!m_bRestartAddFlag)
		iErrorId = NormalRestart();

    return iErrorId;
}
/************************************************
函数功能：摆焊添加运行段运动
参    数：		
返 回 值：报警码
*************************************************/
int HS_AutoMove::WeaveRestartAddMove(double dRatio,HS_GroupJPos &tRealJPos)
{
	int iErrorId = 0;

	//摆焊，增加一段运动，回到停止对应的中间位置，即主运动位置，再进行恢复运动
	m_bRestartAddFlag = false;
	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		if(m_bWeaveStopFlag[iGroup]&&!RepeatPosCheck(tRealJPos.dJPos[iGroup],m_tWeaveStopJPos.dJPos[iGroup]))
		{
			LOG_ALGO("Restart Pos Error,Add Move!");
			GroupMotionData tGroupMotionData = {0};
			GroupTrajData tTrajData = {0};

			m_HS_Int_ReStartAdd[iGroup] = new HS_Int_Factory(MP_Line,m_HS_GroupKin);

			tGroupMotionData.bStartMove = true;
			tGroupMotionData.tBaseMoveData[iGroup].dAcc = m_tTrajData[m_iCurId].tMotionData.tBaseMoveData[iGroup].dAcc;
			tGroupMotionData.tBaseMoveData[iGroup].dDec = m_tTrajData[m_iCurId].tMotionData.tBaseMoveData[iGroup].dDec;
			tGroupMotionData.tBaseMoveData[iGroup].eTrajType = MP_Line;
			tGroupMotionData.iSmooth = m_tTrajData[m_iCurId].tMotionData.iSmooth;
			tGroupMotionData.tBaseMoveData[iGroup].dVel = m_tTrajData[m_iCurId].tMotionData.tBaseMoveData[iGroup].dVel;
			tGroupMotionData.tBaseMoveData[iGroup].dVort = m_tTrajData[m_iCurId].tMotionData.tBaseMoveData[iGroup].dVort;
			tGroupMotionData.tBaseMoveData[iGroup].sCurCoordinate = m_tTrajData[m_iCurId].tMotionData.tBaseMoveData[iGroup].sCurCoordinate;
			memcpy(tGroupMotionData.tBaseMoveData[iGroup].sStartPos.dPos,tRealJPos.dJPos[iGroup],sizeof(double)*MaxAxisNum);
			tGroupMotionData.tBaseMoveData[iGroup].sStartPos.hs_coordinate.iCoordinate = JOINT_COORD_SYSTEM;
			memcpy(tGroupMotionData.tBaseMoveData[iGroup].sEndPos.dPos,m_tWeaveStopJPos.dJPos[iGroup],sizeof(double)*MaxAxisNum);
			tGroupMotionData.tBaseMoveData[iGroup].sEndPos.hs_coordinate.iCoordinate = JOINT_COORD_SYSTEM;

			iErrorId = m_HS_Int_ReStartAdd[iGroup]->PreHandle(tGroupMotionData,&tTrajData,0,iGroup); 
			if(iErrorId != 0) return iErrorId;

			iErrorId = m_HS_Int_ReStartAdd[iGroup]->Plan(&tTrajData,0,iGroup,dRatio,tRealJPos);
			if(iErrorId != 0) return iErrorId;

			m_bRestartAddFlag = true;
		}
	}
	m_bPlanFlag = true;
	return iErrorId;
}
/************************************************
函数功能：复位重启
参    数：		
返 回 值：报警码
*************************************************/
int HS_AutoMove::NormalRestart()
{
	int iErrorId = 0;
	double dTAll = 0;

	double dRatio = m_dRestartRatio;
	HS_GroupJPos tRealJPos = m_tRestartRealJPos;

	m_bPlanFlag = true;
	if(m_bSmoothPreFlag)
	{
		LOG_ALGO("Smooth Move Restart!");
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;
			m_HS_Int_FactoryPre[iGroup]->RestartPlan(dRatio,tRealJPos,RST_SMOOTHPRE,dTAll);
			m_HS_Int_Factory[iGroup]->RestartPlan(dRatio,tRealJPos,RET_SMOOTHNEX,dTAll);
		}

		if(m_bGroupPlanFlag)
		{
			GroupSyncHandle();
		}
	}
	else
	{
		int iRet = 0;
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;
			iRet = m_HS_Int_Factory[iGroup]->RestartPlan(dRatio,tRealJPos,RST_NORMAL,dTAll);
		}
		if(iRet == 1)
		{
			//小线段规划停止时完成前一段运动，则当前段直接跳过
			m_bPlanFlag = false;
			LOG_ALGO("Small Line Stop Finish Done Restart Jump Cur Move!");
		}

		if(m_bGroupPlanFlag)
		{
			GroupSyncHandle();
		}
	}    

	DynLinePlanFilterStart(tRealJPos);
	return iErrorId;
}

int HS_AutoMove::setRatio(double ratio)
{
	printf("/----HS_AutoMove::setRatio1 %f----/\n",ratio);
    if(!m_bPlanFlag)
        return 0;
	//for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	//{
	//	if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
	//		continue;
	//	m_HS_Int_Factory[iGroup]->setRatio(ratio);
	//}

	double dRatioTime[TIMECNT] = {0};
	int iRatioState = 0;

	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		double dTime[TIMECNT] = {0};
		int iRet = m_HS_Int_Factory[iGroup]->GetRatioPara(ratio,dTime); 
		if(iRet != 0)
			iRatioState = iRet;
		if(dTime[TACC] > dRatioTime[TACC])
		{
			memcpy(dRatioTime,dTime,sizeof(double)*TIMECNT);
		}
	}

	for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
	{
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;
		printf("/----HS_AutoMove::setRatio2 %f----/\n",ratio);
		if(iRatioState == 0)
			m_HS_Int_Factory[iGroup]->SetRatioPara(ratio,dRatioTime,true); 
		else if(iRatioState == 1)
			m_HS_Int_Factory[iGroup]->SetRatioPara(ratio,dRatioTime,false); 
	}

	
	if(iRatioState != 2&&m_bGroupPlanFlag)
	{
		LOG_ALGO("Ratio OK[Group]:TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf",
			dRatioTime[TACC],dRatioTime[TCON],dRatioTime[TDEC]);
	}

    return 0;
}

	
HS_MStatus HS_AutoMove::execIntMove(IntData &tIntdata,int &iErrorId)
{
	m_bMoveOnFlag = true;

	if(m_bRestartAddFlag)
	{
		return RestartAddMove(tIntdata,iErrorId);
	}

    if(m_bSetPosErrorFlag)
    {
        HS_MStatus eMStatus = M_Busy;
        AutoPosCheck(tIntdata,eMStatus);
        
        return eMStatus;
    }
    if(!m_bPlanFlag)
    {
        bool bFilterDoneFlag = false;
        if(m_bFilterDoneFlag)
        {
            HS_MStatus eMStatus = GroupIntMove(tIntdata,iErrorId);
            if(m_bSLFiterFlag)
            {
                bFilterDoneFlag = SpaceFilterHandle(tIntdata,eMStatus);
            }
            else
            {
				for(int iGroup = 0;iGroup < MAXGROUPNUM;iGroup++)
				{
					if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
						continue;
					for(int i = 0;i < m_iInterMultCnt;i++)
					{
						bFilterDoneFlag = m_HS_FilterHandle->FilterHandle(tIntdata.tGJPos[i].dJPos[iGroup],tIntdata.tGJPos[i].dJPos[iGroup]);
					}
				}
            }
            if(bFilterDoneFlag)
            {
                m_bFilterDoneFlag = false;
                memcpy(&m_tMoveDoneJPos,&tIntdata.tGJPos[m_iInterMultCnt-1],sizeof(double)*MaxAxisNum);
            }
            return M_Busy;
        }
		m_bMoveOnFlag = false;
		return M_UnInit;
    }
    iErrorId = 0;

	IntData intdatePre;
	HS_MStatus eMStatusPre;
	if(m_bSmoothPreFlag)
	{
		eMStatusPre = GroupIntMove(intdatePre,iErrorId,true);     
	}
    HS_MStatus eMStatus = GroupIntMove(tIntdata,iErrorId);

    if(iErrorId != 0)
    {
        LOG_ALGO("IntMove Error!");
        LOG_ALGO("ErrorNum = %d",iErrorId);
        m_bPlanFlag = false;
        return M_Error;
    }
    //混合平滑
    if(m_bSmoothPreFlag)
    {   

		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;
			m_HS_Int_Factory[iGroup]->MixSmoothMove(intdatePre,tIntdata);
		}

        if(eMStatusPre == M_Done&&m_eStopState == S_NONE)
        {
            m_bSmoothPreFlag = false;
            LOG_ALGO("Pre Move Done!");
        }

		if(m_bSmoothJointFlag&&m_HS_Int_Factory[m_iBaseGroup]->GetHalfSmoothFlag()
			&&(m_eStopState == S_NONE||m_eStopState == S_SLSTOP))
		{
			m_bSmoothJointFlag = false;
			PushJoint(m_HS_Int_FactoryPre[m_iBaseGroup]->m_iLineNum);
			LOG_ALGO("Half Smooth!");
		}

		//平滑混合运动中，无需等待上一段Done，当前结束即可进行下一段
		//混合段正常停止，需等待前后两段均为Done信号
		//小线段的停止规划
        if(m_eStopState != S_NONE&&m_eStopState != S_SLSTOP)
        {
            if(eMStatusPre == M_Busy)
                eMStatus = M_Busy;
        }
    }

	if(!m_bSmoothAheadFlag)
	{
		if(m_HS_Int_Factory[m_iBaseGroup]->GetSmoothAheadFlag())
		{
			m_bSmoothAheadFlag = true;
			LOG_ALGO("Smooth Ahead Signal!");
		}
	}

    if(m_bSLFiterFlag)
    {
        SpaceFilterHandle(tIntdata,eMStatus);
    }

    if(m_tFilterControl->bFilterOpenFlag)
    {
        AutoFilterHandle(tIntdata,eMStatus);
    }
    iErrorId = AutoPosCheck(tIntdata,eMStatus);

    if(iErrorId != 0)
    {
        m_bSetPosErrorFlag = true;
    }

    if(eMStatus == M_Done)
    {
        MoveDoneHandle();
        memcpy(&m_tMoveDoneJPos,&tIntdata.tGJPos[m_iInterMultCnt-1],sizeof(double)*MaxAxisNum);
    }
	else if(eMStatus == M_StopDone||eMStatus == M_StopDone_F)
	{
		m_bPlanFlag = false;
		LOG_ALGO("Stop Done!");
		m_tMoveDoneJPos = tIntdata.tGJPos[m_iInterMultCnt-1];

		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			LOG_ALGO("Stop Done JPos Group %d = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",iGroup,
				m_tMoveDoneJPos.dJPos[iGroup][0],m_tMoveDoneJPos.dJPos[iGroup][1],m_tMoveDoneJPos.dJPos[iGroup][2],
				m_tMoveDoneJPos.dJPos[iGroup][3],m_tMoveDoneJPos.dJPos[iGroup][4],m_tMoveDoneJPos.dJPos[iGroup][5],
				m_tMoveDoneJPos.dJPos[iGroup][6],m_tMoveDoneJPos.dJPos[iGroup][7],m_tMoveDoneJPos.dJPos[iGroup][8]);

			//获取摆焊标识和停止位置
			m_bWeaveStopFlag[iGroup] = m_HS_Int_Factory[iGroup]->GetWeaveStopMainPos(m_tWeaveStopJPos.dJPos[iGroup]);

			if(m_bWeaveStopFlag[iGroup])
			{
				LOG_ALGO("Weave StopMainJPos Group %d = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",iGroup,
					m_tWeaveStopJPos.dJPos[iGroup][0],m_tWeaveStopJPos.dJPos[iGroup][1],m_tWeaveStopJPos.dJPos[iGroup][2],
					m_tWeaveStopJPos.dJPos[iGroup][3],m_tWeaveStopJPos.dJPos[iGroup][4],m_tWeaveStopJPos.dJPos[iGroup][5],
					m_tWeaveStopJPos.dJPos[iGroup][6],m_tWeaveStopJPos.dJPos[iGroup][7],m_tWeaveStopJPos.dJPos[iGroup][8]);
			}
		}

		if(m_eStopState == S_SLSTOP)
			m_bSmoothPreFlag = false;

		if(eMStatus == M_StopDone_F)
		{
			LOG_ALGO("Small Line Stop Done Finish Cur Move!");
			PushJoint(m_HS_Int_Factory[m_iBaseGroup]->m_iLineNum);
		}
	}

    return eMStatus;
}
/************************************************
函数功能：暂停重启添加段运动
参    数：		
		intdata-----插补数据
		iErrorId----报警码
返 回 值：运动规划状态
*************************************************/
HS_MStatus HS_AutoMove::RestartAddMove(IntData &tIntdata,int &iErrorId)
{
	//暂停重启增加运动保证点位同步
	HS_MStatus eMStatus = M_UnInit;

	double dMasterCPos[10][MaxAxisNum] = {0};

	int iMDoneCnt = 0;
	int iGroupNum = 0;

	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		iGroupNum++;

		eMStatus = m_HS_Int_ReStartAdd[iGroup]->execIntMove(tIntdata,iErrorId);

		if(eMStatus == M_Done)
			iMDoneCnt++;
	}

	if(iMDoneCnt == iGroupNum)
		eMStatus = M_Done;

	if(eMStatus == M_Done)
	{
		LOG_ALGO("Restart Add Move Done!");
		m_bRestartAddFlag = false;
		iErrorId = NormalRestart();
		eMStatus = M_Busy;
	}
	else if(eMStatus == M_StopDone)
	{
		m_bPlanFlag = false;
		LOG_ALGO("Stop Done!");
		m_tMoveDoneJPos = tIntdata.tGJPos[m_iInterMultCnt-1];

		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			LOG_ALGO("Stop Done JPos Group %d = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",iGroup,
				m_tMoveDoneJPos.dJPos[iGroup][0],m_tMoveDoneJPos.dJPos[iGroup][1],m_tMoveDoneJPos.dJPos[iGroup][2],
				m_tMoveDoneJPos.dJPos[iGroup][3],m_tMoveDoneJPos.dJPos[iGroup][4],m_tMoveDoneJPos.dJPos[iGroup][5],
				m_tMoveDoneJPos.dJPos[iGroup][6],m_tMoveDoneJPos.dJPos[iGroup][7],m_tMoveDoneJPos.dJPos[iGroup][8]);			
		}
	}

	return eMStatus;
}
/************************************************
函数功能：运动规划位置获取【轴组】
参    数：
		tIntdata-----运动中下发点位
		iErrorId-----运动完成，无点位下发
返 回 值：运动规划状态
*************************************************/
HS_MStatus HS_AutoMove::GroupIntMove(IntData &tIntdata,int &iErrorId,bool bPreFlag)
{
	HS_MStatus eMStatus = M_UnInit;
	HS_MStatus eMStatusNODone = M_UnInit;

	double dMasterCPos[10][MaxAxisNum] = {0};

	int iMDoneCnt = 0;
	int iGroupNum = 0;

	for(int iId = 0;iId < MAXGROUPNUM;iId++)
	{
		int iGroup = m_iGroupId[iId];
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
			continue;

		iGroupNum++;
		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_Slave)
		{
			if(bPreFlag)
				m_HS_Int_FactoryPre[iGroup]->SetMasterCPos(dMasterCPos);
			else
				m_HS_Int_Factory[iGroup]->SetMasterCPos(dMasterCPos);
		}

		if(bPreFlag)
			eMStatus = m_HS_Int_FactoryPre[iGroup]->execIntMove(tIntdata,iErrorId);
		else
			eMStatus = m_HS_Int_Factory[iGroup]->execIntMove(tIntdata,iErrorId);

		if(eMStatus == M_Done)
			iMDoneCnt++;
		else
			eMStatusNODone = eMStatus;

		if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_Master)
		{
			if(bPreFlag)
				m_HS_Int_FactoryPre[iGroup]->GetMasterCPos(dMasterCPos);
			else
				m_HS_Int_Factory[iGroup]->GetMasterCPos(dMasterCPos);
		}
	}

	if(iMDoneCnt == iGroupNum)
		eMStatus = M_Done;
	else
		eMStatus = eMStatusNODone;

	return eMStatus;
}

/************************************************
函数功能：获取平滑拐入点的提前信号量，提供给上层使用
参    数：
返 回 值：true-----拐入信号量
*************************************************/
bool HS_AutoMove::execGetSmoothAheadFlag()
{
	return m_bSmoothAheadFlag;
}

/************************************************
函数功能：获取运动插补下发点位状态
参    数：
返 回 值：true-----运动中下发点位
		false-----运动完成，无点位下发
*************************************************/
bool HS_AutoMove::execGetMoveOnFlag()
{
	return m_bMoveOnFlag;
}

int HS_AutoMove::MoveDoneHandle()
{
    int iErrorId = 0;
    switch(m_eStopState)
    {
    case S_NONE:
	case S_SLSTOP:
        m_bPlanFlag = false;
        m_bSmoothPreFlag = m_bSmoothNextFlag;
        if(m_bSmoothPreFlag)
        {
            if(m_HS_Int_Factory[m_iBaseGroup]->GetSmoothNextFlag())
			{
				m_bSmoothJointFlag = true;
                LOG_ALGO("Smooth Move Done!");
			}
            else
            {
                m_bSmoothPreFlag = false;
                LOG_ALGO("Smooth No Point Move Done!");
                PushJoint(m_HS_Int_Factory[m_iBaseGroup]->m_iLineNum);
				m_HS_Int_Factory[m_iBaseGroup]->GetSmoothNextFlag();
            }
        }
        else
        {
            LOG_ALGO("Move Done!");
            PushJoint(m_HS_Int_Factory[m_iBaseGroup]->m_iLineNum);
        }        
        break;
    case S_STOP:
        m_bPlanFlag = false;
        LOG_ALGO("Stop Done!");
        break;
    default:
        break;
    }
    return iErrorId;
}

int HS_AutoMove::AutoFilterHandle(IntData &intdata,HS_MStatus &eMStatus)
{
    int iErrorId = 0;
    bool bFilterDoneFlag = false;

    for(int i = 0;i < m_iInterMultCnt;i++)
    {
        //bFilterDoneFlag = m_HS_FilterHandle->FilterHandle(intdata.dPos[i],intdata.dPos[i]);
    }
    if(eMStatus == M_Done&&!m_HS_Int_Factory[m_iBaseGroup]->GetSmoothNextFlag())
    {
        if(m_iAutoFilterType == 0)
        {
            if(!bFilterDoneFlag)
                eMStatus = M_Busy;
        }
        else
        {
            m_bFilterDoneFlag = true;
        }
    }
    return iErrorId;
}

bool HS_AutoMove::SpaceFilterHandle(IntData &intdata,HS_MStatus &eMStatus)
{
    int iErrorId = 0;
    bool bFilterDoneFlag = false;
    double dJPos[MaxAxisNum] = {0};
    double dCPos[MaxAxisNum] = {0};

    for(int i = 0;i < m_iInterMultCnt;i++)
    {
       memcpy(dJPos,intdata.tGJPos[i].dJPos[m_iSLGroup],sizeof(double)*MaxAxisNum);
        m_HS_Kinematics->HS_JPosToCPos(dJPos,CP_ToolWork,dCPos);

        if(!m_bSLFilterInitFlag)
        {
            m_HS_Kinematics->EnlerNearstHandle(m_dSLCPos,dCPos);
        }
        else
            m_bSLFilterInitFlag = false;

        memcpy(m_dSLCPos,dCPos,sizeof(double)*MaxAxisNum);
        bFilterDoneFlag = m_HS_SpaceFilter->FilterHandle(dCPos,dCPos);

        m_HS_Kinematics->HS_CPosToJPos(dCPos,CP_ToolWork,m_dSFilterRJPos,dJPos);
        dJPos[6] = dCPos[6];
        dJPos[7] = dCPos[7];
        dJPos[8] = dCPos[8];

        memcpy(m_dSFilterRJPos,dJPos,sizeof(double)*MaxAxisNum);
        memcpy(intdata.tGJPos[i].dJPos[m_iSLGroup],dJPos,sizeof(double)*MaxAxisNum);
    }
    if((eMStatus == M_Done||eMStatus == M_StopDone||eMStatus == M_StopDone_F)&&!m_HS_Int_Factory[m_iBaseGroup]->GetSmoothNextFlag())
    {
        m_bFilterDoneFlag = true;        
    }
    return bFilterDoneFlag;
}


int HS_AutoMove::AutoPosCheck(IntData &intdata,HS_MStatus &eMStatus)
{
    int iRet = 0;
	int iErrorId = 0;

    HS_MStatus eStausCheck = M_Busy;
	bool bErrorFlag = false;

    for(int i = 0;i < m_iInterMultCnt;i++)
    {
		for(int iId = 0;iId < MAXGROUPNUM;iId++)
		{
			int iGroup = m_iGroupId[iId];
			if(m_tHS_GroupRel.eGroupRelType[iGroup] == GRT_NoUse)
				continue;

			iErrorId = m_HS_SetPosCheck[iGroup]->SetJPos(intdata.tGJPos[i].dJPos[iGroup],eStausCheck);			

			if(iErrorId != 0&&!m_bSetPosErrorFlag&&!bErrorFlag)
			{			
				bErrorFlag = true;
				if(iErrorId == ERROR_VMAX)
				{
					if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Line)
					{
						iErrorId = E_L_MOVEOVERJVEL;
					}
					else if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Arc)
					{
						iErrorId = E_C_MOVEOVERJVEL;
					}
				}
				else if(iErrorId == ERROR_QY_WRIST)
				{
					if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Line)
					{
						iErrorId = E_L_MOVEQYWRIST;
					}
					else if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Arc)
					{
						iErrorId = E_C_MOVEQYWRIST;
					}
				}
				else if(iErrorId == ERROR_QY_INSIDE)
				{
					if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Line)
					{
						iErrorId = E_L_MOVEQYINSIDE;
					}
					else if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Arc)
					{
						iErrorId = E_C_MOVEQYINSIDE;
					}
				}
				else if(iErrorId == ERROR_QY_BORDER)
				{
					if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Line)
					{
						iErrorId = E_L_MOVEQYBORDER;
					}
					else if(m_HS_Int_Factory[iGroup]->m_Hs_MoveType == MP_Arc)
					{
						iErrorId = E_C_MOVEQYBORDER;
					}
				}
				LOG_ALGO("SetJPos Check Error!");
				LOG_ALGO("ErrorNum = %d,GroupNum = %d",iErrorId,iGroup);
				if(iErrorId != 0)
					iRet = iErrorId;

				for(int iGroupOther = 0;iGroupOther < MAXGROUPNUM;iGroupOther++)
				{
					if(m_tHS_GroupRel.eGroupRelType[iGroupOther] == GRT_NoUse)
						continue;

					if(iGroupOther != iGroup)
					{
						m_HS_SetPosCheck[iGroupOther]->QuickStop();
					}
				}				
			}
		}
    }
    
    return iRet;
}
/************************************************
函数功能：检测两个关节点位坐标是否为重复点位
参    数：dJPosA---点位A
         dJPosB---点位B
返 回 值：true-----重复点位
         false----非重复点位
*************************************************/
bool HS_AutoMove::RepeatPosCheck(double *dJPosA,double *dJPosB)
{
	PosInfo tPosA;
	PosInfo tPosB;
	tPosA.hs_coordinate.iCoordinate = JOINT_COORD_SYSTEM;
	tPosB.hs_coordinate.iCoordinate = JOINT_COORD_SYSTEM;
	memcpy(tPosA.dPos,dJPosA,sizeof(double)*MaxAxisNum);
	memcpy(tPosB.dPos,dJPosB,sizeof(double)*MaxAxisNum);

	return m_HS_Kinematics->HS_RepeatPosCheck(tPosA,tPosB);    
}
/************************************************
函数功能：写入Joint信息
         *场景1：无平滑完成
         *场景2：平滑输出
         *场景3：有平滑无下一段运动
         *场景4：重复点位
参    数：
         iJointNum---Joint行号
返 回 值：无
*************************************************/
void HS_AutoMove::PushJoint(int iJointNum)
{
    m_iJointBuff[m_iJointBuffIndex] = iJointNum;
    LOG_ALGO("Output Joint [%d] = %d",m_iJointBuffIndex,iJointNum);

    m_iJointBuffIndex = (m_iJointBuffIndex + 1)%MaxJointBuffCnt;
}

/************************************************
函数功能：获取Joint信息
参    数：
         joint---Joint行号Buff[4]
返 回 值：无
*************************************************/
int HS_AutoMove::execGetJoint(int *joint)
{
    memcpy(joint,m_iJointBuff,sizeof(int)*MaxJointBuffCnt);

    //清空数据
    m_iJointBuffIndex = 0;
    memset(m_iJointBuff,-1,sizeof(m_iJointBuff));

    return 0;
}

/************************************************
函数功能：轴组Id排序，主运动在前
参    数：
返 回 值：无
*************************************************/
int HS_AutoMove::GroupIdSort(HS_GroupRel tHS_GroupRel)
{
	for(int i = 0;i < MAXGROUPNUM;i++)
		m_iGroupId[i] = i;

	for(int i = 0;i < MAXGROUPNUM;i++)
	{
		if(tHS_GroupRel.eGroupRelType[i] == GRT_Master&&i != 0)
		{
			m_iGroupId[i] = 0;
			m_iGroupId[0] = i;
			break;
		}
	}
	return 0;
}

/************************************************
函数功能：获取当前运行的行id号
参    数：
返 回 值：返回行id号
*************************************************/
int HS_AutoMove::execGetCurMoveId()
{
	return m_iCurId;
}


bool HS_AutoMove::HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB)
{
	return m_HS_Kinematics->HS_RepeatPosCheck(tPosA,tPosB);
}