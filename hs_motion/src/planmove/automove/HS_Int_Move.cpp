/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Move.h
* 摘    要：插补运动工厂类

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*			
*/
#include "HS_Int_Move.h"
#include "HS_Kinematics.h"

SyncPara HS_Int_Move::m_tSyncAhead[MAXGROUPNUM][MAXLOOKAHEADSLINE+1] = {SyncPara()};                  
int HS_Int_Move::m_iLookAhead[MAXGROUPNUM] = {0};
double HS_Int_Move::m_dTSmoothOff[MAXGROUPNUM] = {0};
bool HS_Int_Move::m_bSLPlanFlag = false;                                     //小线段规划标识
int HS_Int_Move::m_iMaxLookAheadLine = 1;
int HS_Int_Move::m_iDynIndex = -1;  
bool HS_Int_Move::m_bSmoothMoveFlag[MAXGROUPNUM];		
bool HS_Int_Move::m_bSmoothSynthFlag[MAXGROUPNUM] = {0};
bool HS_Int_Move::m_bWristQYHandleFlag[MAXGROUPNUM];	
double HS_Int_Move::m_dLMPos[10][MAXGROUPNUM][5][4];

HS_Int_Move::HS_Int_Move()
{
    m_HS_BasicPara = HS_BasicPara::GetInstance();

    m_dCycle = m_HS_BasicPara->m_dCycle;
    m_iInterMultCnt = m_HS_BasicPara->m_iInterMultCnt;
    //m_tLimitPara = &m_HS_BasicPara->mMotionPara->m_tLimitPara;

    m_HS_VelPlan_Para = new HS_VelPlan_Para *[MaxAxisNum];
    for(int i = 0;i < MaxAxisNum;i++)
    {
        m_HS_VelPlan_Para[i] = new HS_VelPlan_Para(m_dCycle);
    }

    //空间运动的基础速度、加速度参数值【自适应使用】
	m_dCVelPara[0] = 1500;
	m_dCVelPara[1] = 500;
	m_dCAccPara[0] = 4000;
	m_dCAccPara[1] = 3000;

    m_dTFreProtect = 0;
    m_dTAcc = 0;
    m_dTAcc = 0;
    m_dTCon = 0;
    m_dTDec = 0;
    m_dTAll = 0;
    m_dTCur = 0;
    memset(m_iLookAhead,0,sizeof(m_iLookAhead));
	memset(m_dTSmoothOff,0,sizeof(m_dTSmoothOff));
    memset(m_tSyncAhead,0,sizeof(m_tSyncAhead));

    m_bSmoothPreFlag = false;
    m_bSmoothNextFlag = false;
    m_bStopFlag = false;
    m_bDynSmoothFlag = false;
    m_bSLPlagFlag = false;
    m_eRstState = RST_NORMAL;
    m_dTRSTPreAll = 0;
    m_bWristQYFlag = false;

    memset(m_dMoveRatioPos,0,sizeof(m_dMoveRatioPos));
    memset(m_dMovePos,0,sizeof(m_dMovePos));
    memset(m_dMoveVel,0,sizeof(m_dMoveVel));
    memset(m_dMoveAcc,0,sizeof(m_dMoveAcc));
    memset(m_dRJPos,0,sizeof(m_dRJPos));
    memset(m_dStopSPos,0,sizeof(m_dStopSPos));
    memset(&m_tSync,0,sizeof(m_tSync));
    memset(m_dStopPos,0,sizeof(double)*MaxAxisNum);
	memset(m_dMasterCPos,0,sizeof(m_dMasterCPos));
	memset(&m_tHS_GroupRel,0,sizeof(m_tHS_GroupRel));
	memset(&m_bSmoothMoveFlag,0,sizeof(m_bSmoothMoveFlag));
	memset(m_dWeaveStopMainJPos,0,sizeof(m_dWeaveStopMainJPos));

    m_iToolNum = 0;
    m_iWorkNum = 0;
    m_dKVelMax = 1.0;
}

HS_Int_Move::~HS_Int_Move()
{
    for(int i = 0;i < MaxAxisNum;i++)
        delete m_HS_VelPlan_Para[i];
    delete[] m_HS_VelPlan_Para; 
    m_HS_VelPlan_Para = NULL;
}
/************************************************
函数功能：点位运动混合
参    数：intdataPre----前一段插补点位
         intdata-------当前段插补点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::MixSmoothMove(IntData &intdataPre,IntData &intdata)
{
	if(!m_bSmoothSynthFlag[m_iGroupNum])
	{
		for(int i = 0;i < m_iInterMultCnt;i++)
		{
			for(int j = 0;j < MaxAxisNum;j++)
			{
				intdata.tGJPos[i].dJPos[m_iGroupNum][j] = (intdata.tGJPos[i].dJPos[m_iGroupNum][j] - m_tSync.tPreHandle.dSetJPos[0][j]) 
					+ intdataPre.tGJPos[i].dJPos[m_iGroupNum][j];
			}
		}
	}
    return 0;
}

/************************************************
函数功能：提取设置的运动规划信息，按照不同的运动类型进行提取处理【通用接口】
参    数：trajout----运动对象
         index------索引号
		 iGroupNum---轴组号
         tPreHandle---预处理信息
         dRatio------倍率
返 回 值：错误码
*************************************************/
int HS_Int_Move::GetInputParaGE(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPreHandle,double dRatio)
{
    int iErrorId = 0;

    memcpy(&tPreHandle,tTrajData[iIndex].iData[iGroupNum],sizeof(Para_PreHandle));  

	HS_MOVETYPE eTrajType = tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].eTrajType;          

    if(eTrajType == MP_Line||eTrajType == MP_Arc||eTrajType == MP_BLine)
    {
        tPreHandle.dSetVel[0] = tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVel*dRatio*tPreHandle.dSetKVel[0];
        tPreHandle.dSetVel[1] = m_dCVelPara[1]*tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVort/100*dRatio*tPreHandle.dSetKVel[1];
        for(int i = 0;i < 2;i++)
        {            
            tPreHandle.dSetAcc[i] = m_dCAccPara[i]*tPreHandle.dSetMaxAcc*tPreHandle.dSetKAcc[0]*dRatio;
            tPreHandle.dSetDec[i] = m_dCAccPara[i]*tPreHandle.dSetMaxAcc*tPreHandle.dSetKAcc[1]*dRatio;
            if(iIndex == m_iIndex)
                tPreHandle.dSetDis[i] = tPreHandle.dOrigDis[i] - m_dStopPos[i];
            else
                tPreHandle.dSetDis[i] = tPreHandle.dOrigDis[i];
        }
        for(int i = 2;i < SpaceAxisNum;i++)
        {
            tPreHandle.dSetVel[i] = m_dJVelPara[i+4]*dRatio;
            tPreHandle.dSetAcc[i] = m_dJAccPara[i+4]*dRatio*tPreHandle.dSetMaxAcc;
            tPreHandle.dSetDec[i] = m_dJAccPara[i+4]*dRatio*tPreHandle.dSetMaxAcc;
            if(iIndex == m_iIndex)
                tPreHandle.dSetDis[i] = tPreHandle.dOrigDis[i] - m_dStopPos[i];
            else
                tPreHandle.dSetDis[i] = tPreHandle.dOrigDis[i];
        }
    }
    else if(eTrajType == MP_Joint)
    {
        for(int i = 0;i < MaxAxisNum;i++)
        {
            tPreHandle.dSetVel[i] = m_dJVelPara[i]*dRatio*tPreHandle.dSetKVel[i];
            tPreHandle.dSetAcc[i] = m_dJAccPara[i]*dRatio*tPreHandle.dSetMaxAcc*tPreHandle.dSetKAcc[0];		
            tPreHandle.dSetDec[i] = m_dJAccPara[i]*dRatio*tPreHandle.dSetMaxAcc*tPreHandle.dSetKAcc[1];

            if(iIndex == m_iIndex)
                tPreHandle.dSetDis[i] = tPreHandle.dDis[i] - m_dStopPos[i];
            else
                tPreHandle.dSetDis[i] = tPreHandle.dDis[i];
        }
    }

    tPreHandle.dRadius[ORIGNEX] = tTrajData[iIndex].tMotionData.dCnt;
    return iErrorId;
}

/************************************************
函数功能：全局公共的参数检测，防止参数异常等
参    数：
		tMotionData----待检测的数据结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GlobalParaCheck(GroupMotionData &tMotionData)
{
	int iErrorId = 0;

	return iErrorId;
}
/************************************************
函数功能：全局公共的预处理规划
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GlobalPreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPreHandle,BaseMoveData *tMoveData)
{
	int iErrorId = 0;

	tTrajData[iIndex].tMotionData = tGroupMotionData;

	iErrorId = GlobalParaCheck(tTrajData[iIndex].tMotionData);

	m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate; 

	memset(&tPreHandle,0,sizeof(tPreHandle));
	tPreHandle.eMoveType = m_eMoveType;
	tPreHandle.bPreHandled = true;
	m_tGTrajData = tTrajData;
	m_iIndex = iIndex;
	m_iGroupNum = iGroupNum;
	tPreHandle.bWristQYOpenFlag = tTrajData[iIndex].tMotionData.bWristQYFlag;

	if(m_eMoveType == MP_Line||m_eMoveType == MP_Arc)
	{
		if(tGroupMotionData.dCR - 1.0 > -Eps)
		{
			tTrajData[iIndex].tMotionData.dCnt = tGroupMotionData.dCR;
			tPreHandle.bSmoothCRMode = true;
		}	

		if(tGroupMotionData.tBaseMoveData[iGroupNum].bCoorperMove)
			tPreHandle.bCoorperMoveFlag = true;

	}

	m_HS_Kinematic = m_HS_GroupKin->GetKinematicsByNum(iGroupNum);
	m_eHS_RobotType = m_HS_Kinematic->GetRobotType();

	GetToolWorkNum(*tMoveData);
	GroupSyncCheck(tGroupMotionData.tHS_GroupRel,tPreHandle);

	return 0;
}
/************************************************
函数功能：全局公共的规划处理
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GlobalPlan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	int iErrorId = 0;

	if(tTrajData[iIndex].tMotionData.bStartMove||m_eMoveType != MP_BLine)
	{
		m_iDynIndex = -1;
	}

	memset(&m_tSync,0,sizeof(m_tSync));

	m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate;

	memset(m_dStopPos,0,sizeof(double)*MaxAxisNum);
	m_eRstState = RST_NORMAL;
	m_iGroupNum = iGroupNum;
	m_HS_Kinematic = m_HS_GroupKin->GetKinematicsByNum(iGroupNum);
	m_tHS_GroupRel = tTrajData[iIndex].tMotionData.tHS_GroupRel;

	std::string strGroupType[] = {"NoUse","Independent","Master","Slave","Slave2"};

	if(m_eMoveType == MP_Joint)
		LOG_ALGO("Plan Move Joint:Id = %d,LineNum = %d,GroupNum = %d,GroupType = %s!",iIndex,tTrajData[iIndex].tMotionData.iLineNum,iGroupNum,strGroupType[int(m_tHS_GroupRel.eGroupRelType[m_iGroupNum])].c_str());
	else if(m_eMoveType == MP_Line)
		LOG_ALGO("Plan Move Line:Id = %d,LineNum = %d,GroupNum = %d,GroupType = %s!",iIndex,tTrajData[iIndex].tMotionData.iLineNum,iGroupNum,strGroupType[int(m_tHS_GroupRel.eGroupRelType[m_iGroupNum])].c_str());
	else if(m_eMoveType == MP_BLine)
		LOG_ALGO("Plan Move BLine:Id = %d,LineNum = %d,GroupNum = %d,GroupType = %s!",iIndex,tTrajData[iIndex].tMotionData.iLineNum,iGroupNum,strGroupType[int(m_tHS_GroupRel.eGroupRelType[m_iGroupNum])].c_str());
	else if(m_eMoveType == MP_Arc)
		LOG_ALGO("Plan Move Circle:Id = %d,LineNum = %d,GroupNum = %d,GroupType = %s!",iIndex,tTrajData[iIndex].tMotionData.iLineNum,iGroupNum,strGroupType[int(m_tHS_GroupRel.eGroupRelType[m_iGroupNum])].c_str());


	memcpy(m_dRealSJPos,tHS_GroupJPos.dJPos[iGroupNum],sizeof(double)*MaxAxisNum);

	LOG_ALGO("RealJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_dRealSJPos[0],m_dRealSJPos[1],m_dRealSJPos[2],
		m_dRealSJPos[3],m_dRealSJPos[4],m_dRealSJPos[5],
		m_dRealSJPos[6],m_dRealSJPos[7],m_dRealSJPos[8]);

	m_bJPosCorrectFlag = false;

	iErrorId = CheckPreHandle(tTrajData,iIndex); 
	if(iErrorId != 0) return iErrorId;  

	m_bTriggerSmoothFlag = false;
	m_bTimeInNextSmooth = false;

	return iErrorId;
}
/************************************************
函数功能：根据时间求解对应点位的加速度比例信息【直线、圆弧】
参    数：dTime--------待求点时间
		 tSyncPara----规划信息
		 dKAcc--------加速度比例信息
         dKVel--------速度比例值
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetKAccScaleByTime(double dTime,SyncPara &tSyncPara,double &dKAcc,double &dKVel)
{
	int iErrorId = 0;
    
	double dJAcc[MaxAxisNum] = {0};
    double dJVel[MaxAxisNum] = {0};

    iErrorId = GetJParaByTime(dTime,tSyncPara,dJAcc,dJVel);	
	//比例求解 	
    dKAcc = 0;
    dKVel = 0;
	for(int i = 0;i < MaxAxisNum;i++)
	{
		double dAccTemp = 0;
        if(fabs(m_dJAccPara[i]) > Eps)
            dAccTemp = fabs(dJAcc[i]/m_dJAccPara[i]);
		double dKAccSet = tSyncPara.tPreHandle.dSetMaxAcc;		
		dAccTemp = dAccTemp/dKAccSet;		
		dKAcc = Max(dKAcc,dAccTemp);

        double dVelTemp = 0;
        if(fabs(m_dJVelPara[i]) > Eps)
            dVelTemp = fabs(dJVel[i]/m_dJVelPara[i]);
        dKVel = Max(dKVel,dVelTemp);
	}		
	return iErrorId;
}

/************************************************
函数功能：根据时间求解对应点位的加速度比例信息【直线、圆弧】
参    数：dTime--------待求点时间
		 tSyncPara----规划信息
		 dJAcc--------加速度各轴
         dJVel--------速度各轴
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetJParaByTime(double dTime,SyncPara &tSyncPara,double dJAcc[MaxAxisNum],double dJVel[MaxAxisNum])
{
    int iErrorId = 0;
    double dPLanPos[3][SpaceAxisNum] = {0};	
    double dTAdd = 0.001;
    double dTCalc = dTime - dTAdd;

    for(int i = 0;i < 3;i++)
    {
        GetPlanPos(tSyncPara,dTCalc,dPLanPos[i]);
        dTCalc += dTAdd;
    }

    double dMPos[3][5][4] = {0};
    for(int i = 0;i < 3;i++)	
        PlanPos2MPos(tSyncPara,dPLanPos[i],dMPos[i]);	

    double dJPos[3][MaxAxisNum] = {0};

	if(tSyncPara.tPreHandle.bWristQyFlag&&m_eHS_RobotType == HSROB_PUMA)
	{
		for(int i = 0;i < 3;i++)
		{
			memcpy(dJPos[i],tSyncPara.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
		}
	}

    iErrorId = m_HS_Kinematic->HS_MPosToJPos_JXJ(dMPos[0],m_iToolNum,m_iWorkNum,m_eState,CP_ToolWork,dJPos[0],tSyncPara.tPreHandle.bWristQyFlag);
	dJPos[0][6] = dMPos[0][4][0]; dJPos[0][7] = dMPos[0][4][1]; dJPos[0][8] = dMPos[0][4][2];

    if(iErrorId != 0) 
    {
        LOG_ALGO("Error C2J!,CPos = %.3lf,%.3lf,%.3lf",dMPos[0][0][3],dMPos[0][1][3],dMPos[0][2][3]);
        return ERROR_CPOSTOJPOS;
    }

    m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[1],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[1],tSyncPara.tPreHandle.bWristQyFlag);
	dJPos[1][6] = dMPos[1][4][0]; dJPos[1][7] = dMPos[1][4][1]; dJPos[1][8] = dMPos[1][4][2];

    m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[2],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[1],dJPos[2],tSyncPara.tPreHandle.bWristQyFlag);	
	dJPos[2][6] = dMPos[2][4][0]; dJPos[2][7] = dMPos[2][4][1]; dJPos[2][8] = dMPos[2][4][2];

    for(int i = 0;i < MaxAxisNum;i++)
    {
        dJAcc[i] = (dJPos[0][i] + dJPos[2][i] - 2*dJPos[1][i])/dTAdd/dTAdd;	
        dJVel[i] = (dJPos[1][i] - dJPos[0][i])/dTAdd;	
    }
    return iErrorId;
}
/************************************************
函数功能：根据时间求解对应点位的捷度比例信息
参    数：dTime--------待求解时刻
		 tSyncPara----规划信息
		 dKJerkScale--捷度比例信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetKVelScaleByTime(double dTime,SyncPara &tSyncPara,double dKVel[MaxAxisNum])
{
	int iErrorId = 0;
	double dPlanPos[2][SpaceAxisNum] = {0};
	double dTAdd = 0.001;
    double dTCalc = dTime - dTAdd;

	GetPlanPos(tSyncPara,dTCalc,dPlanPos[0]);
	GetPlanPos(tSyncPara,dTCalc+dTAdd,dPlanPos[1]);

	double dMPos[2][5][4] = {0};
	for(int i = 0;i < 2;i++)
	{
		PlanPos2MPos(tSyncPara,dPlanPos[i],dMPos[i]);
	}

	//求解关节坐标
	double dJPos[2][MaxAxisNum] = {0};

	if(tSyncPara.tPreHandle.bWristQyFlag&&m_eHS_RobotType == HSROB_PUMA)
	{
		for(int i = 0;i < 2;i++)
		{
			memcpy(dJPos[i],tSyncPara.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
		}
	}

	iErrorId = m_HS_Kinematic->HS_MPosToJPos_JXJ(dMPos[0],m_iToolNum,m_iWorkNum,m_eState,CP_ToolWork,dJPos[0],tSyncPara.tPreHandle.bWristQyFlag);
	if(iErrorId != 0) 
    {
        LOG_ALGO("Error C2J!,CPos = %.3lf,%.3lf,%.3lf",dMPos[0][0][3],dMPos[0][1][3],dMPos[0][2][3]);
        return ERROR_CPOSTOJPOS;
    }

    m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[1],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[1],tSyncPara.tPreHandle.bWristQyFlag);

	//速度求解
	double dJVel[MaxAxisNum] = {0};
	for(int i = 0;i < 6;i++)
	{
		dJVel[i] = (dJPos[1][i] - dJPos[0][i])/dTAdd;
	}
	
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dKVel[i] = fabs(dJVel[i]/m_dJVelPara[i]);	
        dKVel[i] /= m_dKVelMax;
	}

	return iErrorId;
}
/************************************************
函数功能：根据时间求解得到对应的规划点位信息
参    数：tSyncPara---规划信息
		 dTime-------待求解时刻
		 dPlanPos----转换得到规划点位信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetPlanPos(SyncPara &tSyncPara,double dTime,double dPlanPos[SpaceAxisNum])
{	
	double dKaAcc[SpaceAxisNum] = {0};
	double dKbAcc[SpaceAxisNum] = {0};

	double dTAcc = tSyncPara.tVelPlan.dTime[TACC];
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		dKaAcc[i] = -tSyncPara.tVelPlan.dEVel[i]/(2*dTAcc*dTAcc*dTAcc);
		dKbAcc[i] = tSyncPara.tVelPlan.dEVel[i]/(dTAcc*dTAcc);	
	}

	double dKaDec[SpaceAxisNum] = {0};
	double dKbDec[SpaceAxisNum] = {0};
	double dTDec= tSyncPara.tVelPlan.dTime[TDEC];

	for(int i = 0;i < SpaceAxisNum;i++)
	{		
		dKaDec[i] = -tSyncPara.tVelPlan.dEVel[i]/(2*dTDec*dTDec*dTDec);
		dKbDec[i] = tSyncPara.tVelPlan.dEVel[i]/(dTDec*dTDec);	
	}
	
	if(dTime < Eps)
	{
		for(int i = 0;i < SpaceAxisNum;i++)
			dPlanPos[i] = 0;
	}
	else if(dTime < dTAcc)
	{
		for(int i = 0;i < SpaceAxisNum;i++)
			dPlanPos[i] = dTime*dTime*dTime*(dKaAcc[i]*dTime + dKbAcc[i]);
	}
	else if(dTime < dTAcc + tSyncPara.tVelPlan.dTime[TCON] - Eps)
	{
		for(int i = 0;i < SpaceAxisNum;i++)
			dPlanPos[i] = tSyncPara.tVelPlan.dEVel[i]*dTAcc/2 + tSyncPara.tVelPlan.dEVel[i]*(dTime - dTAcc);
	}
	else if(dTime < tSyncPara.tVelPlan.dTime[TALL])
	{		
		dTime = tSyncPara.tVelPlan.dTime[TALL] - dTime;
		for(int i = 0;i < SpaceAxisNum;i++)
			dPlanPos[i] = tSyncPara.tPreHandle.dSetDis[i] - dTime*dTime*dTime*(dKaDec[i]*dTime + dKbDec[i]);		
	}	
	else
	{
		for(int i = 0;i < SpaceAxisNum;i++)
			dPlanPos[i] = tSyncPara.tPreHandle.dSetDis[i];
	}

	return 0;
}
/************************************************
函数功能：将规划得到的点位转换成M点位
参    数：tSyncPara---规划信息
		 dPlanPos----运动点位信息
		 dMPos-------转换得到M矩阵点位信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::PlanPos2MPos(SyncPara &tSyncPara,double dPlanPos[SpaceAxisNum],double dMPos[5][4])
{
	HS_Math::Matrix_Eye(4,&dMPos[0][0]);

    if(tSyncPara.tPreHandle.eMoveType == MP_Line)
    {
	    for(int i = 0;i < 3;i++)
	    {
		    dMPos[i][3] = tSyncPara.tPreHandle.dSPos[i] + (dPlanPos[0])*tSyncPara.tPreHandle.dKXYZ[i];
	    }
	    //姿态矩阵
	    double dSMPos[4][4] = {0};
	    m_HS_Kinematic->HS_CPosToMPos(tSyncPara.tPreHandle.dSPos,dSMPos);
	    //当前四元数
	    double dQ[4] = {0};
	    memcpy(dQ,tSyncPara.tPreHandle.dQ,sizeof(double)*4);
	    dQ[0] = angle2Rad(dPlanPos[1]);
	    double dQMPos[4][4] = {0};	

	    HS_Math::Matrix_AToM(dQ,dQMPos);
	    HS_Math::Matrix_Multi(4,3,&dSMPos[0][0],&dQMPos[0][0],&dMPos[0][0]);

        for(int i = 0;i < 3;i++)
        {
            dMPos[4][i] = tSyncPara.tPreHandle.dSPos[6+i] + dPlanPos[2+i]; 
        }
    }
    else if(tSyncPara.tPreHandle.eMoveType == MP_Arc)
    {
        double dCircleR = tSyncPara.tPreHandle.dCircleR;

        double dPos_C[4] = {dCircleR*cos((dPlanPos[0])/dCircleR),dCircleR*sin((dPlanPos[0])/dCircleR),0,1};
        double dPos_A[4] = {0};
        HS_Math::Matrix_Multi(4,4,1,&tSyncPara.tPreHandle.dMTrans[0][0],dPos_C,dPos_A);

        for(int i = 0;i < 3;i++)
        {
            dMPos[i][3] = dPos_A[i];
        }
        //姿态矩阵
        double dSMPos[4][4] = {0};
        m_HS_Kinematic->HS_CPosToMPos(tSyncPara.tPreHandle.dSPos,dSMPos);
        //当前四元数
        double dQ[4] = {0};
        memcpy(dQ,tSyncPara.tPreHandle.dQ,sizeof(double)*4);
        dQ[0] = angle2Rad(dPlanPos[1]);
        double dQMPos[4][4] = {0};	
        HS_Math::Matrix_AToM(dQ,dQMPos);
        HS_Math::Matrix_Multi(4,3,&dSMPos[0][0],&dQMPos[0][0],&dMPos[0][0]);

        for(int i = 0;i < 3;i++)
        {
            dMPos[4][i] = tSyncPara.tPreHandle.dSPos[6+i] + dPlanPos[2+i]; 
        }
    }

	return 0;
}

/************************************************
函数功能：将规划得到的点位转换成M点位【直线合成】
参    数：tSyncCur---规划信息【当前段】
		 tSyncNex---规划信息【下一段】
		 dPlanPosCur----运动点位信息【当前段】
		 dPlanPosNex----运动点位信息【下一段】
		 dMPosMix-------转换得到M矩阵点位信息【合成】
返 回 值：错误ID
*************************************************/
int HS_Int_Move::PlanPos2MPos(SyncPara &tSyncCur,SyncPara &tSyncNex,double dPlanPosCur[SpaceAxisNum],double dPlanPosNex[SpaceAxisNum],double dMPosMix[5][4])
{
	//转化为相应的位置
	//位置轴
	for(int i = 0;i < 3;i++)
	{
		dMPosMix[i][3] = tSyncCur.tPreHandle.dSPos[i] + dPlanPosCur[0]*tSyncCur.tPreHandle.dKXYZ[i]	+ dPlanPosNex[0]*tSyncNex.tPreHandle.dKXYZ[i];
	}

	//姿态矩阵
	double dSMPosPre[4][4] = {0};
	m_HS_Kinematic->HS_CPosToMPos(tSyncCur.tPreHandle.dSPos,dSMPosPre);
	//前一段四元数
	double dQPre[4] = {0};
	memcpy(dQPre,tSyncCur.tPreHandle.dQ,sizeof(double)*4);
	dQPre[0] = angle2Rad(dPlanPosCur[1]);
	double dQMPosPre[4][4] = {0};	
	double dRMPos[4][4] = {0};
	HS_Math::Matrix_AToM(dQPre,dQMPosPre);
	HS_Math::Matrix_Multi(4,4,4,&dSMPosPre[0][0],&dQMPosPre[0][0],&dRMPos[0][0]);
	//当前四元数
	double dQ[4] = {0};
	memcpy(dQ,tSyncNex.tPreHandle.dQ,sizeof(double)*4);
	dQ[0] = angle2Rad(dPlanPosNex[1]);
	double dQMPos[4][4] = {0};	
	HS_Math::Matrix_AToM(dQ,dQMPos);
	HS_Math::Matrix_Multi(4,3,&dRMPos[0][0],&dQMPos[0][0],&dMPosMix[0][0]);

	dMPosMix[3][3] = 1.0;

    for(int i = 0;i < 3;i++)
    {
        dMPosMix[4][i] = tSyncCur.tPreHandle.dSPos[6+i] + dPlanPosCur[2+i] + dPlanPosNex[2+i]; 
    }

	return 0;
}

/************************************************
函数功能：通过加减速度对运动进行同步处理---基于加减速度
参    数：tSyncPara--同步后的结果存储
        bLimitS----基于时间约束启动段
        bLimitE----基于时间约束停止段
        bGetStopPara---获取停止参数【加速度、捷度】（调速同样使用）
返 回 值：错误码
*************************************************/
int HS_Int_Move::SyncByAcc(SyncPara &tSync,bool bLimitS,bool bLimitE,bool bGetStopPara)
{	
    int iAxisNum = SpaceAxisNum;
    if(tSync.tPreHandle.eMoveType == MP_Joint)
        iAxisNum = MaxAxisNum;

	//>>根据各轴位移重新规划各轴最大速度
	double dVelNew[MaxAxisNum] = {0};	
	double dAcc[MaxAxisNum] = {0};
	double dDec[MaxAxisNum] = {0};
	
	for(int i = 0;i < iAxisNum;i++)
	{		
		dAcc[i] = tSync.tPreHandle.dSetAcc[i];
		dDec[i] = tSync.tPreHandle.dSetDec[i];

		double dSAcc = 0;
		if(dAcc[i] > Eps)
			dSAcc = 3*tSync.tPreHandle.dSetVel[i]*tSync.tPreHandle.dSetVel[i]/4/dAcc[i];	//加速段位移
		double dSDec = 0;
		if(dDec[i] > Eps)			
			dSDec = 3*tSync.tPreHandle.dSetVel[i]*tSync.tPreHandle.dSetVel[i]/4/dDec[i];	//减速段位移		

		if(dSAcc + dSDec > fabs(tSync.tPreHandle.dSetDis[i]))
		{			
			if((dAcc[i] + dDec[i]) > Eps)
			{
				//重新求解最大速度 VMax = sqrt(4*d*acc*dec/3/(acc+dec))
				dVelNew[i] = 2*sqrt(fabs(tSync.tPreHandle.dSetDis[i])*dAcc[i]*dDec[i]/3/(dAcc[i]+dDec[i]));
			}
			else
				dVelNew[i] = 0;	
		}
		else
		{
			dVelNew[i] = tSync.tPreHandle.dSetVel[i];
		}
		if(tSync.tPreHandle.dSetDis[i] < Eps)
			dVelNew[i] = -dVelNew[i];
	}


	//>>对速度进行同步
	double dKDV[MaxAxisNum] = {0};
	double dMaxKDV = 0;	
	for(int i = 0;i < iAxisNum;i++)
	{			
		//1.计算与速度的比值
		if(fabs(dVelNew[i]) < Eps)
			dKDV[i] = 0;
		else 
			dKDV[i] = tSync.tPreHandle.dSetDis[i]/dVelNew[i];
		//2.求解最大值		
		dMaxKDV = Max(dKDV[i],dMaxKDV);			
	}	

	if(dMaxKDV > Eps)
	{
		for(int i = 0;i < iAxisNum;i++)
			dVelNew[i] = dKDV[i]*dVelNew[i]/dMaxKDV;
	}

	//同步段时间
	double dTSync = dMaxKDV;
	double dTAcc = 0;
	double dTDec = 0;

	for(int i = 0;i < iAxisNum;i++)
	{
		double dTemp = 0;	
		if(fabs(dAcc[i]) > Eps)
			dTemp = fabs(3*dVelNew[i]/dAcc[i]/2);
		dTAcc = Max(dTemp,dTAcc);
		if(fabs(dDec[i]) > Eps)
			dTemp = fabs(3*dVelNew[i]/dDec[i]/2);
		dTDec = Max(dTemp,dTDec);
	}

	tSync.tVelPlan.dTime[TACC] = dTAcc;
	tSync.tVelPlan.dTime[TDEC] = dTDec;
	tSync.tVelPlan.dTime[TCON] = dMaxKDV - dTAcc/2 - dTDec/2;
	tSync.tVelPlan.dTime[TALL] = dTAcc + dTDec + tSync.tVelPlan.dTime[TCON];

	memcpy(tSync.tVelPlan.dEVel,dVelNew,sizeof(double)*MaxAxisNum);

    if(tSync.tPreHandle.eMoveType == MP_Joint)
    {
        tSync.tPreHandle.dRealKJVel = 0;
        for(int i = 0;i < iAxisNum;i++)
        {
            if(m_dJVelPara[i] > Eps)
                tSync.tPreHandle.dSetKVel[i] = fabs(tSync.tVelPlan.dEVel[i]/m_dJVelPara[i]);
            tSync.tPreHandle.dRealKJVel = Max(tSync.tPreHandle.dSetKVel[i],tSync.tPreHandle.dRealKJVel);
        }
    }
    else
    {
        //根据实际速度计算新的比例值
        if(tSync.tPreHandle.dPreHandleVel[0] > Eps)
            tSync.tPreHandle.dRealKJVel = fabs(dVelNew[0]/tSync.tPreHandle.dPreHandleVel[0])*tSync.tPreHandle.dOrigKJVel;
    }


    if(bGetStopPara)
    {
	    for(int i = 0;i < iAxisNum;i++)
	    {
		    if(dTAcc > Eps&&dTDec > Eps)
		    {
			    tSync.tVelPlan.dAcc[i] = fabs(tSync.tVelPlan.dEVel[i]*3/dTAcc/2);
			    tSync.tVelPlan.dDec[i] = fabs(tSync.tVelPlan.dEVel[i]*3/dTDec/2);
                tSync.tVelPlan.dJerk[i] = tSync.tVelPlan.dAcc[i]/dTAcc;
                double dJerk = tSync.tVelPlan.dDec[i]/dTDec;
                if(tSync.tVelPlan.dJerk[i] < dJerk)
                    tSync.tVelPlan.dJerk[i] = dJerk;
		    }      

            //获取减速停止使用的减速度值
            if(tSync.tVelPlan.dAcc[i] < tSync.tVelPlan.dDec[i])
                tSync.tVelPlan.dStopAcc[i] = tSync.tVelPlan.dDec[i];
            else
                tSync.tVelPlan.dStopAcc[i] = tSync.tVelPlan.dAcc[i];

            tSync.tVelPlan.dStopAcc[i] *= 1.2;

            //采用捷度，降低倍率调节对减速停止的影响//1.2倍
            tSync.tVelPlan.dStopJerk[i] = tSync.tVelPlan.dJerk[i]*1.44;
	    }	
    }

    if(bLimitS||bLimitE)
    {
        //时间约束添加处理：不同的条件下的时间约束应当有一定的调节规律
        //基于实际规划速度的调整；进行迭代优化
        double dTFreProtect = m_dTFreProtect;
        double dKJVel = tSync.tPreHandle.dRealKJVel;        
        for(int i = 0;i < 5;i++)
        {
            dTFreProtect = m_HS_BasicPara->CalcTProtectByVel(m_dTFreProtect,dKJVel,m_iGroupNum);  
            double dTAccNew = dTAcc;
            double dTDecNew = dTDec;            

            //时间约束
            if(dTAcc < dTFreProtect&&bLimitS)
            {
                dTAccNew = dTFreProtect;           
            }
            if(dTDec < dTFreProtect&&bLimitE)
            {
                dTDecNew = dTFreProtect;           
            }           
            SyncByTime(dTAccNew,dTDecNew,tSync,bGetStopPara);

            if(tSync.tVelPlan.dTime[TCON] < Eps)
            {
                dKJVel = (dKJVel + tSync.tPreHandle.dRealKJVel)/2;
            }
            else
                dKJVel = tSync.tPreHandle.dRealKJVel;
        }
    }
    else
    {
        if(dTAcc < LIMITACCTIME || dTDec < LIMITACCTIME)
        {
            SyncByTime(dTAcc,dTDec,tSync,bGetStopPara);
        }
    }
	return 0;
}

/************************************************
函数功能：通过加减速度对运动进行同步处理---基于加减速时间
参    数：dTAcc------加速段时间
		 dTDec------减速段时间
		 tSyncPara--同步后的结果存储
         bRecalAcc---是否重新获取加减速度值
返 回 值：错误码
*************************************************/
int HS_Int_Move::SyncByTime(double dTAcc,double dTDec,SyncPara &tSync,bool bRecalAcc)
{
    int iAxisNum = SpaceAxisNum;
    if(tSync.tPreHandle.eMoveType == MP_Joint)
        iAxisNum = MaxAxisNum;

    if(dTAcc < LIMITACCTIME)
        dTAcc = LIMITACCTIME;
    if(dTDec < LIMITACCTIME)
        dTDec = LIMITACCTIME;

	//>>根据各轴位移重新规划各轴最大速度
	double dVelNew[MaxAxisNum] = {0};	
	double dAcc[MaxAxisNum] = {0};
	double dDec[MaxAxisNum] = {0};
	for(int i = 0;i < iAxisNum;i++)
	{		
		double dSAcc = tSync.tPreHandle.dSetVel[i]*dTAcc/2;	//加速段位移
		double dSDec = tSync.tPreHandle.dSetVel[i]*dTDec/2;	//减速段位移		

		if(dSAcc + dSDec > fabs(tSync.tPreHandle.dSetDis[i]))
		{			
			//重新求解最大速度 VMax = sqrt(4*d*acc*dec/3/(acc+dec))			
			dVelNew[i] = fabs(2*tSync.tPreHandle.dSetDis[i])/(dTAcc+dTDec);				
		}
		else
		{
			dVelNew[i] = tSync.tPreHandle.dSetVel[i];			
		}
	}

	//>>对速度进行同步
	double dKDV[MaxAxisNum] = {0};
	double dMaxKDV = 0;	
	for(int i = 0;i < iAxisNum;i++)
	{			
		//1.计算与速度的比值
		if(fabs(dVelNew[i]) < Eps)
			dKDV[i] = 0;
		else 
			dKDV[i] = fabs(tSync.tPreHandle.dSetDis[i]/dVelNew[i]);
		//2.求解最大值		
		dMaxKDV = Max(dKDV[i],dMaxKDV);			
	}	

	if(dMaxKDV > Eps)
	{
		for(int i = 0;i < iAxisNum;i++)
        {
			dVelNew[i] = dKDV[i]*dVelNew[i]/dMaxKDV;
            if(tSync.tPreHandle.dSetDis[i] < Eps)
                dVelNew[i] = -dVelNew[i];
        }
	}

	tSync.tVelPlan.dTime[TACC] = dTAcc;
	tSync.tVelPlan.dTime[TDEC] = dTDec;
	tSync.tVelPlan.dTime[TCON] = dMaxKDV - dTAcc/2 - dTDec/2;

	tSync.tVelPlan.dTime[TALL] = dTAcc + dTDec + tSync.tVelPlan.dTime[TCON];

	memcpy(tSync.tVelPlan.dEVel,dVelNew,sizeof(double)*MaxAxisNum);

    if(tSync.tPreHandle.eMoveType == MP_Joint)
    {
        tSync.tPreHandle.dRealKJVel = 0;
        for(int i = 0;i < iAxisNum;i++)
        {
            if(m_dJVelPara[i] > Eps)
                tSync.tPreHandle.dSetKVel[i] = fabs(tSync.tVelPlan.dEVel[i]/m_dJVelPara[i]);
            tSync.tPreHandle.dRealKJVel = Max(tSync.tPreHandle.dSetKVel[i],tSync.tPreHandle.dRealKJVel);
        }
    }
    else
    {
        //根据实际速度计算新的比例值
        if(tSync.tPreHandle.dPreHandleVel[0] > Eps)
            tSync.tPreHandle.dRealKJVel = fabs(dVelNew[0]/tSync.tPreHandle.dPreHandleVel[0])*tSync.tPreHandle.dOrigKJVel;
    }	

    if(bRecalAcc)
    {
        for(int i = 0;i < iAxisNum;i++)
        {
            if(dTAcc > Eps&&dTDec > Eps)
            {
                tSync.tVelPlan.dAcc[i] = fabs(tSync.tVelPlan.dEVel[i]*3/dTAcc/2);
                tSync.tVelPlan.dDec[i] = fabs(tSync.tVelPlan.dEVel[i]*3/dTDec/2);
                tSync.tVelPlan.dJerk[i] = tSync.tVelPlan.dAcc[i]/dTAcc;
                double dJerk = tSync.tVelPlan.dDec[i]/dTDec;
                if(tSync.tVelPlan.dJerk[i] < dJerk)
                    tSync.tVelPlan.dJerk[i] = dJerk;
            }

            if(tSync.tVelPlan.dAcc[i] < tSync.tVelPlan.dDec[i])
                tSync.tVelPlan.dStopAcc[i] = tSync.tVelPlan.dDec[i];
            else
                tSync.tVelPlan.dStopAcc[i] = tSync.tVelPlan.dAcc[i];

            tSync.tVelPlan.dStopAcc[i] *= 1.2;
            //采用捷度，降低倍率调节对减速停止的影响//1.2倍
            tSync.tVelPlan.dStopJerk[i] = tSync.tVelPlan.dJerk[i]*1.44;
        }
    }
	return 0;
}

/************************************************
函数功能：运动调速
参    数：设定的速度倍率
返 回 值：错误码
*************************************************/
int HS_Int_Move::Ratio(double dRatio)
{
	double dTime[TIMECNT] = {0};

	int iRet = GetRatioPara(dRatio,dTime); 
	if(iRet == 0)
	{
		SetRatioPara(dRatio,dTime,true);
	}
	else if(iRet == 1)
	{
		SetRatioPara(dRatio,dTime,false);
	} 
	return 0;
}

/************************************************
函数功能：获取调速得到的规划参数
参    数：无
返 回 值：0-----表示获取参数OK，进行后续速度调整
        1------表示调速失败，不进行后续速度调整
		2------表示参数过小，无需调整
*************************************************/
int HS_Int_Move::GetRatioPara(double dRatio,double dTime[TIMECNT])
{
	//摆焊暂不响应调速
	if(m_bWeaveFlag)
		return 2;

	//触发条件，倍率有一定改变才进行触发，避免如果调速失败，持续触发功能
	if((fabs(dRatio - m_dSaveRatio) > 0.01))
	{
		m_dSaveRatio = dRatio;
		//倍率改变不大，则不触发调速
		if(fabs(dRatio - m_dSetRatio) > 0.02)   
		{
			//计算调速对应的位移量
			int iAxis = m_tSync.tPreHandle.iMaxAxis;
			double dSAcc = 0;
			double dSVel = 0;
			m_HS_VelPlan_Para[iAxis]->GetVel(dSVel);
			m_HS_VelPlan_Para[iAxis]->GetAcc(dSAcc);
			double dEVel = dRatio*m_tSync.tVelPlan.dEVel[iAxis]/m_dSetRatio;	
			double dJerk = m_tSync.tVelPlan.dStopJerk[iAxis]/1.44;

			//如果此时加速，则应当按照高速的捷度限制进行规划【去除倍率调整相关，按照减速的捷度进行调速】
			if(dJerk < Eps)	
			{
				LOG_ALGO("Jerk Error = %.3lf",dJerk);
				dJerk = 20000;
				return 0;
			}
			if(fabs(dEVel) < Eps)
			{
				LOG_ALGO("Jerk EVel = %.3lf,Axis = %d",dEVel,iAxis);
				return 0;
			}

			double dAcc = sqrt(fabs(3*(dSVel - dEVel)*dJerk/2));
			//判断SAcc与Jerk对应的Acc值，如果大较多，则可能处于平滑段，需要一定程度上进行限制
			double dTSAccAdd = fabs(dSAcc/dJerk/2);
			if(fabs(dSAcc) > m_tSync.tVelPlan.dStopAcc[iAxis])
			{
				dTSAccAdd = fabs(m_tSync.tVelPlan.dStopAcc[iAxis]/dJerk/2);
			}

			double dTAcc = fabs(3*(dSVel - dEVel)/dAcc/2) + dTSAccAdd;

			//存在与前一段的平滑，基于平滑段时间的保护，当前未对前段运动进行调速，故其时间会保持大于平滑段剩余时间
			double dSmoothPre = 0;
			if(m_tSync.tVelPlan.dTSmooth[PRE] > Eps)
			{
				dSmoothPre = m_tSync.tVelPlan.dTSmooth[PRE] - m_dTCur + m_dCycle;
				LOG_ALGO("Last Time SmoothPre = %.3lf;Ratio TAcc = %.3lf",dSmoothPre,dTAcc);
				if(dTAcc < dSmoothPre)
				{
					dTAcc = dSmoothPre;
				}
			}

			double dDisAcc = dSAcc*dTAcc*dTAcc/12 + (dSVel + dEVel)*dTAcc/2;
			double dDec = sqrt(fabs(3*dEVel*dJerk/2));
			double dTDec = fabs(3*dEVel/dDec/2);

			double dDisDec = dTDec*dEVel/2;
			double dSAll = m_tSync.tPreHandle.dSetDis[iAxis] - m_dMoveRatioPos[iAxis];

			LOG_ALGO("Jerk = %.3lf,Acc = %.3lf,Dec = %.3lf",dJerk,dAcc,dDec);

			if(fabs(dDisAcc + dDisDec) < fabs(dSAll))
			{
				double dTCon = fabs((dSAll - (dDisAcc + dDisDec))/dEVel);
				LOG_ALGO("Ratio OK:%.3lf To %.3lf;TCur = %.3lf,TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf",
					m_dSetRatio,dRatio,m_dTCur,dTAcc,dTCon,dTDec);

				dTime[TACC] = dTAcc;
				dTime[TCON] = dTCon;
				dTime[TDEC] = dTDec;
				dTime[TALL] = dTAcc + dTCon + dTDec;
			}
			else
			{
				LOG_ALGO("Ratio Failed:%.3lf To %.3lf;TCur = %.3lf",m_dSetRatio,dRatio,m_dTCur);
				return 1;
			}
		}
		else
		{
			LOG_ALGO("Ratio Failed:%.3lf To %.3lf;TCur = %.3lf",
				m_dSetRatio,dRatio,m_dTCur);
			return 2;
		}
	}
	else 
		return 2;
	return 0;
}
/************************************************
函数功能：获取调速得到的规划参数
参    数：
		dRatio----------调速倍率
		dTime-----------调速修改时间
		bRatioOKFlag----是否成功完成当前段调速
返 回 值：错误码
*************************************************/
int HS_Int_Move::SetRatioPara(double dRatio,double dTime[TIMECNT],bool bRatioOKFlag)
{
	int iErrorId = 0;

	int iAxisNum = SpaceAxisNum;
	if(m_eMoveType == MP_Joint)
		iAxisNum = MaxAxisNum;

	if(bRatioOKFlag)
	{
		for(int i = 0;i < iAxisNum;i++)
		{
			m_tVelPlanPara[i].dEVel = dRatio*m_tSync.tVelPlan.dEVel[i]/m_dSetRatio;	
			//修改当前速度
			m_tSync.tVelPlan.dEVel[i] = m_tVelPlanPara[i].dEVel;
			m_tVelPlanPara[i].dTCon = dTime[TCON];
			m_tVelPlanPara[i].dTAcc = dTime[TACC];
			m_tVelPlanPara[i].dTDec = dTime[TDEC];
			m_tVelPlanPara[i].dTAll = dTime[TALL];
			m_tVelPlanPara[i].dDis = m_tSync.tPreHandle.dSetDis[i];//修改为内部处理 - m_dMovePos[i];
			m_tVelPlanPara[i].dAddKVel = 1.0;
			m_HS_VelPlan_Para[i]->Ratio(m_tVelPlanPara[i]);
		}


		m_tSync.tVelPlan.dTime[TACC] = dTime[TACC];
		m_tSync.tVelPlan.dTime[TCON] = dTime[TCON];
		m_tSync.tVelPlan.dTime[TDEC] = dTime[TDEC];
		m_tSync.tVelPlan.dTime[TALL] = dTime[TALL];
		m_dTCur = m_dCycle;
		m_dTAll = dTime[TACC] + dTime[TCON] + dTime[TDEC];
		m_dSetRatio = dRatio;
	}

	//与下一段平滑的平滑调速处理
	if(m_bSmoothNextFlag)
	{
		RatioHandleSmoothNex();
	}
	return iErrorId;
}
/************************************************
函数功能：调速对存在下一段的平滑进行处理，修调下一段的速度以及平滑段信息
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_Move::RatioHandleSmoothNex()
{
    int iErrorId = 0;

    //当前平滑仅前瞻一段，按此进行与下一段的处理
    int iLookAhead = 1;
    int iBuffNext = (m_iIndex + iLookAhead)%MaxBuffSize;
    GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext];   
		
    double dTSmooth = m_tSync.tVelPlan.dTSmooth[NEX];
    //平滑段时间处理
    if(dTSmooth > m_tSync.tVelPlan.dTime[TDEC])
        dTSmooth = m_tSync.tVelPlan.dTime[TDEC];

    if(CheckTrajDataAvailable(m_tGTrajData,iBuffNext) == 0)
    {	
        //LOG_ALGO("Ratio Smooth Handle With NexMove Type = %d!",tTrajDataNex.elemt.eTrajType);

        memset(&m_tSyncAhead[m_iGroupNum][iLookAhead],0,sizeof(SyncPara));
        GetInputParaGE(m_tGTrajData,iBuffNext,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle,m_dSetRatio);

        SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead]);	

        if(m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TACC] < dTSmooth)
        {
            SyncByTime(dTSmooth,m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TDEC],m_tSyncAhead[m_iGroupNum][iLookAhead]);
        }
        m_tSync.tVelPlan.dTSmooth[NEX] = dTSmooth;
        m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTSmooth[PRE] = dTSmooth;
    }
    return 0;
}

/************************************************
函数功能：执行减速停止规划
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_Move::StopPlan()
{
	double dTStop = CalcStopTime();

	StopPlanByTime(dTStop);


	return 0;
}
/************************************************
函数功能：计算当前规划减速停止的时间
参    数：
返 回 值：错误码
*************************************************/
double HS_Int_Move::CalcStopTime()
{
	int iAxisNum = SpaceAxisNum;
	if(m_eMoveType == MP_Joint)
		iAxisNum = MaxAxisNum;
	//参数初始化			
	m_dTStop = 0;	
	m_iLookAhead[m_iGroupNum] = 0;

	memcpy(m_dStopSPos,m_dMovePos,sizeof(double)*MaxAxisNum); 
	double dStopJerk[MaxAxisNum] = {0};
	double dStopAcc[MaxAxisNum] = {0};
	memcpy(dStopAcc,m_tSync.tVelPlan.dStopAcc,sizeof(double)*MaxAxisNum);
	memcpy(dStopJerk,m_tSync.tVelPlan.dStopJerk,sizeof(double)*MaxAxisNum);

	//同步时间，根据加速度优化出最佳的时间【修改为基于捷度】			
	double dStopTime = 0;
	m_dTStop = 0.01;
	for(int i = 0;i < iAxisNum;i++)
	{
		double dDec = sqrt(fabs(3*m_dMoveVel[i]*dStopJerk[i]/2));
		if(dDec > Eps)
			dStopTime = fabs(3*m_dMoveVel[i]/dDec/2);
		m_dTStop = Max(m_dTStop,dStopTime);
	}

	double dTCalc = m_dTStop;	            //求解时间值
	double dTMax = m_dTStop*2;
	double dTMin = 0.01;
	double dAMax[MaxAxisNum] = {0};
	bool bOverAcc = false;		            //是否超出加速度限制			
	for(int iCnt = 0;iCnt < 8;iCnt++)
	{
		bOverAcc = false;
		//优化时间，选取合适的停止时间
		for(int i = 0;i < iAxisNum;i++)
		{
			//求解各轴的最大加速度
			double dKa = (m_dMoveAcc[i]*dTCalc + 2*m_dMoveVel[i])/4/dTCalc/dTCalc/dTCalc;
			double dKb = (-3*m_dMoveVel[i] - 2*m_dMoveAcc[i]*dTCalc)/3/dTCalc/dTCalc;

			if(fabs(dKa) > Eps)					
			{
				//判断最大值的时间位置																		
				dAMax[i] = fabs(-3*dKb*dKb/dKa/4 + m_dMoveAcc[i]);	
				double dTTop = dKb/4/dKa;   //最大加速度的时间点，如果为正，则抛物线的中心线小于0
				if(dTTop < Eps)
				{
					if((dAMax[i] > dStopAcc[i]+Eps)&&fabs(m_dMoveAcc[i]) < dStopAcc[i]+Eps)
					{
						bOverAcc = true;
						break;
					}
				}
				//捷度判断							
				double dTMax = fabs(dTCalc + dKb/4/dKa);
				if(fabs(dTMax) > Eps)
				{
					double dJerk = fabs(dAMax[i]/dTMax/2);
					if(dJerk > dStopJerk[i])
					{
						bOverAcc = true;
						break;
					}
				}							
			}					
		}				

		if(bOverAcc)				
			dTMin = dTCalc;
		else								
			dTMax = dTCalc;										

		dTCalc = (dTMin + dTMax)/2;
	}		

	m_dTStop = dTCalc;

	//最大减速时间限制
	if(m_dTStop > STOPTIMEMAX)
	{
		LOG_ALGO("StopTime > StopTimeMax = %.3lf,%.3lf",m_dTStop,STOPTIMEMAX);
		m_dTStop = STOPTIMEMAX;
	}
	if(m_eMoveType == MP_Joint)
	{
		LOG_ALGO("StopDec = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
			dStopAcc[0],dStopAcc[1],dStopAcc[2],dStopAcc[3],
			dStopAcc[4],dStopAcc[5],dStopAcc[6],dStopAcc[7],dStopAcc[8]);
		LOG_ALGO("StopJerk = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
			dStopJerk[0],dStopJerk[1],dStopJerk[2],dStopJerk[3],
			dStopJerk[4],dStopJerk[5],dStopJerk[6],dStopJerk[7],dStopJerk[8]);
	}
	else
	{
		LOG_ALGO("StopDec = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
			dStopAcc[0],dStopAcc[1],dStopAcc[2],dStopAcc[3],dStopAcc[4]);
		LOG_ALGO("StopJerk = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
			dStopJerk[0],dStopJerk[1],dStopJerk[2],dStopJerk[3],dStopJerk[4]);
	}

	return m_dTStop;
}
/************************************************
函数功能：按照给定的时间规划减速停止
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Move::StopPlanByTime(double dTStop)
{
	int iAxisNum = SpaceAxisNum;
	if(m_eMoveType == MP_Joint)
		iAxisNum = MaxAxisNum;

	m_dTStop = dTStop;
	
	//执行规划输出、减速停止
	for(int i = 0;i < iAxisNum;i++)
	{
		m_dStopKa[i] = (m_dMoveAcc[i]*m_dTStop + 2*m_dMoveVel[i])/(4*m_dTStop*m_dTStop*m_dTStop);
		m_dStopKb[i] = (-3*m_dMoveVel[i] - 2*m_dMoveAcc[i]*m_dTStop)/(3*m_dTStop*m_dTStop);
		m_dStopKc[i] = m_dMoveAcc[i]/2;		
		m_dStopDis[i] = m_dMoveAcc[i]*m_dTStop*m_dTStop/12 + m_dMoveVel[i]*m_dTStop/2;
        m_dStopPos[i] = m_dStopSPos[i] + m_dStopDis[i];
	}

	/************************************************
	对当前段减速停止进行规划
	*************************************************/	
    if(m_eMoveType == MP_Joint)
    {
        LOG_ALGO("SVel = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
            m_dMoveVel[0],m_dMoveVel[1],m_dMoveVel[2],m_dMoveVel[3],m_dMoveVel[4],m_dMoveVel[5],m_dMoveVel[6],m_dMoveVel[7],m_dMoveVel[8]);
        LOG_ALGO("SAcc = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
            m_dMoveAcc[0],m_dMoveAcc[1],m_dMoveAcc[2],m_dMoveAcc[3],m_dMoveAcc[4],m_dMoveAcc[5],m_dMoveAcc[6],m_dMoveAcc[7],m_dMoveAcc[8]);
        LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
            m_dMovePos[0],m_dMovePos[1],m_dMovePos[2],m_dMovePos[3],m_dMovePos[4],m_dMovePos[5],m_dMovePos[6],m_dMovePos[7],m_dMovePos[8]);
        LOG_ALGO("StopDis = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
            m_dStopDis[0],m_dStopDis[1],m_dStopDis[2],m_dStopDis[3],m_dStopDis[4],m_dStopDis[5],m_dStopDis[6],m_dStopDis[7],m_dStopDis[8]);
        LOG_ALGO("TCur = %.3lf,TStop = %.3lf",m_dTCur,m_dTStop);
    }
    else
    {
        LOG_ALGO("SVel = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
            m_dMoveVel[0],m_dMoveVel[1],m_dMoveVel[2],m_dMoveVel[3],m_dMoveVel[4]);
        LOG_ALGO("SAcc = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
            m_dMoveAcc[0],m_dMoveAcc[1],m_dMoveAcc[2],m_dMoveAcc[3],m_dMoveAcc[4]);
        LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
            m_dMovePos[0],m_dMovePos[1],m_dMovePos[2],m_dMovePos[3],m_dMovePos[4]);
        LOG_ALGO("StopDis = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
            m_dStopDis[0],m_dStopDis[1],m_dStopDis[2],m_dStopDis[3],m_dStopDis[4]);
        LOG_ALGO("TCur = %.3lf,TStop = %.3lf",m_dTCur,m_dTStop);
    }

	m_bStopFlag = true;
	m_dTCur = m_dCycle;

	//摆焊停止处理
	if(m_eMoveType == MP_Line||m_eMoveType == MP_Arc)
	{
		if(m_bWeaveFlag&&!m_bTimeInNextSmooth)
			WeaveStop();
	}
	return 0;
}

/************************************************
函数功能：运动混合平滑处理
参    数：tSyncCur---当前段规划信息
          tSyncNex---下一段规划信息
返 回 值：错误码
*************************************************/
int HS_Int_Move::SmoothHandle(SyncPara &tSyncCur,SyncPara &tSyncNex)
{	    
    int iErrorId = 0;
    bool bLimitSetOKFlag = false;
    double dTMin = 0;
    double dTMax = 0;
    bool bRestartFlag = false;
    /*****************************************************************************/
    //迭代处理相关参数
    const int MAXCNT = 5;		
    const double MAXLIMIT = 1.0;
    double dTSmooth = 0;	
    SyncPara tSyncSave[2];	                    //缓存第一次计算的数据
    memset(tSyncSave,0,sizeof(tSyncSave));
    INTERTYPE eInterType = IT_NONE; 
    double dKLimit = 0.0;
    double dKPre = 0.0;                         //上一次的比例值
    /*****************************************************************************/
    for(int iInterCnt = 0;iInterCnt < MAXCNT;iInterCnt++)
    {	
        if(iInterCnt == 0)
        {
            tSyncCur.tPreHandle.dRadius[NEX] = tSyncCur.tPreHandle.dRadius[ORIGNEX];		
            tSyncNex.tPreHandle.dRadius[PRE] = tSyncCur.tPreHandle.dRadius[ORIGNEX];

            //Mode0：时间优先的平滑规划，轨迹约束靠后，最大时间为TACC+2*TCON，保留一半的匀速段时间避免前后段的影响
            double dTConNex = tSyncCur.tVelPlan.dTime[TCON] + (tSyncCur.tVelPlan.dTime[TACC] - tSyncCur.tVelPlan.dTime[TDEC]);
            if(dTConNex < Eps)
                dTConNex = 0;
            else if(dTConNex > tSyncCur.tVelPlan.dTime[TCON]*2)
                dTConNex = tSyncCur.tVelPlan.dTime[TCON]*2;

            double dTSCurMax = tSyncCur.tVelPlan.dTime[TDEC]+dTConNex;

            double dTConPre = tSyncNex.tVelPlan.dTime[TCON] - (tSyncNex.tVelPlan.dTime[TACC] - tSyncNex.tVelPlan.dTime[TDEC]);
            if(dTConPre < Eps)
                dTConPre = 0;
            else if(dTConPre > tSyncNex.tVelPlan.dTime[TCON]*2)
                dTConPre = tSyncNex.tVelPlan.dTime[TCON]*2;
            double dTSNexMax = tSyncNex.tVelPlan.dTime[TACC]+dTConPre;

            //平滑系数百分比---基于时间
            //dTSCurMax = tSyncCur.tPreHandle.dRadius[ORIGNEX]/100*dTSCurMax;

            //平滑系数百分比---基于位移量
            dTSCurMax = GetTSmoothByDis(tSyncCur,dTSCurMax);

            if(dTSCurMax < dTSNexMax)
                dTSmooth = dTSCurMax;
            else
                dTSmooth = dTSNexMax;

            tSyncSave[0] = tSyncCur;
            tSyncSave[1] = tSyncNex;
        }
        else
        {
            tSyncCur = tSyncSave[0];
            tSyncNex = tSyncSave[1];
        }

        //根据平滑段的时间优化当前运动，使得平滑段的时间 = 加减速段的时间
        if(dTSmooth > tSyncCur.tVelPlan.dTime[TDEC])
            SyncByTime(tSyncCur.tVelPlan.dTime[TACC],dTSmooth,tSyncCur);
        if(dTSmooth > tSyncNex.tVelPlan.dTime[TACC])
            SyncByTime(dTSmooth,tSyncNex.tVelPlan.dTime[TDEC],tSyncNex);

        tSyncCur.tVelPlan.dTSmooth[NEX] = dTSmooth;
        tSyncNex.tVelPlan.dTSmooth[PRE] = dTSmooth;

        /*********************对平滑段速度的处理与优化，约束合成速度，避免速度超出限制*********************************/
        //求解当前段的匀速点（结束位置）对应的速度合成值
        //该点位处于下一段运动中的时刻值		
        double dKVel = 0;		
        //平滑中间时刻	
        double dTInPre = tSyncCur.tVelPlan.dTime[TALL] - dTSmooth/2;
        double dTInNext = dTSmooth/2;	
        double dKAcc = 0;
        double dKJerk = 0;
        SmoothMaxJParaCalc(tSyncCur,tSyncNex,dTInPre,dTInNext,dKVel,dKAcc);	

        //if(m_bAutoTorchFlag)
        //{
        //    dKAcc = CalcCorrectKTorch(dTSmooth,dKAcc);
        //}

        //基于时间，以及加速度值【电流比例值】，对平滑段的时间进行约束处理
        dKJerk = CalcKJerkSmooth(dKAcc,m_dTFreProtect,dTSmooth);
        /*****************************迭代参数的修改更改处理**********************************************************/		
        dKAcc = Max(dKAcc,dKJerk);        
        dKLimit = Max(dKVel,dKAcc);
        if(iInterCnt == 0)
        {			
            if(dKLimit < MAXLIMIT)
            {		
                //可以进行提速处理，暂不处理
                break;
            }
            else
            {				
                //速度或者加速度超出，则增加平滑段的时间以约束处理【平滑合成段】
                eInterType = IT_MOVEDOWN_S;
                dTMin = dTSmooth;
                dTSmooth *= dKLimit;			
                dTMax = dTSmooth;	
                dKPre = dKLimit;
            }
        }
        else 
        {
            switch(eInterType)
            {
            case IT_MOVEDOWN_S:
                if(dKLimit > 1.0)
                {
                    dTMin = dTSmooth;
                    dTMax = dTSmooth*dKLimit;
                    dTSmooth = dTMax;
                }
                else
                {
                    //取得合适解，二分法求解最优
                    dTSmooth = (dTMin + dTMax)/2;
                    eInterType = IT_MOVEDOWN;
                }
                break;
            case IT_MOVEDOWN:
                if(dKLimit > 1.0)
                {
                    dTMin = dTSmooth;
                    dTMax = dTSmooth*dKLimit;
                }
                else
                {
                    //取得合适解，二分法求解最优
                    dTMax = dTSmooth;
                    dTMin = dTSmooth*dKLimit;
                }
                dTSmooth = (dTMin + dTMax)/2;  
                break;
            default:
                break;
            }	
            dKPre = dKLimit;
        }
    }

	if(tSyncCur.tPreHandle.bSmoothCRMode)
	{
		//最大时间求解---简化版处理，限定平滑时间---最大是减速停止时间
		double dTSCurMax = tSyncCur.tVelPlan.dTime[TDEC];
		double dTSNexMax = tSyncNex.tVelPlan.dTime[TACC];
		double dTSmoothMax = dTSCurMax;
		if(dTSNexMax < dTSmoothMax)
			dTSmoothMax = dTSNexMax;
		dTSmooth = GetTSmoothByDis(tSyncCur,dTSmoothMax); 

		//所需位移对应时间不是整数倍时间，要-1个插补周期，不然会在所需位移之前输出Done信号
		double tall = tSyncCur.tVelPlan.dTime[TDEC]+tSyncCur.tVelPlan.dTime[TACC]+tSyncCur.tVelPlan.dTime[TCON];
		tall -= dTSmooth;
		int n = tall/m_dCycle;
		double time = n*m_dCycle;
		double ertime = tall-time;
		if(ertime>0.00001)
			dTSmooth -= m_dCycle;
		/////////////////////////////////////////////

		tSyncCur.tVelPlan.dTSmooth[NEX] = dTSmooth;
		tSyncNex.tVelPlan.dTSmooth[PRE] = dTSmooth;
	}

	SmoothDisCalc(tSyncCur,false);
	SmoothDisCalc(tSyncNex,true);

	return 0;
}
/************************************************
函数功能：计算对应的平滑段的距离长度
参    数：
		 tSync---规划信息
		 bPre----是否前段平滑长度		 
返 回 值：错误码
*************************************************/
int HS_Int_Move::SmoothDisCalc(SyncPara &tSyncCur,bool bPre)
{
	if(bPre)
	{
		double dT = tSyncCur.tVelPlan.dTSmooth[PRE];
		//求解该时间对应的位移量
		if(dT < tSyncCur.tVelPlan.dTime[TACC]+Eps)
		{
			double dTAcc = tSyncCur.tVelPlan.dTime[TACC];
			double dKa = - tSyncCur.tVelPlan.dEVel[0]/(2*dTAcc*dTAcc*dTAcc);
			double dKb = tSyncCur.tVelPlan.dEVel[0]/(dTAcc*dTAcc);	

			tSyncCur.tPreHandle.dDisSmooth[PRE] = dT*dT*dT*(dKa*dT + dKb);
		}
		else if(dT < tSyncCur.tVelPlan.dTime[TACC] + tSyncCur.tVelPlan.dTime[TCON])
		{
			tSyncCur.tPreHandle.dDisSmooth[PRE] = tSyncCur.tVelPlan.dEVel[0]*( dT - tSyncCur.tVelPlan.dTime[TACC]/2);
		}
		else
		{
			double dTLast = tSyncCur.tVelPlan.dTime[TALL] - dT;
			double dTDec = tSyncCur.tVelPlan.dTime[TDEC];
			double dKa = - tSyncCur.tVelPlan.dEVel[0]/(2*dTDec*dTDec*dTDec);
			double dKb = tSyncCur.tVelPlan.dEVel[0]/(dTDec*dTDec);	

			double dDisTemp = dTLast*dTLast*dTLast*(dKa*dTLast + dKb);

			tSyncCur.tPreHandle.dDisSmooth[PRE] = tSyncCur.tPreHandle.dSetDis[0] - dDisTemp;
		}
	}
	else
	{
		double dT = tSyncCur.tVelPlan.dTSmooth[NEX];
		//求解该时间对应的位移量
		if(dT < tSyncCur.tVelPlan.dTime[TDEC] +Eps)
		{
			double dTAcc = tSyncCur.tVelPlan.dTime[TDEC];
			double dKa = - tSyncCur.tVelPlan.dEVel[0]/(2*dTAcc*dTAcc*dTAcc);
			double dKb = tSyncCur.tVelPlan.dEVel[0]/(dTAcc*dTAcc);	

			tSyncCur.tPreHandle.dDisSmooth[NEX] = dT*dT*dT*(dKa*dT + dKb);
		}
		else if(dT < tSyncCur.tVelPlan.dTime[TDEC] + tSyncCur.tVelPlan.dTime[TCON])
		{
			tSyncCur.tPreHandle.dDisSmooth[NEX] = tSyncCur.tVelPlan.dEVel[0]*( dT - tSyncCur.tVelPlan.dTime[TDEC]/2);
		}
		else
		{
			double dTLast = tSyncCur.tVelPlan.dTime[TALL] - dT;
			double dTDec = tSyncCur.tVelPlan.dTime[TACC];
			double dKa = - tSyncCur.tVelPlan.dEVel[0]/(2*dTDec*dTDec*dTDec);
			double dKb = tSyncCur.tVelPlan.dEVel[0]/(dTDec*dTDec);	

			double dDisTemp = dTLast*dTLast*dTLast*(dKa*dTLast + dKb);

			tSyncCur.tPreHandle.dDisSmooth[NEX] = tSyncCur.tPreHandle.dSetDis[0] - dDisTemp;
		}
	}
	return 0;
}
/************************************************
函数功能：运动混合平滑处理【动态规划处理，与通用平滑处理有所区别】
参    数：tSyncCur---当前段规划信息
          tSyncNex---下一段规划信息
返 回 值：错误码
*************************************************/
int HS_Int_Move::DynSmoothHandle(SyncPara &tSyncCur,SyncPara &tSyncNex)
{	    
    //当前剩余允许最大可调节平滑时间值
    //匀速段的速度不能发生改变
    double dTMinCur = m_dTCur;
    if(dTMinCur < tSyncCur.tVelPlan.dTime[TACC])
        dTMinCur = tSyncCur.tVelPlan.dTime[TACC];
    dTMinCur += m_dCycle;

    double dLastTSmooth = tSyncCur.tVelPlan.dTime[TALL] - dTMinCur;

    int iErrorId = 0;
    bool bLimitSetOKFlag = false;
    double dTMin = 0;
    double dTMax = 0;
    bool bRestartFlag = false;
    /*****************************************************************************/
    //迭代处理相关参数
    const int MAXCNT = 5;		
    const double MAXLIMIT = 1.0;
    double dTSmooth = 0;	
    SyncPara tSyncSave[2];	                    //缓存第一次计算的数据
    memset(tSyncSave,0,sizeof(tSyncSave));
    INTERTYPE eInterType = IT_NONE; 
    double dKLimit = 0.0;
    double dKPre = 0.0;                         //上一次的比例值
    /*****************************************************************************/
    for(int iInterCnt = 0;iInterCnt < MAXCNT;iInterCnt++)
    {	
        if(iInterCnt == 0)
        {
            tSyncCur.tPreHandle.dRadius[NEX] = tSyncCur.tPreHandle.dRadius[ORIGNEX];		
            tSyncNex.tPreHandle.dRadius[PRE] = tSyncCur.tPreHandle.dRadius[ORIGNEX];

            //Mode0：时间优先的平滑规划，轨迹约束靠后，最大时间为TACC+2*TCON，保留一半的匀速段时间避免前后段的影响
            double dTConNex = tSyncCur.tVelPlan.dTime[TCON] + (tSyncCur.tVelPlan.dTime[TACC] - tSyncCur.tVelPlan.dTime[TDEC]);
            if(dTConNex < Eps)
                dTConNex = 0;
            else if(dTConNex > tSyncCur.tVelPlan.dTime[TCON]*2)
                dTConNex = tSyncCur.tVelPlan.dTime[TCON]*2;

            double dTSCurMax = tSyncCur.tVelPlan.dTime[TDEC]+dTConNex;

            double dTConPre = tSyncNex.tVelPlan.dTime[TCON] - (tSyncNex.tVelPlan.dTime[TACC] - tSyncNex.tVelPlan.dTime[TDEC]);
            if(dTConPre < Eps)
                dTConPre = 0;
            else if(dTConPre > tSyncNex.tVelPlan.dTime[TCON]*2)
                dTConPre = tSyncNex.tVelPlan.dTime[TCON]*2;
            double dTSNexMax = tSyncNex.tVelPlan.dTime[TACC]+dTConPre;

            //平滑系数百分比---基于时间
            //dTSCurMax = tSyncCur.tPreHandle.dRadius[ORIGNEX]/100*dTSCurMax;

            //平滑系数百分比---基于位移量
            dTSCurMax = GetTSmoothByDis(tSyncCur,dTSCurMax);

            if(dTSCurMax < dTSNexMax)
                dTSmooth = dTSCurMax;
            else
                dTSmooth = dTSNexMax;

            if(dTSmooth > dLastTSmooth)
                dTSmooth = dLastTSmooth;

            tSyncSave[0] = tSyncCur;
            tSyncSave[1] = tSyncNex;
        }
        else
        {
            tSyncCur = tSyncSave[0];
            tSyncNex = tSyncSave[1];
        }

        //根据平滑段的时间优化当前运动，使得平滑段的时间 = 加减速段的时间
        if(dTSmooth > tSyncCur.tVelPlan.dTime[TDEC])
            SyncByTime(tSyncCur.tVelPlan.dTime[TACC],dTSmooth,tSyncCur);
        if(dTSmooth > tSyncNex.tVelPlan.dTime[TACC])
            SyncByTime(dTSmooth,tSyncNex.tVelPlan.dTime[TDEC],tSyncNex);

        tSyncCur.tVelPlan.dTSmooth[NEX] = dTSmooth;
        tSyncNex.tVelPlan.dTSmooth[PRE] = dTSmooth;

        /*********************对平滑段速度的处理与优化，约束合成速度，避免速度超出限制*********************************/
        //求解当前段的匀速点（结束位置）对应的速度合成值
        //该点位处于下一段运动中的时刻值		
        double dKVel = 0;		
        //平滑中间时刻	
        double dTInPre = tSyncCur.tVelPlan.dTime[TALL] - dTSmooth/2;
        double dTInNext = dTSmooth/2;	
        double dKAcc = 0;
        double dKJerk = 0;
        SmoothMaxJParaCalc(tSyncCur,tSyncNex,dTInPre,dTInNext,dKVel,dKAcc);	

        //基于时间，以及加速度值【电流比例值】，对平滑段的时间进行约束处理
        dKJerk = CalcKJerkSmooth(dKAcc,m_dTFreProtect,dTSmooth);
        /*****************************迭代参数的修改更改处理**********************************************************/		
        dKAcc = Max(dKAcc,dKJerk);        
        dKLimit = Max(dKVel,dKAcc);
        if(iInterCnt == 0)
        {			
            if(dKLimit < MAXLIMIT)
            {		
                //可以进行提速处理，暂不处理
                break;
            }
            else
            {				
                //速度或者加速度超出，则增加平滑段的时间以约束处理【平滑合成段】
                eInterType = IT_MOVEDOWN_S;
                dTMin = dTSmooth;
                dTSmooth *= dKLimit;			
                dTMax = dTSmooth;	
                dKPre = dKLimit;
            }
        }
        else 
        {
            switch(eInterType)
            {
            case IT_MOVEDOWN_S:
                if(dKLimit > 1.0)
                {
                    dTMin = dTSmooth;
                    dTMax = dTSmooth*dKLimit;
                    dTSmooth = dTMax;
                }
                else
                {
                    //取得合适解，二分法求解最优
                    dTSmooth = (dTMin + dTMax)/2;
                    eInterType = IT_MOVEDOWN;
                }
                break;
            case IT_MOVEDOWN:
                if(dKLimit > 1.0)
                {
                    dTMin = dTSmooth;
                    dTMax = dTSmooth*dKLimit;
                }
                else
                {
                    //取得合适解，二分法求解最优
                    dTMax = dTSmooth;
                    dTMin = dTSmooth*dKLimit;
                }
                dTSmooth = (dTMin + dTMax)/2;  
                break;
            default:
                break;
            }	
            dKPre = dKLimit;
        }
        if(dTSmooth > dLastTSmooth)
            dTSmooth = dLastTSmooth;
    }
	return 0;
}

/************************************************
函数功能：动态平滑处理，平滑规划时无下一段点位，运动过程中获取到下一段点位
参    数：
返 回 值：平滑时间
*************************************************/
int HS_Int_Move::DynSmoothPlan()
{
    int iErrorId = 0;
    int iBuffNext = (m_iIndex + 1)%MaxBuffSize;
    GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext];   

    int iRet = CheckTrajDataAvailable(m_tGTrajData,iBuffNext);

    if(iRet == 0)
    {
        //混合平滑处理
        GetInputParaGE(m_tGTrajData,iBuffNext,m_iGroupNum,m_tSyncAhead[m_iGroupNum][1].tPreHandle,m_dSetRatio);

        SyncByAcc(m_tSyncAhead[m_iGroupNum][1],false,true);

        //局限版平滑处理，优先保证当前段的规划不变
        iErrorId = DynSmoothHandle(m_tSyncAhead[m_iGroupNum][0],m_tSyncAhead[m_iGroupNum][1]);

        if(iErrorId == 0)
        {
            LOG_ALGO("Dyn Smooth Plan OK!");
            m_tSync = m_tSyncAhead[m_iGroupNum][0];
            LOG_ALGO("TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
                m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);
            m_iLookAhead[m_iGroupNum] = 1;

			m_bSmoothMoveFlag[m_iGroupNum] = true;
			m_bSmoothNextFlag = true;

            //重新规划当前段参数
            for(int i = 0;i < MaxAxisNum;i++)
            {
                m_tVelPlanPara[i].dTCon = m_tSync.tVelPlan.dTime[TCON];
                m_tVelPlanPara[i].dTAcc = m_tSync.tVelPlan.dTime[TACC];
                m_tVelPlanPara[i].dTDec = m_tSync.tVelPlan.dTime[TDEC];
                m_tVelPlanPara[i].dTAll = m_tSync.tVelPlan.dTime[TALL];
                m_HS_VelPlan_Para[i]->RePlan(m_tVelPlanPara[i]);
            }

            m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
            m_dTCon = m_tSync.tVelPlan.dTime[TCON];
            m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
            m_dTAll = m_tSync.tVelPlan.dTime[TALL];
        }
        else
        {
            LOG_ALGO("Dyn Smooth Plan Failed!");
        }
        
        m_bDynSmoothFlag = false;
    }
    return 0;
}

/************************************************
函数功能：在一段运动运行结束的周期内对点位进行修正预测，保证后续运动的点位一致性【修正一段运动只执行一次】
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Move::MoveDoneJPosCorrect()
{
    if(m_bJPosCorrectFlag)
        return 0;
    else
        m_bJPosCorrectFlag = true;

    int iErrorId = 0;
    bool bJPosErrorFlag = false;
    int iErrorAxis = 0;
    int iErrorCorrect = 0;

    for(int i = 0;i < 6;i++)
    {
        double dJDis = m_dRJPos[i] - m_tSync.tPreHandle.dSetJPos[1][i];
        if(fabs(dJDis) > 180.0)
        {
            bJPosErrorFlag = true;
            LOG_ALGO("Error MoveDone JPos[%d],RJPos = %.3lf,SJPos = %.3lf",i,m_dRJPos[i],m_tSync.tPreHandle.dSetJPos[1][i]);
            iErrorAxis = i;
            if(dJDis > 450)
                iErrorCorrect = 2;
            else if(dJDis > 180)
                iErrorCorrect = 1;
            else if(dJDis < -180)
                iErrorCorrect = -1;
            else if(dJDis < -540)
                iErrorCorrect = -2;
            break;
        }
    }

    if(bJPosErrorFlag)
    {
        //获取后续的点位并进行修正
        for(int i = 0;i < MaxBuffSize;i++)
        {
            int iBuff = (m_iIndex + 1 + i)%MaxBuffSize;
            if(CheckTrajDataAvailable(m_tGTrajData,iBuff) == 0)
            {
                Para_PreHandle tPreHandle;
                memcpy(&tPreHandle,m_tGTrajData[iBuff].iData[m_iGroupNum],sizeof(Para_PreHandle)); 

                if(m_tGTrajData[iBuff].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_Joint)
                {
                    tPreHandle.dSPos[iErrorAxis] += iErrorCorrect*360;
                    tPreHandle.dSetJPos[0][iErrorAxis] += iErrorCorrect*360;
                    
                    if(m_tGTrajData[iBuff].tMotionData.tBaseMoveData[m_iGroupNum].sEndPos.hs_coordinate.iCoordinate == JOINT_COORD_SYSTEM)
                    {
                        tPreHandle.dDis[iErrorAxis] = tPreHandle.dEPos[iErrorAxis] - tPreHandle.dSPos[iErrorAxis];

                        JointReAdjustPara(tPreHandle);
                        memcpy(m_tGTrajData[iBuff].iData[m_iGroupNum],&tPreHandle,sizeof(Para_PreHandle)); 
                        LOG_ALGO("JPos Correct Id = %d",iBuff);
                        LOG_ALGO("SJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                            tPreHandle.dSetJPos[0][0],tPreHandle.dSetJPos[0][1],tPreHandle.dSetJPos[0][2],tPreHandle.dSetJPos[0][3],tPreHandle.dSetJPos[0][4],tPreHandle.dSetJPos[0][5]);
                        LOG_ALGO("EJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                            tPreHandle.dSetJPos[1][0],tPreHandle.dSetJPos[1][1],tPreHandle.dSetJPos[1][2],tPreHandle.dSetJPos[1][3],tPreHandle.dSetJPos[1][4],tPreHandle.dSetJPos[1][5]);
                        break;
                    }
                    else
                    {
                        tPreHandle.dEPos[iErrorAxis] += iErrorCorrect*360;
                        tPreHandle.dSetJPos[1][iErrorAxis] += iErrorCorrect*360;
                        memcpy(m_tGTrajData[iBuff].iData[m_iGroupNum],&tPreHandle,sizeof(Para_PreHandle)); 
                        LOG_ALGO("JPos Correct Id = %d",iBuff);
                        LOG_ALGO("SJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                            tPreHandle.dSetJPos[0][0],tPreHandle.dSetJPos[0][1],tPreHandle.dSetJPos[0][2],tPreHandle.dSetJPos[0][3],tPreHandle.dSetJPos[0][4],tPreHandle.dSetJPos[0][5]);
                        LOG_ALGO("EJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                            tPreHandle.dSetJPos[1][0],tPreHandle.dSetJPos[1][1],tPreHandle.dSetJPos[1][2],tPreHandle.dSetJPos[1][3],tPreHandle.dSetJPos[1][4],tPreHandle.dSetJPos[1][5]);
                    }
                }
                else
                {
                    tPreHandle.dSetJPos[0][iErrorAxis] += iErrorCorrect*360;
                    tPreHandle.dSetJPos[1][iErrorAxis] += iErrorCorrect*360;
                    memcpy(m_tGTrajData[iBuff].iData[m_iGroupNum],&tPreHandle,sizeof(Para_PreHandle)); 
                    LOG_ALGO("JPos Correct Id = %d",iBuff);
                    LOG_ALGO("SJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                        tPreHandle.dSetJPos[0][0],tPreHandle.dSetJPos[0][1],tPreHandle.dSetJPos[0][2],tPreHandle.dSetJPos[0][3],tPreHandle.dSetJPos[0][4],tPreHandle.dSetJPos[0][5]);
                    LOG_ALGO("EJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                        tPreHandle.dSetJPos[1][0],tPreHandle.dSetJPos[1][1],tPreHandle.dSetJPos[1][2],tPreHandle.dSetJPos[1][3],tPreHandle.dSetJPos[1][4],tPreHandle.dSetJPos[1][5]);
                }
            }
            else
                break;
        }        
    }
    return iErrorId;
}

/************************************************
函数功能：对关节处理信息进行重新规划获取
参    数：
返 回 值：平滑时间
*************************************************/
int HS_Int_Move::JointReAdjustPara(Para_PreHandle &tPH_Joint)
{
    int iErrorId = 0;
    //重新规划
    SyncPara tSyncPara; 
    memset(&tSyncPara,0,sizeof(tSyncPara));
    tSyncPara.tPreHandle = tPH_Joint;
    SyncByAcc(tSyncPara);

    tPH_Joint.dRealKJVel = 0;
    for(int i = 0;i < MaxAxisNum;i++)
    {
        if(m_dJVelPara[i] > Eps)
            tPH_Joint.dSetKVel[i] = fabs(tSyncPara.tVelPlan.dEVel[i]/m_dJVelPara[i]);
        tPH_Joint.dRealKJVel = Max(tPH_Joint.dSetKVel[i],tPH_Joint.dRealKJVel);
    }

    return iErrorId;
}

/************************************************
函数功能：基于位移的平滑时间计算，CNT对应的是最大位移量的百分比
参    数：tSync---规划信息
         dTSmoothMax---最大平滑时间
返 回 值：平滑时间
*************************************************/
double HS_Int_Move::GetTSmoothByDis(SyncPara &tSync,double dTSmoothMax)
{
    if(!tSync.tPreHandle.bSmoothCRMode)
    {
        //100%时，直接返回最大时间
        if(fabs(tSync.tPreHandle.dRadius[ORIGNEX] - 100) < Eps)
            return dTSmoothMax;
    }

    double dTSmooth = 0;
    int iAxis = tSync.tPreHandle.iMaxAxis;

    //计算部分位移点对应的比例情况
    //减速段长度
    double dLDec = fabs(tSync.tVelPlan.dEVel[iAxis]*tSync.tVelPlan.dTime[TDEC]/2);

    //总位移量计算
    double dLSmoothMax = dLDec;

    if(dTSmoothMax > tSync.tVelPlan.dTime[TDEC])
    {
        dLSmoothMax = dLDec + (dTSmoothMax - tSync.tVelPlan.dTime[TDEC])*tSync.tVelPlan.dEVel[iAxis];
    }

    //计算平滑系数对应的位移量,100对应的是一半，则200才对应全部的长度
    double dLSmooth = tSync.tPreHandle.dRadius[ORIGNEX]/200*fabs(tSync.tPreHandle.dSetDis[iAxis]);

    if(tSync.tPreHandle.bSmoothCRMode)
        dLSmooth = tSync.tPreHandle.dRadius[ORIGNEX];

    //根据不同的位移量计算对应的时间
    if(dLSmooth > dLSmoothMax)
    {
        dTSmooth = dTSmoothMax;
    }
    else if(dLSmooth > dLDec)
    {
        //修改，匀速段的时间会转化为减速段时间，故而需要进行处理
        dTSmooth = tSync.tVelPlan.dTime[TDEC] + 2*(dLSmooth - dLDec)/fabs(tSync.tVelPlan.dEVel[iAxis]);
        if(dTSmooth > dLSmoothMax)
            dTSmooth = dLSmoothMax;
    }
    else
    {
        if(!tSync.tPreHandle.bSmoothCRMode)
        {
            //处于加减速段，简化处理计算过程
            double dK = dLSmooth/dLDec;
            double dKS = pow(dK,1.0/3.0);
            dTSmooth = tSync.tVelPlan.dTime[TDEC] * dKS;
        }
        else
        {
            double dTDec = tSync.tVelPlan.dTime[TDEC];
            //用牛顿迭代法求解出拐出点的时间
            //位移公式：f(t) = at^4 + bt^3 + c*t^2 + d*t f'(t) = 4at^3 + 3bt^2 + 2*c*t + d			
            double dB = tSync.tVelPlan.dEVel[iAxis]/dTDec/dTDec;
            double dA = -dB/dTDec/2;			
            double dX = dTDec;		//迭代初值
            double dX1 = dTDec/2;	
            double dFt = 0;
            double dHt = 0;

            if(tSync.tVelPlan.dEVel[iAxis] < 0)
                dLSmooth = -dLSmooth;

            for(int i = 0;i < 15&&fabs(dX - dX1) >= 0.0001;i++)
            {
                dX1 = dX;
                dFt = (dA*dX + dB)*dX*dX*dX - dLSmooth;
                dHt = (4*dA*dX + 3*dB)*dX*dX;
                if(fabs(dHt) > Eps)
                    dX = dX - dFt/dHt;
                else				//改变初始点位
                    dX = dX/2;
            }

            if(fabs(dX - dX1) > 0.0001||dX < Eps)
            {
                //m_HS_Printer->outDebugInfo("Motion_P","Line","GetTSmoothByDis",0,AllDeb,"Calc Error!");
                dTSmooth = tSync.tVelPlan.dTime[TDEC];
            }
            else
                dTSmooth = dX;		
        }
    }

	//修改5：避免当前平滑段时间 > 下一段的总时间，导致插补异常
	if(dTSmooth > dTSmoothMax)
		dTSmooth = dTSmoothMax;

    return dTSmooth;
}

/************************************************
函数功能：计算两段平滑，平滑合成段对应的最大关节速度比例系数
参    数：
		 tSyncCur---当前段规划信息
		 tSyncNex---下一段规划信息
		 dTInPre----待求时刻值(当前段）
		 dTInNext---待求时刻值（下一段）
		 dKJVel-----得到关节最大比例系数
返 回 值：错误码
*************************************************/
int HS_Int_Move::SmoothMaxJVelCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJVel)
{	
	double dPlanVelCur[MaxAxisNum] = {0};
	double dPlanVelNex[MaxAxisNum] = {0};		
    
    GetPlanVel(tSyncCur,dTInPre,dPlanVelCur);	
    GetPlanVel(tSyncNex,dTInNext,dPlanVelNex);

	double dJVel[MaxAxisNum] = {0};
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dJVel[i] = dPlanVelCur[i] + dPlanVelNex[i];
		double dTemp = 0;
		if(m_dJVelPara[i] > Eps)
			dTemp = fabs(dJVel[i]/m_dJVelPara[i]);
		dKJVel = Max(dTemp,dKJVel);
	}
	return 0;
}

/************************************************
函数功能：计算平滑，平滑合成段对应的最大关节加速度比例系数
参    数：
		 tSyncCur---当前段规划信息
		 tSyncNex---下一段规划信息
		 dTInPre----待求时刻值(当前段）
		 dTInNext---待求时刻值（下一段）
		 dKJVel-----得到关节最大比例系数
返 回 值：错误码
*************************************************/
int HS_Int_Move::SmoothMaxJAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJAcc,double &dKJVel)
{
	double dSetMaxAcc = tSyncCur.tPreHandle.dSetMaxAcc;

	if(dSetMaxAcc < tSyncNex.tPreHandle.dSetMaxAcc)	
		dSetMaxAcc = tSyncNex.tPreHandle.dSetMaxAcc;

	double dJJAcc1[MaxAxisNum] = {0};
	double dJJAcc2[MaxAxisNum] = {0};

    GetPlanAcc(tSyncCur,dTInPre,dJJAcc1);
    GetPlanAcc(tSyncNex,dTInNext,dJJAcc2);

	double dJAcc[MaxAxisNum] = {0};
	for(int i = 0;i < MaxAxisNum;i++)
	{	
		dJAcc[i] = dJJAcc1[i] + dJJAcc2[i];
		double dTemp = 0;
		if(m_dJAccPara[i] > Eps)
			dTemp = fabs(dJAcc[i]/m_dJAccPara[i]);
		double dKDec = dSetMaxAcc;		
        dTemp = dTemp/dKDec;
		dKJAcc = Max(dKJAcc,dTemp);
	}  
	return 0;
}
/************************************************
函数功能：根据时间求解得到对应的时刻的关节速度、加速度比例信息
参    数：tSyncCur---当前段规划信息
        tSyncNex---下一段规划信息
        dTInPre----待求时刻值(当前段）
        dTInNext---待求时刻值（下一段）
        dKJAcc-----关节加速度比例
        dKJVel-----关节速度比例
		bRealCalc--不考虑约束处理只计算实际加速度的方案
返 回 值：错误ID
*************************************************/
int HS_Int_Move::SmoothMaxJParaCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJVel,double &dKJAcc,bool bRealCalc)
{
	if(tSyncCur.tPreHandle.bWristQyFlag||tSyncNex.tPreHandle.bWristQyFlag)
	{
		dKJAcc = 0;
		dKJVel = 0;
		LOG_ALGO("WristQy Smooth No AccCalc!");
		return 0;
	}

    double dSetMaxAcc = tSyncCur.tPreHandle.dSetMaxAcc;

    if(dSetMaxAcc < tSyncNex.tPreHandle.dSetMaxAcc)	
        dSetMaxAcc = tSyncNex.tPreHandle.dSetMaxAcc;

	//平滑段加速度约束处理需要对时间进行区分，对于小线段，平滑段时间较短的情况，无需进行放大处理
	bool bKMaxAccProFlag = true;
	double dTSmoothLimit = m_dCycle*5;
	if(tSyncCur.tVelPlan.dTime[TDEC] < dTSmoothLimit||tSyncNex.tVelPlan.dTime[TACC] < dTSmoothLimit)
		bKMaxAccProFlag = false;

    double dJVel1[MaxAxisNum] = {0};
    double dJVel2[MaxAxisNum] = {0};
    double dJAcc1[MaxAxisNum] = {0};
    double dJAcc2[MaxAxisNum] = {0};

    //获取速度和加速度，进行叠加处理
    GetPlanPara(tSyncCur,dTInPre,dJVel1,dJAcc1);
    GetPlanPara(tSyncNex,dTInNext,dJVel2,dJAcc2);

    double dJAcc[MaxAxisNum] = {0};
    double dJVel[MaxAxisNum] = {0};
	dKJAcc = 0;
    for(int i = 0;i < MaxAxisNum;i++)
    {	
        dJVel[i] = dJVel1[i] + dJVel2[i];
        dJAcc[i] = dJAcc1[i] + dJAcc2[i];
        double dTemp = 0;
        if(m_dJAccPara[i] > Eps)
            dTemp = fabs(dJAcc[i]/m_dJAccPara[i]);
        double dKDec = dSetMaxAcc;		

        //【240620修改】未防止平滑均变成前后段一致，放大加速度约束比例，平滑段的最大加速度约束，可以放大至单段设置的2倍，且不超过200
		if(bKMaxAccProFlag&&!bRealCalc)
		{
			double dKProtect = dSetMaxAcc*2;
			if(dKProtect > 2.0)
				dKProtect = 2.0;
			if(dKDec < dKProtect)
				dKDec = dKProtect;
		}

        dTemp = dTemp/dKDec;
        dKJAcc = Max(dKJAcc,dTemp);
        
        dTemp = 0;
        if(m_dJVelPara[i] > Eps)
            dTemp = fabs(dJVel[i]/m_dJVelPara[i]);
        dKJVel = Max(dTemp,dKJVel);
    } 
    return 0;
}
/************************************************
函数功能：根据时间求解得到对应的规划速度、加速度信息
参    数：tSyncPara---规划信息
		 dTime-------待求解时刻
		 dPlanVel----转换得到规划速度信息
         dPlanAcc----转换得到规划加速度信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetPlanPara(SyncPara &tSyncPara,double dTime,double dPlanVel[MaxAxisNum],double dPlanAcc[MaxAxisNum])
{
    if(tSyncPara.tPreHandle.eMoveType == MP_Joint)
    {
        double dKaAcc[MaxAxisNum] = {0};
        double dKbAcc[MaxAxisNum] = {0};
        double dTAcc = tSyncPara.tVelPlan.dTime[TACC];
        for(int i = 0;i < MaxAxisNum;i++)
        {
            dKaAcc[i] = -tSyncPara.tVelPlan.dEVel[i]/(2*dTAcc*dTAcc*dTAcc);
            dKbAcc[i] = tSyncPara.tVelPlan.dEVel[i]/(dTAcc*dTAcc);	
        }

        double dKaDec[MaxAxisNum] = {0};
        double dKbDec[MaxAxisNum] = {0};
        double dTDec= tSyncPara.tVelPlan.dTime[TDEC];
        for(int i = 0;i < MaxAxisNum;i++)
        {
            dKaDec[i] = tSyncPara.tVelPlan.dEVel[i]/(2*dTDec*dTDec*dTDec);
            dKbDec[i] = -tSyncPara.tVelPlan.dEVel[i]/(dTDec*dTDec);	
        }

        if(dTime < dTAcc)
        {
            for(int i = 0;i < MaxAxisNum;i++)
            {
                dPlanVel[i] = dTime*dTime*(4*dKaAcc[i]*dTime + 3*dKbAcc[i]);
                dPlanAcc[i] = 12*dKaAcc[i]*dTime*dTime + 6*dKbAcc[i]*dTime;
            }
        }
        else if(dTime < dTAcc + tSyncPara.tVelPlan.dTime[TCON])
        {
            for(int i = 0;i < MaxAxisNum;i++)
            {
                dPlanVel[i] = tSyncPara.tVelPlan.dEVel[i];
                dPlanAcc[i] = 0;
            }
        }
        else
        {
            dTime = tSyncPara.tVelPlan.dTime[TALL] - dTime;
            for(int i = 0;i < MaxAxisNum;i++)
            {
                dPlanVel[i] = dTime*dTime*(4*dKaDec[i]*dTime + 3*dKbDec[i]);
                dPlanAcc[i] = 12*dKaDec[i]*dTime*dTime + 6*dKbDec[i]*dTime;
            }
        }		
    }
    else if(tSyncPara.tPreHandle.eMoveType == MP_Line||tSyncPara.tPreHandle.eMoveType == MP_Arc)
    {
         int iErrorId = GetJParaByTime(dTime,tSyncPara,dPlanAcc,dPlanVel);	
    }
    return 0;
}
/************************************************
函数功能：根据时间求解得到对应的规划速度信息
参    数：tSyncPara---规划信息
		 dTime-------待求解时刻
		 dPlanVel----转换得到规划速度信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetPlanVel(SyncPara &tSyncPara,double dTime,double dPlanVel[MaxAxisNum])
{
	double dKaAcc[MaxAxisNum] = {0};
	double dKbAcc[MaxAxisNum] = {0};
	double dTAcc = tSyncPara.tVelPlan.dTime[TACC];
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dKaAcc[i] = -tSyncPara.tVelPlan.dEVel[i]/(2*dTAcc*dTAcc*dTAcc);
		dKbAcc[i] = tSyncPara.tVelPlan.dEVel[i]/(dTAcc*dTAcc);	
	}

	double dKaDec[MaxAxisNum] = {0};
	double dKbDec[MaxAxisNum] = {0};
	double dTDec= tSyncPara.tVelPlan.dTime[TDEC];
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dKaDec[i] = tSyncPara.tVelPlan.dEVel[i]/(2*dTDec*dTDec*dTDec);
		dKbDec[i] = -tSyncPara.tVelPlan.dEVel[i]/(dTDec*dTDec);	
	}

	if(dTime < dTAcc)
	{
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanVel[i] = dTime*dTime*(4*dKaAcc[i]*dTime + 3*dKbAcc[i]);
	}
	else if(dTime < dTAcc + tSyncPara.tVelPlan.dTime[TCON])
	{
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanVel[i] = tSyncPara.tVelPlan.dEVel[i];
	}
	else
	{
		dTime = tSyncPara.tVelPlan.dTime[TALL] - dTime;
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanVel[i] = dTime*dTime*(4*dKaDec[i]*dTime + 3*dKbDec[i]);
	}		
	return 0;
}
/************************************************
函数功能：根据时间求解得到对应的规划速度信息
参    数：tSyncPara---规划信息
		 dTime-------待求解时刻
		 dPlanVel----转换得到规划速度信息
返 回 值：错误ID
*************************************************/
int HS_Int_Move::GetPlanAcc(SyncPara &tSyncPara,double dTime,double dPlanAcc[MaxAxisNum])
{
	double dKaAcc[MaxAxisNum] = {0};
	double dKbAcc[MaxAxisNum] = {0};
	double dTAcc = tSyncPara.tVelPlan.dTime[TACC];
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dKaAcc[i] = -tSyncPara.tVelPlan.dEVel[i]/(2*dTAcc*dTAcc*dTAcc);
		dKbAcc[i] = tSyncPara.tVelPlan.dEVel[i]/(dTAcc*dTAcc);	
	}

	double dKaDec[MaxAxisNum] = {0};
	double dKbDec[MaxAxisNum] = {0};
	double dTDec= tSyncPara.tVelPlan.dTime[TDEC];
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dKaDec[i] = tSyncPara.tVelPlan.dEVel[i]/(2*dTDec*dTDec*dTDec);
		dKbDec[i] = -tSyncPara.tVelPlan.dEVel[i]/(dTDec*dTDec);	
	}

	if(dTime < dTAcc)
	{
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanAcc[i] = 12*dKaAcc[i]*dTime*dTime + 6*dKbAcc[i]*dTime;
	}
	else if(dTime < dTAcc + tSyncPara.tVelPlan.dTime[TCON])
	{
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanAcc[i] = 0;
	}
	else
	{
		dTime = tSyncPara.tVelPlan.dTime[TALL] - dTime;
		for(int i = 0;i < MaxAxisNum;i++)
			dPlanAcc[i] = 12*dKaDec[i]*dTime*dTime + 6*dKbDec[i]*dTime;
	}		
	return 0;
}
/************************************************
函数功能：计算平滑段对应的时间保护修调系数
参    数：dTFreProtect---时间保护值
		 dTSmooth-------平滑段时间
         dKJAcc---------加速度或电流比例系数
返 回 值：时间修调系数
*************************************************/
double HS_Int_Move::CalcKJerkSmooth(double dKJAcc,double dTFreProtect,double dTSmooth)
{
    double dKJerk = 0;
    if(dTSmooth  < dTFreProtect)
    {
        //最小时间计算，应该与电流比例值或者加速度比例值相关，电流比例值越小，时间应该越小
        double dT = dKJAcc*dTFreProtect; 
        if(dT > dTFreProtect)
            dT = dTFreProtect;

        if(dT > Eps)
        {
            dKJerk = dT/dTSmooth;
        }
    }
    return dKJerk;
}

/************************************************
函数功能：检测点位是否有效点位
参    数：trajout---基地址
		 index-----索引号
返 回 值：点位状态
         0---------点位有效
         -1---------无点位
         -2---------有点位未预处理或重复点位
         其他---------有点位预处理但预处理报警
*************************************************/
int HS_Int_Move::CheckTrajDataAvailable(GroupTrajData *trajout,int index)
{
	int iAvailable = 0;

	Para_PreHandle tPreHandle;

	memcpy(&tPreHandle,trajout[index].iData[m_iGroupNum],sizeof(Para_PreHandle));    

	if((tPreHandle.eMoveType > HS_MOVETYPE_MIN&&tPreHandle.eMoveType < HS_MOVETYPE_MAX)&&trajout[index].tMotionData.iLineNum != -1)
	{
		if(tPreHandle.bPreHandled&&!tPreHandle.bRepeatPos)
		{
			if(tPreHandle.iPreHandledError == 0)
			{
				iAvailable = 0;
			}
			else
			{
				iAvailable = tPreHandle.iPreHandledError;                
			}
		}
		else
		{
			iAvailable = -2;
		}
	}
	else
		iAvailable = -1;

	return iAvailable;
}
/************************************************
函数功能：检测运行段预处理是否有效
参    数：trajout---基地址
		 index-----索引号
返 回 值：错误码
*************************************************/
int HS_Int_Move::CheckPreHandle(GroupTrajData *tTrajData,int index)
{ 
    memcpy(&m_tSync.tPreHandle,tTrajData[index].iData[m_iGroupNum],sizeof(Para_PreHandle));
    m_bRepeatPosFlag = false;
    if(m_tSync.tPreHandle.bPreHandled)
    {
        if(m_tSync.tPreHandle.iPreHandledError != 0)
        {
            LOG_ALGO("PreHandledError = %d!",m_tSync.tPreHandle.iPreHandledError);
            return m_tSync.tPreHandle.iPreHandledError;
        }

        if(m_tSync.tPreHandle.bRepeatPos)
        {
            m_bRepeatPosFlag = true;
            LOG_ALGO("Repreat Pos!");
            return W_REPEAT;
        }
    }
    else
    {
        LOG_ALGO("PreHandledError = %d!",m_tSync.tPreHandle.iPreHandledError);
        return ERROR_NOPREHANDLED;
    }

	//过腕部奇异功能
	m_HS_Kinematic->HS_SetWristQYHandle(m_tSync.tPreHandle.bWristQyFlag);
    return 0;
}
/************************************************
函数功能：检测点位是否有效点位
参    数：trajout---基地址
		 index-----索引号
         dEndJPos---上一段的结束位置
返 回 值：是否成功         
*************************************************/
bool HS_Int_Move::GetPreMovePara(GroupTrajData *trajout,int index,double dEndJPos[MaxAxisNum])
{
	int iPreIndex = index - 1;
	if(iPreIndex < 0)
		iPreIndex = MaxBuffSize - 1;

	if(trajout[iPreIndex].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType > HS_MOVETYPE_MIN
		&&trajout[iPreIndex].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType < HS_MOVETYPE_MAX)
	{
		Para_PreHandle tPreHandle;

		memcpy(&tPreHandle,trajout[iPreIndex].iData[m_iGroupNum],sizeof(Para_PreHandle));  

		if(tPreHandle.bPreHandled)
		{
			memcpy(dEndJPos,tPreHandle.dSetJPos[1],sizeof(double)*MaxAxisNum);
		}
		else
			return false;
	}
	else
		return false;


	return true;
}
/************************************************
函数功能：与下一段的平滑是否存在
参    数：
返 回 值：存在平滑，返回true         
*************************************************/
bool HS_Int_Move::GetSmoothNextFlag()
{
    bool bSmoothNextFlag = false;

    if(m_tGTrajData[m_iIndex].tMotionData.dCnt > Eps&&m_tSync.tVelPlan.dTSmooth[NEX] > Eps)
        bSmoothNextFlag = true;

    return bSmoothNextFlag;
}
/************************************************
函数功能：是否运行至平滑中间时刻
参    数：
返 回 值：        
*************************************************/
bool HS_Int_Move::GetHalfSmoothFlag()
{
	if((m_dTCur > m_tSync.tVelPlan.dTSmooth[PRE]/2 - m_dCycle))
		return true;
	else 
		return false;
	return true;
}

/************************************************
函数功能：获取平滑拐入点的提前信号量
参    数：
返 回 值：true-----拐入信号量
*************************************************/
bool HS_Int_Move::GetSmoothAheadFlag()
{
	double dAheadTime = 0.05;
	if(m_bSmoothNextFlag)
	{
		if(m_dTCur - (m_dTAll - m_tSync.tVelPlan.dTSmooth[NEX] - m_dCycle - dAheadTime) > Eps&&m_tSync.tVelPlan.dTSmooth[NEX] > Eps)
		{ 
			return true;
		}
	}
	return false;
}
/************************************************
函数功能：当前段是否为小线段规划【特定的滤波处理】
参    数：
返 回 值：小线段规划返回true         
*************************************************/
bool HS_Int_Move::GetSLPlanFlag()
{
    return m_bSLPlagFlag;
}

/************************************************
函数功能：点位检测，判断点位是否通过奇异区间，如果通过，则不可运行
参    数：
		 tPreHandle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Move::CheckQYPass(Para_PreHandle &tPreHandle)
{
    int iErrorId = 0;
    if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA)
    {
        if((tPreHandle.dSetJPos[0][4]*tPreHandle.dSetJPos[1][4] < -Eps))
        {
			if(tPreHandle.eMoveType == MP_Line)
				return E_L_QYPASSWRIST;
			else if(tPreHandle.eMoveType == MP_Arc)
				return E_C_QYPASSWRIST;
        }
    }
    else if(m_HS_Kinematic->GetRobotType() == HSROB_SCARA)
    {
        if((tPreHandle.dSetJPos[0][1]*tPreHandle.dSetJPos[1][1] < -Eps))
        {
			if(tPreHandle.eMoveType == MP_Line)
				return E_L_QYPASSWRIST;
			else if(tPreHandle.eMoveType == MP_Arc)
				return E_C_QYPASSWRIST;
        }
    }

    return iErrorId;
}


SyncPara HS_Int_Move::GetSyncPara()
{
	return m_tSync;
}

int HS_Int_Move::SetSyncParaPlan(SyncPara tSyncPara)
{
	m_tSync = tSyncPara;

	for(int i = 0;i < MaxAxisNum;i++)
	{
		m_tVelPlanPara[i].eTypeAcc = APType_Para;
		m_tVelPlanPara[i].dTCon = m_tSync.tVelPlan.dTime[TCON];
		m_tVelPlanPara[i].dTAcc = m_tSync.tVelPlan.dTime[TACC];
		m_tVelPlanPara[i].dTDec = m_tSync.tVelPlan.dTime[TDEC];
		m_tVelPlanPara[i].dTAll = m_tSync.tVelPlan.dTime[TALL];
		m_tVelPlanPara[i].dTSmoothOff = m_dTSmoothOff[m_iGroupNum];
		m_tVelPlanPara[i].dDis = m_tSync.tPreHandle.dSetDis[i];
		m_tVelPlanPara[i].dEVel = m_tSync.tVelPlan.dEVel[i];

		//if(tFilterControl.bFilterOpenFlag)
		//{
		//	m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i],tFilterControl.tFilterPara);
		//}
		//else
			m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i]);		
	}	

	LOG_ALGO("Sync:TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
		m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);

	m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
	m_dTCon = m_tSync.tVelPlan.dTime[TCON];
	m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
	m_dTAll = m_tSync.tVelPlan.dTime[TALL];
	return 0;
}

int HS_Int_Move::GetMasterCPos(double dMasterCPos[10][MaxAxisNum])
{
	memcpy(dMasterCPos,m_dMasterCPos,sizeof(m_dMasterCPos));
	return 0;
}

int HS_Int_Move::SetMasterCPos(double dMasterCPos[10][MaxAxisNum])
{
	memcpy(m_dMasterCPos,dMasterCPos,sizeof(m_dMasterCPos));
	return 0;
}

/************************************************
函数功能：协同功能检测
参   数：
		tHS_GroupRel----协同关系
		tPreHandle--------预处理信息
返 回 值：无
*************************************************/
int HS_Int_Move::GroupSyncCheck(HS_GroupRel tHS_GroupRel,Para_PreHandle &tPreHandle)
{
	if(tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Slave)
	{
		tPreHandle.bGroupSyncFlag = true;
		for(int i = 0;i < MAXGROUPNUM;i++)
		{
			if(tHS_GroupRel.eGroupRelType[i] == GRT_Master)
			{
				tPreHandle.iSyncMasterNum = i;
				break;
			}
		}
	}
	return 0;
}

/************************************************
函数功能：多转轴角度的修正
参    数：
		 tPH_Line---处理缓存结构体
         dEJPos-----预测得到的关节点位
返 回 值：错误ID
*************************************************/
int HS_Int_Move::JPosAutoHandle(Para_PreHandle &tPreHandle,double dEJPos[6],int iAxis)
{
	const double dErrorLimit = 2.0;
	
	if(tPreHandle.dSetJPos[1][iAxis] - tPreHandle.dSetJPos[0][iAxis] > dErrorLimit&&
		dEJPos[iAxis] - tPreHandle.dSetJPos[0][iAxis] < -dErrorLimit)
	{
		tPreHandle.dSetJPos[1][iAxis] -= 360;
		LOG_ALGO("Axis%d ReCalc JPos Backward = %.3lf,PreJPos = %.3lf",
			iAxis,tPreHandle.dSetJPos[1][iAxis],dEJPos[iAxis]);
	}
	else if(tPreHandle.dSetJPos[1][iAxis] - tPreHandle.dSetJPos[0][iAxis] < -dErrorLimit&&
		dEJPos[iAxis] - tPreHandle.dSetJPos[0][iAxis] > dErrorLimit)
	{
		tPreHandle.dSetJPos[1][iAxis] += 360;
		LOG_ALGO("Axis%d ReCalc JPos Backward = %.3lf,PreJPos = %.3lf",
			iAxis,tPreHandle.dSetJPos[1][iAxis],dEJPos[iAxis]);
	}

	return 0;
}
/************************************************
函数功能：关节重复点位检测
参    数：
		 dJPosA-----检测点A
         dJPosB-----检测点B
返 回 值：true---重复点位
*************************************************/
bool HS_Int_Move::bCheckRepeatJPos(double dJPosA[MaxAxisNum],double dJPosB[MaxAxisNum])
{
	bool bJErrorFlag = true;
	const double dErrorLimit = 0.001;
	for(int i = 0;i < MaxAxisNum;i++)
	{
		if(fabs(dJPosA[i] - dJPosB[i]) > dErrorLimit)
		{
			bJErrorFlag = false;
			break;
		}
	}
	return bJErrorFlag;
}
/************************************************
函数功能：获取设置的工具工件号
参   数：elemt----运动输入信息
返 回 值：无
*************************************************/
void HS_Int_Move::GetToolWorkNum(BaseMoveData tMoveData)
{
	if(tMoveData.sCurCoordinate.iCoordinate == BASE_COORD_SYSTEM||tMoveData.sCurCoordinate.iCoordinate == JOINT_COORD_SYSTEM)
	{
		m_iToolNum = -1;
		m_iWorkNum = -1;
	}
	else if(tMoveData.sCurCoordinate.iCoordinate == USER_COORD_SYSTEM||tMoveData.sCurCoordinate.iCoordinate == TOOL_COORD_SYSTEM)
	{
		m_iToolNum = tMoveData.sCurCoordinate.iToolNum;
		m_iWorkNum = tMoveData.sCurCoordinate.iWorkNum;
	}

	m_HS_Kinematic->HS_PrintCoordinate(m_iToolNum,m_iWorkNum);

	LOG_ALGO("Move iCoordinate = %d,Tool = %d,Work = %d",tMoveData.sCurCoordinate.iCoordinate,m_iToolNum,m_iWorkNum);
}

/************************************************
函数功能：组协同点位转换处理
参   数：tGroupMotionData---运行段信息
		tPreHandle-----------预处理信息
返 回 值：无
*************************************************/
int HS_Int_Move::GroupSyncPosChange(Para_PreHandle &tPreHandle)
{
	if(tPreHandle.bGroupSyncFlag)
	{
		Para_PreHandle tPreHandleMaster = Para_PreHandle();

		memcpy(&tPreHandleMaster,m_tGTrajData[m_iIndex].iData[tPreHandle.iSyncMasterNum],sizeof(Para_PreHandle));

		m_HS_Kinematic->HS_SyncRelativePosChange(tPreHandleMaster.dWTSPos,tPreHandle.dWTSPos,tPreHandle.dSPos);
		m_HS_Kinematic->HS_SyncRelativePosChange(tPreHandleMaster.dWTEPos,tPreHandle.dWTEPos,tPreHandle.dEPos);
	}

	if(tPreHandle.bCoorperMoveFlag)
	{
		m_HS_Kinematic->HS_TWPosToTCPos(tPreHandle.dWTSPos,tPreHandle.dSPos);
		m_HS_Kinematic->HS_TWPosToTCPos(tPreHandle.dWTEPos,tPreHandle.dEPos);
	}

	return 0;
}

/************************************************
函数功能：对运动的结束点进行粗预测
参    数：
		 tPreHandle---处理缓存结构体
         dEJPos-----预测得到的关节点位
返 回 值：错误ID
*************************************************/
int HS_Int_Move::JPosPrediction(Para_PreHandle &tPreHandle,double dEJPos[6])
{
    int iErrorId = 0;

    SyncPara tSync;
    tSync.tPreHandle = tPreHandle;

    double dPlanPos[SpaceAxisNum] = {0};
    double dMPos[5][4] = {0};

    const int DIFFCNT = 15;
    double dSJPos[6] = {0};
    
    memcpy(dSJPos,tPreHandle.dSetJPos[0],sizeof(double)*6);

    double dJPosCalc[6] = {0};
    for(int i = 0;i < DIFFCNT;i++)
    {
        for(int j = 0;j < 2;j++)
        {
            dPlanPos[j] = tPreHandle.dSetDis[j]*(i+1)/DIFFCNT; 
        }

        PlanPos2MPos(tSync,dPlanPos,dMPos);

        iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos,m_iToolNum,m_iWorkNum,CP_ToolWork,dSJPos,dJPosCalc);
        if(iErrorId != 0)
            return iErrorId;

		if(dJPosCalc[4]*tPreHandle.dSetJPos[0][4] < -Eps)
		{
			//腕部形态改变代表求解异常，求解得到的临近点位不可信
			return -1;
		}

        memcpy(dSJPos,dJPosCalc,sizeof(double)*6);
    }

    memcpy(dEJPos,dJPosCalc,sizeof(double)*6);

    return iErrorId;
}

/************************************************
函数功能：对运动规划的参数信息进行调整，包含结束点关节坐标，以及A360处理等
参    数：
		 tPreHandle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Move::MoveAdjust(Para_PreHandle &tPreHandle)
{
    int iErrorId = 0;

    if(m_eHS_RobotType == HSROB_SCARA)
    {
        double dEJPos[6] = {0};
        iErrorId = JPosPrediction(tPreHandle,dEJPos);
        if(iErrorId != 0) return iErrorId;
        
        if(m_HS_Kinematic->GetA360Flag())
        {
            //当前姿态角度旋转对应关节4轴旋转的方向
            bool bAngleDir = true;
            if(dEJPos[3] - tPreHandle.dSetJPos[0][3] < 0)
            {
                bAngleDir = false;
            }
            //A360模式下，根据目标点位置以及预测的关节角度值，通过修改姿态运动位移量的方式使得运动达到目标点的4轴坐标值
            while(fabs(dEJPos[3] - tPreHandle.dSetJPos[1][3]) > 181)
            {
                if(dEJPos[3] > tPreHandle.dSetJPos[1][3])
                {
                    dEJPos[3] -= 360;
                    if(bAngleDir)
                        tPreHandle.dSetDis[1] -= 360;
                    else
                        tPreHandle.dSetDis[1] += 360;
                }
                else
                {
                    dEJPos[3] += 360;
                    if(bAngleDir)
                        tPreHandle.dSetDis[1] += 360;
                    else
                        tPreHandle.dSetDis[1] -= 360;
                }
            }
        }
        else
        {
            m_HS_Kinematic->HS_NearestPoint(tPreHandle.dSetJPos[1][3],dEJPos[3],-1);
			JPosAutoHandle(tPreHandle,dEJPos,3);
        }
    }
	else if(m_eHS_RobotType == HSROB_PUMA)
	{
		double dEJPos[6] = {0};
		iErrorId = JPosPrediction(tPreHandle,dEJPos);
		if(iErrorId == -1)
		{
			iErrorId = 0;
		}
		else
		{
			if(iErrorId != 0) return iErrorId;
			m_HS_Kinematic->HS_NearestPoint(tPreHandle.dSetJPos[1][0],dEJPos[0],-1);
			m_HS_Kinematic->HS_NearestPoint(tPreHandle.dSetJPos[1][3],dEJPos[3],-1);
			m_HS_Kinematic->HS_NearestPoint(tPreHandle.dSetJPos[1][5],dEJPos[5],-1);

			JPosAutoHandle(tPreHandle,dEJPos,3);
			JPosAutoHandle(tPreHandle,dEJPos,5);
		}
	}

    return iErrorId;
}

/************************************************
函数功能：插补规划更新点位状态
参    数：
		 intdata----关节点位输出
		 iErrorId---错误码
返 回 值：Busy---运行中
		 M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Move::UpdateIntMove(IntData &intdata,int &iErrorId)
{
	HS_MStatus eMStatus = M_Busy;
	//LOG_ALGO("------------------------------------------------------------------------");
	for(int i = 0;i < m_iInterMultCnt;i++)
	{
		memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
		if(!m_bPlanFlag)
		{
			//存在重复点位，不进行后续的运动，防止报错
			eMStatus = M_Done;            
			continue;
		}

		if(i == m_iInterMultCnt -1)
		{
			//获取最后一个周期的状态和运行百分比
			eMStatus = Move(iErrorId,true);
			intdata.dPercent = m_dPercent;
			intdata.dDisStart = m_dDisStart;
			intdata.dDisEnd = m_dDisEnd;
		}
		else
		{
			Move(iErrorId);
		}
		if(iErrorId != 0)
		{
			eMStatus = M_Error;
			break;
		}

		iErrorId = UpdateJPos(i);

		if(iErrorId != 0)
		{
			eMStatus = M_Error;
			break;
		}
		//LOG_ALGO("InterJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],\
		m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);     
		memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_dRJPos,sizeof(double)*MaxAxisNum);
		if(eMStatus == M_Done)
		{
			LOG_ALGO("Done JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
				m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);

			MoveDoneJPosCorrect();
		}
	}

	if(m_bTriggerSmoothFlag)
	{
		m_bTriggerSmoothFlag = false;
		if(m_bWeaveFlag)
		{
			m_bSmoothSynthFlag[m_iGroupNum] = true;
			m_bTimeInNextSmooth = true;
		}
		else
		{
			m_bSmoothSynthFlag[m_iGroupNum] = false;
		}
		LOG_ALGO("SynthSmooth = %d",(int)m_bSmoothSynthFlag[m_iGroupNum]);
	}
	//LOG_ALGO("------------------------------------------------------------------------");
	return eMStatus;
}

/************************************************
函数功能：空间位置到关节为止的点位转换处理
参    数：
	iInterId----插补索引号
返 回 值：错误码
*************************************************/
int HS_Int_Move::UpdateJPos(int iInterId)
{
	int iErrorId = 0;

	if(m_bSmoothSynthFlag[m_iGroupNum]&&!m_bTimeInNextSmooth)
	{
		double dLMPos[5][4] = {0};

		memcpy(dLMPos,m_dLMPos[iInterId][m_iGroupNum],sizeof(m_dRMPos));

		//运动合成，合成后再逆解
		double dSJMPos[4][4] = {0};
		m_HS_Kinematic->HS_CPosToMPos(m_tSync.tPreHandle.dSPos,dSJMPos);
		HS_Math::Matrix_Inverse(4,&dSJMPos[0][0],&dSJMPos[0][0]);

		double dMatrixCJ[4][4] = {0};	
		HS_Math::Matrix_Multi(4,4,4,&dSJMPos[0][0],&m_dRMPos[0][0],&dMatrixCJ[0][0]);	

		//合成位置
		double dMaxtrixMix[4][4] = {0};
		HS_Math::Matrix_Multi(4,4,4,&dLMPos[0][0],&dMatrixCJ[0][0],&m_dRMPos[0][0]);	

		//附加轴
		for(int i = 0;i < 3;i++)
		{
			m_dRMPos[4][i] = dLMPos[4][i] + (m_dRMPos[4][i] - m_tSync.tPreHandle.dSPos[6+i]);
		}
	}

	if(m_tSync.tPreHandle.bCoorperMoveFlag)
	{
		double dTWMPos[5][4] = {0};
		m_HS_Kinematic->HS_TCMPosToTWMPos(m_dRMPos,dTWMPos);

		double dLJPos[MaxAxisNum];
		memcpy(dLJPos,m_dRJPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_MPosToJPos(dTWMPos,CP_ToolWork,dLJPos,m_dRJPos);
	}
	else if(m_tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Slave)
	{
		double dSlaveMPos[5][4] = {0};

		m_HS_Kinematic->HS_SyncRelativeToSlave(m_dMasterCPos[iInterId],m_dRMPos,dSlaveMPos);

		double dLJPos[MaxAxisNum];
		memcpy(dLJPos,m_dRJPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_MPosToJPos(dSlaveMPos,CP_ToolWork,dLJPos,m_dRJPos);
	}
	else if(m_bWeaveFlag||m_bStopSineNextFlag)
	{
		double dWeaveMPos[5][4] = {0};

		if(!m_bTimeInNextSmooth)
		{
			WeavePosCalc(&dWeaveMPos[0][0],m_bStopFlag);

			double dLJPos[MaxAxisNum];
			memcpy(dLJPos,m_dRJPos,sizeof(double)*MaxAxisNum);
			memcpy(m_dWeaveStopMainJPos,m_dRJPos,sizeof(double)*MaxAxisNum);

			if(m_bStopDoneFlag&&iInterId == m_iInterMultCnt -1)
			{
				m_bSmoothSynthFlag[m_iGroupNum] = false;
				m_bWeaveFlag = false;
				iErrorId = m_HS_Kinematic->HS_MPosToJPos(m_dRMPos,CP_ToolWork,dLJPos,m_dWeaveStopMainJPos);
			}

			iErrorId = m_HS_Kinematic->HS_MPosToJPos(dWeaveMPos,CP_ToolWork,dLJPos,m_dRJPos);

			if(iErrorId != 0)
			{
				LOG_ALGO("RMPos = %.3lf,%.3lf,%.3lf",m_dRMPos[0][3],m_dRMPos[1][3],m_dRMPos[2][3]);
				LOG_ALGO("WeaveMPos = %.3lf,%.3lf,%.3lf",dWeaveMPos[0][3],dWeaveMPos[1][3],dWeaveMPos[2][3]);
			}
		}
	}
	else 
	{
		double dLJPos[MaxAxisNum];
		memcpy(dLJPos,m_dRJPos,sizeof(double)*MaxAxisNum);
		iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(m_dRMPos,m_iToolNum,m_iWorkNum,CP_ToolWork,dLJPos,m_dRJPos,m_tSync.tPreHandle.bWristQyFlag);

		if(m_tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Master)
		{
			m_HS_Kinematic->HS_JPosToCPos(m_dRJPos,m_iToolNum,-1,m_dMasterCPos[iInterId]);
		}
	}

	//附加轴  
	for(int i = 0;i < 3;i++)
	{
		m_dRJPos[6+i] = m_dRMPos[4][i];
	}

	if(m_bTimeInNextSmooth)
	{
		memcpy(m_dLMPos[iInterId][m_iGroupNum],m_dRMPos,sizeof(m_dRMPos));
	}

	return iErrorId;
}

/************************************************
函数功能：停止运动运行
参   数：
返 回 值：运行状态
*************************************************/
HS_MStatus HS_Int_Move::StopMoveHandle(double dMovePos[MaxAxisNum])
{
	int iAxisNum = SpaceAxisNum;
	if(m_eMoveType == MP_Joint)
		iAxisNum = MaxAxisNum;

	HS_MStatus eMStatus = M_Busy;
	if(m_dTCur < m_dTStop)
	{
		for(int i = 0;i < iAxisNum;i++)
		{
			dMovePos[i] = m_dTCur*m_dTCur*m_dTCur*(m_dStopKa[i]*m_dTCur + m_dStopKb[i]) + 
				m_dTCur*(m_dStopKc[i]*m_dTCur + m_dMoveVel[i]);
		}
	}
	else
	{
		for(int i = 0;i < iAxisNum;i++)
		{
			dMovePos[i] = m_dStopDis[i];
		}
		//eMStatus = M_Done;
		eMStatus = M_StopDone;
	}

	for(int i = 0;i < iAxisNum;i++)
	{
		dMovePos[i] += m_dStopSPos[i];
	}

	//摆焊添加：等待摆焊暂停结束后再返回Done
	if(m_bWeaveFlag && (eMStatus == M_StopDone))
	{
		if(!m_bWeaveStopDoneFlag) 
			return  M_Busy;
		else
			m_bStopDoneFlag = true;
	}

	if(fabs(m_tSync.tPreHandle.dDis[m_tSync.tPreHandle.iMaxAxis]) > Eps)
		m_dPercent = fabs(dMovePos[m_tSync.tPreHandle.iMaxAxis]/m_tSync.tPreHandle.dDis[m_tSync.tPreHandle.iMaxAxis]);	

	return eMStatus;
}

/************************************************
函数功能：计算速度，生成曲线【执行一次运动插补】
参    数：
        iErrorId------报警码
        bLastCycle----是否是多点插补的最后一个周期
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Move::IntMove(int &iErrorId,bool bLastCycle)
{
	int iAxisNum = SpaceAxisNum;
	if(m_eMoveType == MP_Joint)
		iAxisNum = MaxAxisNum;

	int iNum_Done = 0;
	HS_MStatus eMStatus = M_Busy;
	double dMovePos[MaxAxisNum] = {0};
	double dMovePosPre[MaxAxisNum] = {0};
    double dOrignPos[MaxAxisNum] = {0};
	if(m_bStopFlag)	//执行减速停止
	{
		eMStatus = StopMoveHandle(dMovePos);
	}
	else
	{	
        for(int i = 0;i < iAxisNum;i++)
        {
            HS_MStatus Status = m_HS_VelPlan_Para[i]->Move(dMovePos[i]);

            //m_HS_VelPlan_Para[i]->GetVel(m_dMoveVel[i]);
            //m_HS_VelPlan_Para[i]->GetAcc(m_dMoveAcc[i]);
            m_HS_VelPlan_Para[i]->GetOrignPos(dOrignPos[i]);
            m_HS_VelPlan_Para[i]->GetFilterVel(m_dMoveVel[i]);
            m_HS_VelPlan_Para[i]->GetFilterAcc(m_dMoveAcc[i]);

            if(Status == M_Done||Status == M_UnInit)
                iNum_Done++;

            if(Status == M_Error)
                eMStatus = M_Error;
		}	

        if(iNum_Done == iAxisNum)
        {
            //LOG_ALGO("Move Done!");
            eMStatus = M_Done;	

			//等待Stop摆焊模式端点停留完成
			if(m_bWeaveFlag&&(m_bStopSineWaitTFlag ||m_bStopWeaveMainWaitFlag))
			{
				eMStatus = M_Busy;
			}
        }

        //平滑处理
        if(m_bSmoothNextFlag&&bLastCycle)
        {
            if(m_dTCur - (m_dTAll - m_tSync.tVelPlan.dTSmooth[NEX] - m_dCycle) > Eps)
            { 
				m_bTriggerSmoothFlag = true;
                m_bSmoothPreFlag = true;				
                m_dTSmoothOff[m_iGroupNum] = m_dTCur - (m_dTAll - m_tSync.tVelPlan.dTSmooth[NEX]);
                if(m_dTSmoothOff[m_iGroupNum] > Eps || m_dTSmoothOff[m_iGroupNum] < -m_dCycle)	//避免与关节混合平滑等运动中时间计算错误
                {
                    m_dTSmoothOff[m_iGroupNum] = 0;
                }
                LOG_ALGO("TSmoothOff = %.3lf,TAll = %.3lf,TSmooth = %.3lf",m_dTSmoothOff[m_iGroupNum],m_dTAll,m_tSync.tVelPlan.dTSmooth[NEX]);

                m_bSmoothNextFlag = false;
                eMStatus = M_Done;
            }			
        }

        for(int i = 0;i < iAxisNum;i++)
        {
            dMovePos[i] += m_dStopPos[i];
            dOrignPos[i] += m_dStopPos[i];
        }
    }		

	if(m_eMoveType == MP_Joint)
	{
		for(int i = 0;i < iAxisNum;i++)
		{
			m_dRJPos[i] = m_tSync.tPreHandle.dSPos[i] + dMovePos[i];
		}
	}
	else
		PlanPos2MPos(m_tSync,dMovePos,m_dRMPos);

	//m_dMovePos代表相对于当前运动起点的位移量，提供给减速停止使用
	for(int i = 0;i < iAxisNum;i++)
	{
		m_dMovePos[i] = dMovePos[i];
        m_dMoveRatioPos[i] = dOrignPos[i];
	}	

	m_dTCur += m_dCycle;

    if(fabs(m_tSync.tPreHandle.dSetDis[m_tSync.tPreHandle.iMaxAxis]) > Eps)
        m_dPercent = fabs(dMovePos[m_tSync.tPreHandle.iMaxAxis]/m_tSync.tPreHandle.dSetDis[m_tSync.tPreHandle.iMaxAxis]);

	m_dDisStart = m_dMovePos[0];
	m_dDisEnd = m_tSync.tPreHandle.dLength - m_dDisStart;

	return eMStatus;				
}
