
/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Line.cpp
* 摘    要：直线运动同步插补算法

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HS_Int_Line.h"
/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Int_Line::~HS_Int_Line()
{
}

HS_Int_Line::HS_Int_Line(HS_GroupKin *pGroupKin)
{
	m_HS_GroupKin = pGroupKin;
	m_eMoveType = MP_Line;
}
/************************************************
函数功能：参数复位
参   数：    
返 回 值：错误码
*************************************************/
int HS_Int_Line::ResetData()
{
	memset(&m_tHS_GroupRel,0,sizeof(m_tHS_GroupRel));
	memset(&m_bSmoothMoveFlag,0,sizeof(m_bSmoothMoveFlag));
	memset(&m_bSmoothSynthFlag,0,sizeof(m_bSmoothSynthFlag));
	return 0;
}
/************************************************
函数功能：执行 运动的预处理过程
参   数：elemt----运动输入信息
		trajout---规划缓存信息    
返 回 值：错误码
*************************************************/
int HS_Int_Line::PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum)
{
	int iErrorId = 0;

	BaseMoveData *tMoveData = &tGroupMotionData.tBaseMoveData[iGroupNum];
	Para_PreHandle tPH_Line;

	GlobalPreHandle(tGroupMotionData,tTrajData,iIndex,iGroupNum,tPH_Line,tMoveData);

	while(1)
	{
		iErrorId = GetStartPos(*tMoveData,tPH_Line);
		if(iErrorId != 0) break;

		iErrorId = GetEndPos(*tMoveData,tPH_Line);
		if(iErrorId != 0) break;

		iErrorId = GroupSyncPosChange(tPH_Line);
		if(iErrorId != 0) break;

		iErrorId = GetMoveDis(tPH_Line);
		if(iErrorId != 0)
		{            
			break;
		}

		iErrorId = CheckPosLimit(tPH_Line);
		if(iErrorId != 0) 
		{
			LOG_ALGO("Check Pos Limit ErrorNum = %d",iErrorId);
			break;
		}

		if(!tPH_Line.bRepeatPos)
		{
			AutoBasePara(*tMoveData,tPH_Line);
			//if(tPH_Line.bWristQyFlag)
			//{

			//}
			//else
			{
				iErrorId = AutoAdjustPara(*tMoveData,tPH_Line);
				if(iErrorId != 0) 
				{
					if(iErrorId == ERROR_CPOSTOJPOS||iErrorId == ERROR_UNREACHABLE)
					{
						iErrorId = E_L_MOVEUNRABLE;
					}
					break;
				}
			}			
		}
		break;
	}

	//存储预处理数据
	tPH_Line.iPreHandledError = iErrorId;
	tPH_Line.eState = m_eState;
	memcpy(tTrajData[iIndex].iData[iGroupNum],&tPH_Line,sizeof(Para_PreHandle));	

	return iErrorId;
}

/************************************************
函数功能：获取 运动起点的坐标
参   数：elemt----运动输入信息
		tPH_Line---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Line::GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sStartPos.hs_coordinate;

    //前后段坐标匹配
    if(!(m_tGTrajData[m_iIndex].tMotionData.bStartMove&&sCoord.iCoordinate == JOINT_COORD_SYSTEM))
    {
        //获取前一段的数据
        double dEndJPos[MaxAxisNum] = {0};
        bool bRet = GetPreMovePara(m_tGTrajData,m_iIndex,dEndJPos);
        if(bRet)
        {
            memcpy(tPH_Line.dSetJPos[0],dEndJPos,sizeof(double)*MaxAxisNum);
            m_HS_Kinematic->HS_JPosToCPos(dEndJPos,m_iToolNum,m_iWorkNum,tPH_Line.dSPos);	
            m_eState = m_HS_Kinematic->HS_JPosToAState(tPH_Line.dSetJPos[0]);
        }
        else
        {
            LOG_ALGO("Error NoStartMove Can't Get Pre Data!");
            m_eState = 0;
            iErrorId = ERROR_STARTPOS;
        }
    }
	else if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
        //启动段
		double dSJPos[MaxAxisNum] = {0};
		memcpy(dSJPos,tMoveData.sStartPos.dPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_JPosToCPos(dSJPos,m_iToolNum,m_iWorkNum,tPH_Line.dSPos);	
		memcpy(tPH_Line.dSetJPos[0],dSJPos,sizeof(double)*MaxAxisNum);

        m_eState = m_HS_Kinematic->HS_JPosToAState(dSJPos);

        LOG_ALGO("Start JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
            dSJPos[0],dSJPos[1],dSJPos[2],dSJPos[3],dSJPos[4],dSJPos[5],dSJPos[6],dSJPos[7],dSJPos[8]);
	}
    else
    {
        LOG_ALGO("Error StartCPos With StartMoveFlag!");
        iErrorId = ERROR_STARTPOS;
    }

    LOG_ALGO("eState = %d",(int)m_eState);

	//协同点位
	m_HS_Kinematic->HS_JPosToCPos(tPH_Line.dSetJPos[0],m_iToolNum,-1,tPH_Line.dWTSPos);

	return iErrorId;
}

/************************************************
函数功能：获取 运动终点的坐标
参   数：elemt----运动输入信息
		tPH_Line---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Line::GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sEndPos.hs_coordinate;
	if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
		double dEJPos[MaxAxisNum] = {0};
		memcpy(dEJPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_JPosToCPos(dEJPos,m_iToolNum,m_iWorkNum,tPH_Line.dEPos);	
		memcpy(tPH_Line.dSetJPos[1],dEJPos,sizeof(double)*MaxAxisNum);

        LOG_ALGO("SetEnd JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
            dEJPos[0],dEJPos[1],dEJPos[2],dEJPos[3],dEJPos[4],dEJPos[5],dEJPos[6],dEJPos[7],dEJPos[8]);

        iErrorId = CheckQYPass(tPH_Line);
        if(iErrorId != 0)
        {
            LOG_ALGO("ErrorEndJPos Pass QY ErrorNum = %d",iErrorId);
        }
	}
	else
	{	
		double dECPos[MaxAxisNum] = {0};
		memcpy(dECPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		memcpy(tPH_Line.dEPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);

        LOG_ALGO("SetEnd CPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;Tool = %d,Work = %d,eState = %d",\
            dECPos[0],dECPos[1],dECPos[2],dECPos[3],dECPos[4],dECPos[5],dECPos[6],dECPos[7],dECPos[8],sCoord.iToolNum,sCoord.iWorkNum,(int)m_eState);

		m_HS_Kinematic->HS_CPosChangeCoord(dECPos,sCoord.iToolNum,sCoord.iWorkNum,m_iToolNum,m_iWorkNum,tPH_Line.dEPos);

        iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(dECPos,sCoord.iToolNum,sCoord.iWorkNum,m_eState,tPH_Line.dSetJPos[0],tPH_Line.dSetJPos[1]);
		if(iErrorId != 0)
		{            
			if(m_eHS_RobotType == HSROB_SCARA&&iErrorId == E_C2J_ATTITUDE)
			{
				LOG_ALGO("C2J Error! Scara Attitude UnReachable!");
				return E_L_ATTUNREABLE_Scara;
			}
            LOG_ALGO("EndCPos C2JError!");
            m_HS_Kinematic->HS_PrintCoord(sCoord.iToolNum,sCoord.iWorkNum);
			return E_L_TARGETUNREABLE;
		}
		//点位修正
		m_HS_Kinematic->HS_JPosToCPos(tPH_Line.dSetJPos[1],m_iToolNum,m_iWorkNum,tPH_Line.dEPos);
		/*LOG_ALGO("Get ECPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;",\
			tPH_Line.dEPos[0],tPH_Line.dEPos[1],tPH_Line.dEPos[2],tPH_Line.dEPos[3],tPH_Line.dEPos[4],tPH_Line.dEPos[5],
			tPH_Line.dEPos[6],tPH_Line.dEPos[7],tPH_Line.dEPos[8]);*/
	}

	m_HS_Kinematic->HS_JPosToCPos(tPH_Line.dSetJPos[1],m_iToolNum,-1,tPH_Line.dWTEPos);
	return iErrorId;
}
/************************************************
函数功能：获取 运动位移量等信息
参   数：tPH_Line---预处理输入输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Line::GetMoveDis(Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;

	CalcMovePara(tPH_Line);

	//过腕部奇异处理标识
	if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA&&tPH_Line.bWristQYOpenFlag)
	{
		PUMAWristQYCheck(tPH_Line);

		if(tPH_Line.bWristQyFlag)
		{
			iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(tPH_Line.dEPos,m_iToolNum,m_iWorkNum,m_eState,tPH_Line.dSetJPos[0],tPH_Line.dSetJPos[1],tPH_Line.bWristQyFlag);
			m_HS_Kinematic->HS_JPosToCPos(tPH_Line.dSetJPos[1],m_iToolNum,m_iWorkNum,tPH_Line.dEPos);
		}
	}
	
	if(!tPH_Line.bRepeatPos)
    {
		if(!tPH_Line.bWristQyFlag)
			MoveAdjust(tPH_Line);
    }

	memcpy(tPH_Line.dOrigDis,tPH_Line.dSetDis,sizeof(double)*SpaceAxisNum);

	LOG_ALGO("WristQyOpenFlag = %d,WristHandleFlag = %d",(int)tPH_Line.bWristQYOpenFlag,(int)tPH_Line.bWristQyFlag);
    LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Line.dSPos[0],tPH_Line.dSPos[1],tPH_Line.dSPos[2],tPH_Line.dSPos[3],tPH_Line.dSPos[4],tPH_Line.dSPos[5],
        tPH_Line.dSPos[6],tPH_Line.dSPos[7],tPH_Line.dSPos[8]);
    LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Line.dEPos[0],tPH_Line.dEPos[1],tPH_Line.dEPos[2],tPH_Line.dEPos[3],tPH_Line.dEPos[4],tPH_Line.dEPos[5],
        tPH_Line.dEPos[6],tPH_Line.dEPos[7],tPH_Line.dEPos[8]);
    LOG_ALGO("SJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Line.dSetJPos[0][0],tPH_Line.dSetJPos[0][1],tPH_Line.dSetJPos[0][2],tPH_Line.dSetJPos[0][3],tPH_Line.dSetJPos[0][4],tPH_Line.dSetJPos[0][5],
        tPH_Line.dSetJPos[0][6],tPH_Line.dSetJPos[0][7],tPH_Line.dSetJPos[0][8]);
    LOG_ALGO("EJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Line.dSetJPos[1][0],tPH_Line.dSetJPos[1][1],tPH_Line.dSetJPos[1][2],tPH_Line.dSetJPos[1][3],tPH_Line.dSetJPos[1][4],tPH_Line.dSetJPos[1][5],
        tPH_Line.dSetJPos[1][6],tPH_Line.dSetJPos[1][7],tPH_Line.dSetJPos[1][8]);

	return iErrorId;
}

/************************************************
函数功能：计算运动信息
参   数：tPH_Line---预处理输入输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Line::CalcMovePara(Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;

	//1、位置空间位移
	for(int i = 0;i < 3;i++)
		tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];

	tPH_Line.dLength = sqrt(tPH_Line.dDis[0]*tPH_Line.dDis[0] + tPH_Line.dDis[1]*tPH_Line.dDis[1] + tPH_Line.dDis[2]*tPH_Line.dDis[2]);

	if(tPH_Line.dLength  > Eps)
	{
		tPH_Line.dKXYZ[0] = tPH_Line.dDis[0]/tPH_Line.dLength;
		tPH_Line.dKXYZ[1] = tPH_Line.dDis[1]/tPH_Line.dLength;
		tPH_Line.dKXYZ[2] = tPH_Line.dDis[2]/tPH_Line.dLength;
	}

	//姿态四元数求解
	m_HS_Kinematic->EulerZYX_CalcQ(tPH_Line.dSPos,tPH_Line.dEPos,tPH_Line.dQ);

	//姿态位移
	m_HS_Kinematic->EulerZYX_CalcDis(&tPH_Line.dSPos[3],&tPH_Line.dEPos[3],&tPH_Line.dDis[3],true);

	//对附加轴进行处理，都是 坐标
	for(int i = 6;i < MaxAxisNum;i++)
	{
		tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];
	}

	double dSumDis = 0;		//总的位移长度
	double dMaxDis = 0;

	if(tPH_Line.dLength > Eps)
		tPH_Line.iMaxAxis = 0;
	else 
		tPH_Line.iMaxAxis = 1;

	tPH_Line.dSetDis[0] = tPH_Line.dLength;	
	//姿态变化四元数角度值，并将其转化为角度制
	tPH_Line.dSetDis[1] = Rad2angle(tPH_Line.dQ[0]);
	//附加轴
	for(int i = 2;i < SpaceAxisNum;i++)
	{
		tPH_Line.dSetVel[i] = m_dJVelPara[i+4];
		tPH_Line.dSetDis[i] = tPH_Line.dDis[i+4];
	}

	for(int i = 0;i < MaxAxisNum;i++)
	{
		double dDis  = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];
		dSumDis += fabs(dDis);
		if(fabs(dDis) > dMaxDis)
		{
			dMaxDis = fabs(dDis);
		}
	}

	if(dSumDis < Eps)
	{
		tPH_Line.bRepeatPos = true;
	}

	return iErrorId;
}
/************************************************
函数功能：六轴过腕部奇异功能检测
参    数：
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Line::PUMAWristQYCheck(Para_PreHandle &tPreHandle)
{
	int iErrorId = 0;

	if(fabs(tPreHandle.dSetJPos[0][4]) < BRWristQYLimit||
		fabs(tPreHandle.dSetJPos[1][4]) < BRWristQYLimit||
		tPreHandle.dSetJPos[0][4]*tPreHandle.dSetJPos[1][4] < Eps)
	{
		tPreHandle.bWristQyFlag = true;
	}

	return iErrorId;
}
/************************************************
函数功能：点位检测，判断是否超限位、奇异等
参    数：
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Line::CheckPosLimit(Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;

	iErrorId = m_HS_Kinematic->HS_JPosLimitCheck(tPH_Line.dSetJPos[1]);
    if(iErrorId != 0) 
        return E_L_TARGETOVERLIMIT;

    iErrorId = m_HS_Kinematic->HS_QYLimitCheck(tPH_Line.dSetJPos[0]);

    if(!(tPH_Line.bWristQyFlag&&iErrorId == ERROR_QY_WRIST))
    {
        if(iErrorId != 0) 
        {
            if(iErrorId == ERROR_QY_BORDER)
                return E_L_STARTQYBORDER;
            else if(iErrorId == ERROR_QY_INSIDE)
                return E_L_STARTQYINSIDE;
            else if(iErrorId == ERROR_QY_WRIST)
                return E_L_STARTQYWRIST;

            return iErrorId;
        }
    }
    else
        iErrorId = 0;

    iErrorId = m_HS_Kinematic->HS_QYLimitCheck(tPH_Line.dSetJPos[1]);
    if(!(m_bWristQYFlag&&iErrorId == ERROR_QY_WRIST))
    {
        if(iErrorId != 0) 
        {
            if(iErrorId == ERROR_QY_BORDER)
                return E_L_ENDQYBORDER;
            else if(iErrorId == ERROR_QY_INSIDE)
                return E_L_ENDQYINSIDE;
            else if(iErrorId == ERROR_QY_WRIST)
                return E_L_ENDQYWRIST;
            
            return iErrorId;
        }
    }
    else
        iErrorId = 0;
	
	return iErrorId;
}
/************************************************
函数功能： 设定参数的自适应调整，主要是对 速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：elemt------运动信息
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Line::AutoBasePara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;
	//初始参数设置
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPH_Line.dSetKVel[i] = 1.0;					
	}
	tPH_Line.dSetKAcc[0] = 1.0;
	tPH_Line.dSetKAcc[1] = 1.0;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPH_Line.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPH_Line.dSetMaxAcc = tMoveData.dDec/100;

	tPH_Line.dSetVel[0] = tMoveData.dVel;
	tPH_Line.dSetVel[1] = m_dCVelPara[1]*tMoveData.dVort/100;

	Para_PreHandle tPH = tPH_Line;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPH_Line.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPH_Line.dSetMaxAcc = tMoveData.dDec/100;

	return iErrorId;
}

/************************************************
函数功能： 设定参数的自适应调整，主要是对 速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：elemt------运动信息
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Line::AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	if (gettimeofday(&l_beginTime_Auto, NULL) == -1)
		return -1;
#endif
	if(tPH_Line.bGroupSyncFlag)
	{
		//协同从运动坐标求解需要主运动点位
	}
	else
		iErrorId = AutoAdjustJAcc(tPH_Line);
#ifdef _LINUX_
	if(gettimeofday(&l_endTime_Auto, NULL) == -1)
		return -1;
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);

	LOG_ALGO("Time_AutoAdjust = %.3lf\n",de_us_Auto);
#endif
	return iErrorId;
}
/************************************************
函数功能：基于 加速度自适应限制
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Line::AutoAdjustJAcc(Para_PreHandle &tPH_Line)
{
	Para_PreHandle tPH = tPH_Line;
	SyncPara tSyncPara;
    memset(&tSyncPara,0,sizeof(tSyncPara));
	int iErrorId = 0;
	double dMaxKJVel[3] = {0};					//运动的 速度比例系数，速度的非线性处理	
	for(int iCnt = 0;iCnt < MAXCNT;iCnt++)
	{
/************STEP1：规划参数的设定******************************/
		for(int i = 0;i < SpaceAxisNum;i++)
		{
			tPH.dSetVel[i] = tPH_Line.dSetVel[i]*tPH.dSetKVel[i];
			//按照加减速比例约束
			if(i < 2)
			{
				tPH.dSetAcc[i] = m_dCAccPara[i]*tPH.dSetMaxAcc*tPH.dSetKAcc[0];
				tPH.dSetDec[i] = m_dCAccPara[i]*tPH.dSetMaxAcc*tPH.dSetKAcc[1];
			}
			else
			{
				tPH.dSetAcc[i] = m_dJAccPara[i]*tPH.dSetMaxAcc;
				tPH.dSetDec[i] = m_dJAccPara[i]*tPH.dSetMaxAcc;
			}
		}

		tSyncPara.tPreHandle = tPH;
		SyncByAcc(tSyncPara);

		/************STEP2：加速度自适应参数调整******************************/
		double dMaxKAcc[2] = {0};
        double dMaxKVel = 0;
        double dKJVelTemp = 0;
		double dTime = tSyncPara.tVelPlan.dTime[TACC]/2;
		iErrorId = GetKAccScaleByTime(dTime,tSyncPara,dMaxKAcc[0],dKJVelTemp);
		if(iErrorId != 0) return iErrorId;
        dMaxKVel = Max(dMaxKVel,dKJVelTemp);

		dTime = tSyncPara.tVelPlan.dTime[TALL] - tSyncPara.tVelPlan.dTime[TDEC]/2;
		iErrorId = GetKAccScaleByTime(dTime,tSyncPara,dMaxKAcc[1],dKJVelTemp);
		if(iErrorId != 0) return iErrorId;
        dMaxKVel = Max(dMaxKVel,dKJVelTemp);

/************STEP3：速度自适应参数调整，约束空间速度匹配 速度******************************/	
		double dKVel[3][MaxAxisNum] = {0};
		dTime = tSyncPara.tVelPlan.dTime[TACC];
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[0]);
		if(iErrorId != 0) return iErrorId;

		dTime = tSyncPara.tVelPlan.dTime[TACC] + tSyncPara.tVelPlan.dTime[TCON]/2;
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[1]);
		if(iErrorId != 0) return iErrorId;

		dTime = tSyncPara.tVelPlan.dTime[TALL] - tSyncPara.tVelPlan.dTime[TDEC];
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[2]);
		if(iErrorId != 0) return iErrorId;

		memset(dMaxKJVel,0,sizeof(dMaxKJVel));
		for(int i = 0;i < MaxAxisNum;i++)
		{
			dMaxKJVel[0] = Max(dMaxKJVel[0],dKVel[0][i]);
			dMaxKJVel[1] = Max(dMaxKJVel[1],dKVel[1][i]);
			dMaxKJVel[2] = Max(dMaxKJVel[2],dKVel[2][i]);
		}
/************STEP4：捷度和速度比例系数调整计算******************************/
		//最大 速度比值		
		for(int i = 0;i < 3;i++)
		{
			dMaxKVel = Max(dMaxKVel,dMaxKJVel[i]);
		}

		//缓存当前的 速度比例值
		tPH_Line.dOrigKJVel = dMaxKVel;

		if(dMaxKVel > 10)   //如果 速度求解超出限制值10倍，则表示有奇异位置或者不可达
		{
			LOG_ALGO("UnReachable KMAX = %.3lf",dMaxKVel);
			return ERROR_UNREACHABLE;
		}
		//根据 执行的速度比例系数对设定速度进行缩放处理，但最终设定速度不超出原有的设定速度值
		//当前实际速度与设定速度的比例值
		double dKRealVel;
		for(int i = 0;i < 2;i++)
		{
			if(tPH.dSetVel[i] < Eps)			
				dKRealVel = 0;			
			else
				dKRealVel = fabs(tSyncPara.tVelPlan.dEVel[i])/tPH.dSetVel[i];//欧拉角360修改

			if(dMaxKVel > Eps)
				tPH.dSetKVel[i] = tPH.dSetKVel[i]*dKRealVel/dMaxKVel;	

			if(tPH.dSetKVel[i] > 1.0)
				tPH.dSetKVel[i] = 1.0;
			else if(tPH.dSetKVel[i] < 0.0001)		//避免值太小
				tPH.dSetKVel[i] = 0.0001;
		}
		
		//求解捷度放大比例参数，取较小值
		for(int i = 0;i < 2;i++)
		{
			if(dMaxKAcc[i] > Eps)
				tPH.dSetKAcc[i] = tPH.dSetKAcc[i]/dMaxKAcc[i];
		}				
	}	
    memcpy(tPH_Line.dPreHandleVel,tSyncPara.tVelPlan.dEVel,sizeof(double)*SpaceAxisNum);
	const double MAXKJACC = 50.0;
	const double MINKJACC = 0.001;
	//对最大最小比例进行限制
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPH_Line.dSetKVel[i] = tPH.dSetKVel[i];
	}
	for(int i = 0;i < 2;i++)
	{
		tPH_Line.dSetKAcc[i] = tPH.dSetKAcc[i];
		if(tPH_Line.dSetKAcc[i] < MINKJACC)
			tPH_Line.dSetKAcc[i] = MINKJACC;
		else if(tPH_Line.dSetKAcc[i] > MAXKJACC)
			tPH_Line.dSetKAcc[i] = MAXKJACC;
	}
	return 0;
}

/************************************************
函数功能：运动规划
参    数：dSDece-----减速运行时的减速度
		 pMCur------运行段信息
		 dRatio-----速度比例因子
		 dRealJPos--当前实际的位置
返 回 值：错误ID
*************************************************/
int HS_Int_Line::Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	int iErrorId = 0;

	m_bPlanFlag = false;
	iErrorId = GlobalPlan(tTrajData,iIndex,iGroupNum,dRatio,tHS_GroupJPos);	
	if(iErrorId != 0)
	{
		if(iErrorId == W_REPEAT)
		{
			GetSysPara(tTrajData,iIndex,dRatio);
			memcpy(m_dRJPos,m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
			if(!bCheckRepeatJPos(m_dRJPos,m_dRealSJPos))	
			{
				LOG_ALGO("Reapeat Pos StartPos Change!");
				iErrorId = 0;
			}
			else
				VelPlan();
		}
		if(iErrorId != 0) return iErrorId; 
	}

	iErrorId = GetSysPara(tTrajData,iIndex,dRatio);
	if(iErrorId != 0) return iErrorId;  

	iErrorId = StartPosCheckAndReplan();
	if(iErrorId != 0) return iErrorId;

	iErrorId = TrajectoryPlan();
	if(iErrorId != 0) return iErrorId; 

	VelPlan();

	m_bPlanFlag = true;
	m_bWristQYHandleFlag[m_iGroupNum] = m_tSync.tPreHandle.bWristQyFlag;
	m_dWeaveMoveBVel = m_tSync.tVelPlan.dEVel[0];

	WeaveMovePlan();

	return iErrorId;
}

/************************************************
函数功能：运动规划的起点点位检测以及重新规划
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Line::StartPosCheckAndReplan()
{
	int iErrorId = 0;

	if(CheckStopWeaveType()&&m_tGTrajData[m_iIndex].tMotionData.dCnt > Eps)
	{
		m_tGTrajData[m_iIndex].tMotionData.dCnt = 0;
		LOG_ALGO("Stop Weave Type Clear Cnt!");
	}

	memcpy(m_dRJPos,m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);

	if(m_bSmoothMoveFlag[m_iGroupNum]||m_bStopSineNextFlag)
	{
		if(m_bSmoothSynthFlag[m_iGroupNum])
		{
			memcpy(m_dRJPos,m_dRealSJPos,sizeof(double)*MaxAxisNum);
		}
		return iErrorId;
	}

	bool bRepeatFlag = bCheckRepeatJPos(m_dRealSJPos,m_tSync.tPreHandle.dSetJPos[0]);

	bool bReplanFlag = false;
	if(!bRepeatFlag)
	{
		if(m_bWristQYHandleFlag[m_iGroupNum])
		{
			LOG_ALGO("Wrist QYHandle RePlan!");
			bReplanFlag = true;
		}	
		else 
		{
			LOG_ALGO("SetJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_tSync.tPreHandle.dSetJPos[0][0],m_tSync.tPreHandle.dSetJPos[0][1],m_tSync.tPreHandle.dSetJPos[0][2],m_tSync.tPreHandle.dSetJPos[0][3],m_tSync.tPreHandle.dSetJPos[0][4],m_tSync.tPreHandle.dSetJPos[0][5],\
				m_tSync.tPreHandle.dSetJPos[0][6],m_tSync.tPreHandle.dSetJPos[0][7],m_tSync.tPreHandle.dSetJPos[0][8]);
			LOG_ALGO("RealJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_dRealSJPos[0],m_dRealSJPos[1],m_dRealSJPos[2],m_dRealSJPos[3],m_dRealSJPos[4],m_dRealSJPos[5],\
				m_dRealSJPos[6],m_dRealSJPos[7],m_dRealSJPos[8]);
			iErrorId = E_L_STARTPOS;
			LOG_ALGO("ErrorNum:%d,Line Error StartPos!",iErrorId);
			return iErrorId;
		}
	}

	if(bReplanFlag)
	{
		Para_PreHandle tPH_Line = m_tSync.tPreHandle;

		m_HS_Kinematic->HS_JPosToCPos(m_dRealSJPos,m_iToolNum,m_iWorkNum,tPH_Line.dSPos);	
		memcpy(tPH_Line.dSetJPos[0],m_dRealSJPos,sizeof(double)*MaxAxisNum);

		for(int i = 0;i < 3;i++)
			tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];

		tPH_Line.dLength = sqrt(tPH_Line.dDis[0]*tPH_Line.dDis[0] + tPH_Line.dDis[1]*tPH_Line.dDis[1] + tPH_Line.dDis[2]*tPH_Line.dDis[2]);

		if(tPH_Line.dLength  > Eps)
		{
			tPH_Line.dKXYZ[0] = tPH_Line.dDis[0]/tPH_Line.dLength;
			tPH_Line.dKXYZ[1] = tPH_Line.dDis[1]/tPH_Line.dLength;
			tPH_Line.dKXYZ[2] = tPH_Line.dDis[2]/tPH_Line.dLength;
		}

		//姿态四元数求解
		m_HS_Kinematic->EulerZYX_CalcQ(tPH_Line.dSPos,tPH_Line.dEPos,tPH_Line.dQ);

		//姿态位移
		m_HS_Kinematic->EulerZYX_CalcDis(&tPH_Line.dSPos[3],&tPH_Line.dEPos[3],&tPH_Line.dDis[3],true);

		//对附加轴进行处理，都是 坐标
		for(int i = 6;i < MaxAxisNum;i++)
		{
			tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];
		}
		
		tPH_Line.dSetDis[0] = tPH_Line.dLength;	
		tPH_Line.dSetDis[1] = Rad2angle(tPH_Line.dQ[0]);
		for(int i = 2;i < SpaceAxisNum;i++)
		{
			tPH_Line.dSetDis[i] = tPH_Line.dDis[i+4];
		}

		if(tPH_Line.dSetKVel[0] < 0.1)
			tPH_Line.dSetKVel[0] = 0.1;
		if(tPH_Line.dSetKVel[0] < 0.1)
			tPH_Line.dSetKVel[1] = 0.1;

		memcpy(tPH_Line.dOrigDis,tPH_Line.dSetDis,sizeof(double)*SpaceAxisNum);

		memcpy(m_tGTrajData[m_iIndex].iData[m_iGroupNum],&tPH_Line,sizeof(Para_PreHandle));	
		m_tSync.tPreHandle = tPH_Line;		
	}

	return iErrorId;
}
/************************************************
函数功能：暂停重启运动规划
参    数：
		 ratio-------速度比例因子
		 realjpos----当前实际的关节位置
返 回 值：错误ID
*************************************************/
int HS_Int_Line::RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll)
{
    int iErrorId = 0;

    m_dSetRatio = dRatio;    

	LOG_ALGO("Restart Plan,Id = %d,GroupNum = %d,RstState = %d!",m_iIndex,m_iGroupNum,eRstState);

	memset(m_tSyncAhead[m_iGroupNum],0,sizeof(m_tSyncAhead[m_iGroupNum]));
	m_eRstState = eRstState;
	m_dTRSTPreAll = dTAll;
    m_iLookAhead[m_iGroupNum] = 0;

    //直接缓存停止距离，重新规划
    iErrorId = TrajectoryPlan();
    if(iErrorId != 0) return iErrorId;  

    VelPlan();

	if(eRstState != RST_SMOOTHPRE)
	{
		WeaveMovePlan();
	}
	else
	{
		HS_WeavePara tWeavePara = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum].tWeavePara;

		if(tWeavePara.bWeaveEn)
		{
			m_bSmoothSynthFlag[m_iGroupNum] = true;
		}
	}

    dTAll = m_dTAll;

    return iErrorId;
}

/************************************************
函数功能：从上层给定的运动结构体数据中获取待使用的信息
参    数：pMCur---运动结构体
		 dRatio--速度比例因子
返 回 值：错误码
*************************************************/
int HS_Int_Line::GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio)
{	
	m_tGTrajData = tTrajData;
	m_iIndex = iIndex;
	m_dSetRatio = dRatio;

	COORD_SYSTEM iCoordinate = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iCoordinate;	

    //设置系统工具工件坐标系
    if(iCoordinate == BASE_COORD_SYSTEM||iCoordinate == JOINT_COORD_SYSTEM)
    {
        m_iToolNum = -1;
        m_iWorkNum = -1;
    }
    else if(iCoordinate == USER_COORD_SYSTEM||iCoordinate == TOOL_COORD_SYSTEM)
    {
        m_iToolNum = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iToolNum;
        m_iWorkNum = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iWorkNum;
    }

	if(m_tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Slave)
	{
		m_iWorkNum = -1;
	}

    m_HS_Kinematic->HS_SetCoordinate(m_iToolNum,m_iWorkNum);
	return 0;
}
/************************************************
函数功能：插补规划，根据给定的位置速度加速度等信息进行速度、轨迹的插补规划
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_Line::TrajectoryPlan(void)
{	
	m_bStopDoneFlag = false;

	memset(&m_tSync,0,sizeof(SyncPara));
    GetInputPara(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSync.tPreHandle,1.0);
    m_eState = m_tSync.tPreHandle.eState;
    SyncByAcc(m_tSync,true,true,true);

    GetInputPara(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSync.tPreHandle,m_dSetRatio);
    SyncByAcc(m_tSync,true,true);	

    //平滑规划【单段式前瞻】
    int iLookAhead = 0;
    int iErrorId = 0;
    m_bDynSmoothFlag = false;

    for(iLookAhead = 0;iLookAhead < MAXLOOKAHEAD;iLookAhead++)
    {
		int iBuffCur = (m_iIndex + iLookAhead)%MaxBuffSize;
		GroupTrajData tTrajDataCur = m_tGTrajData[iBuffCur];
		int iBuffNext = (m_iIndex + iLookAhead + 1)%MaxBuffSize;
		GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext]; 

        //已规划部分
        if(iLookAhead < m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
            continue;
        }
        else if(iLookAhead == m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
        }
        else if(iLookAhead == 0)
        {
            memset(&m_tSyncAhead[m_iGroupNum][iLookAhead],0,sizeof(SyncPara));
            GetInputParaGE(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle,m_dSetRatio);
            if(m_eRstState == RST_SMOOTHPRE)
            {
                SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
                m_tGTrajData[m_iIndex].tMotionData.dCnt = 0;
                break;
            }
            else
            {
                if(tTrajDataCur.tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iBuffNext) == 0)
				{
					if((WeaveMoveCheck(tTrajDataCur)&&!WeaveMoveCheck(tTrajDataNex))||
						(!WeaveMoveCheck(tTrajDataCur)&&WeaveMoveCheck(tTrajDataNex)))
					{
						tTrajDataCur.tMotionData.dCnt = 0;
						LOG_ALGO("Weave Move Clear Cnt!");
						SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
						break;
					}
					else
						SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true);	
				}
                else
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
            }

            if(m_eRstState == RET_SMOOTHNEX)
            {
                if(m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TACC] < m_dTRSTPreAll)
                {
                    SyncByTime(m_dTRSTPreAll,m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TDEC],m_tSyncAhead[m_iGroupNum][iLookAhead]);
                }
            }            
        }  

        if(tTrajDataCur.tMotionData.dCnt > Eps)
        {
            int iRet = CheckTrajDataAvailable(m_tGTrajData,iBuffNext);
            if(iRet == 0)
            {
				if((WeaveMoveCheck(tTrajDataCur)&&!WeaveMoveCheck(tTrajDataNex))||
					(!WeaveMoveCheck(tTrajDataCur)&&WeaveMoveCheck(tTrajDataNex)))
				{
					tTrajDataCur.tMotionData.dCnt = 0;
					LOG_ALGO("Weave Move Clear Cnt!");
					break;
				}

                //混合平滑处理
                memset(&m_tSyncAhead[m_iGroupNum][iLookAhead+1],0,sizeof(SyncPara));
                GetInputParaGE(m_tGTrajData,iBuffNext,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle,m_dSetRatio);

                int iBuffNext2 = (m_iIndex + iLookAhead + 2)%MaxBuffSize;
                if(tTrajDataNex.tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iBuffNext2) == 0)
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                else
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1],false,true);       

                if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Joint)
                    LOG_ALGO("Smooth Handle With Joint!");
                else if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Line)
                    LOG_ALGO("Smooth Handle With Line!");
                else if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Arc)
                    LOG_ALGO("Smooth Handle With Cicle!");

                iErrorId = SmoothHandle(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                if(iErrorId != 0)
                    return iErrorId;
            }
            else if(iRet == -1)
            {
                //无点位，动态平滑规划
                m_bDynSmoothFlag = true;
                LOG_ALGO("Need Dyn Smooth Plan!");
                break;
            }
            else
            {
                //有点位，但是点位异常
                break;
            }
        }
        else
        {
            break;
        }
    }

    m_iLookAhead[m_iGroupNum] = iLookAhead;

    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dAcc,m_tSync.tVelPlan.dAcc,sizeof(m_tSync.tVelPlan.dAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dDec,m_tSync.tVelPlan.dDec,sizeof(m_tSync.tVelPlan.dDec));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dJerk,m_tSync.tVelPlan.dJerk,sizeof(m_tSync.tVelPlan.dJerk));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopAcc,m_tSync.tVelPlan.dStopAcc,sizeof(m_tSync.tVelPlan.dStopAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopJerk,m_tSync.tVelPlan.dStopJerk,sizeof(m_tSync.tVelPlan.dStopJerk));
    m_tSync = m_tSyncAhead[m_iGroupNum][0];

	if(m_tSync.tVelPlan.dTSmooth[NEX] > Eps)
	{
		m_bSmoothMoveFlag[m_iGroupNum] = true;
		m_bSmoothNextFlag = true;
	}
	else 
	{
		m_bSmoothMoveFlag[m_iGroupNum] = false;
		m_bSmoothNextFlag = false;
	}

	m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
	m_dTCon = m_tSync.tVelPlan.dTime[TCON];
	m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
	m_dTAll = m_tSync.tVelPlan.dTime[TALL];
	return 0;
}
/************************************************
函数功能：获取输出信息进行数据的存储
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_Line::GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Line,double dRatio)
{
	int iErrorId = 0;

	memcpy(&tPH_Line,tTrajData[iIndex].iData[iGroupNum],sizeof(Para_PreHandle));    

	tPH_Line.dSetVel[0] = tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVel*dRatio*tPH_Line.dSetKVel[0];
	tPH_Line.dSetVel[1] = m_dCVelPara[1]*tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVort/100*dRatio*tPH_Line.dSetKVel[1];
	for(int i = 0;i < 2;i++)
	{
		//按照加减速比例约束
		tPH_Line.dSetAcc[i] = m_dCAccPara[i]*tPH_Line.dSetMaxAcc*tPH_Line.dSetKAcc[0]*dRatio;
		tPH_Line.dSetDec[i] = m_dCAccPara[i]*tPH_Line.dSetMaxAcc*tPH_Line.dSetKAcc[1]*dRatio;
        tPH_Line.dSetDis[i] = tPH_Line.dOrigDis[i] - m_dStopPos[i];
	}
	//附加轴
	for(int i = 2;i < SpaceAxisNum;i++)
	{
		tPH_Line.dSetVel[i] = m_dJVelPara[i+4]*dRatio;
		tPH_Line.dSetAcc[i] = m_dJAccPara[i+4]*dRatio*tPH_Line.dSetMaxAcc;
		tPH_Line.dSetDec[i] = m_dJAccPara[i+4]*dRatio*tPH_Line.dSetMaxAcc;
        tPH_Line.dSetDis[i] = tPH_Line.dOrigDis[i] - m_dStopPos[i];
	}

    m_dTFreProtect = m_HS_BasicPara->HS_GetTFre(tPH_Line.dSetJPos[0],tTrajData[iIndex].tMotionData.iSmooth,m_iGroupNum);
	return iErrorId;
}

/************************************************
函数功能：速度规划
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Line::VelPlan(void)
{
	//执行速度规划
	memset(m_tVelPlanPara,0,sizeof(m_tVelPlanPara));	

    FilterControl tFilterControl = m_tGTrajData[m_iIndex].tMotionData.tFilterControl;

	//暂不使用
	tFilterControl.bFilterOpenFlag = false;

	for(int i = 0;i < SpaceAxisNum;i++)
	{
        m_tVelPlanPara[i].eTypeAcc = APType_Para;
		m_tVelPlanPara[i].dTCon = m_tSync.tVelPlan.dTime[TCON];
		m_tVelPlanPara[i].dTAcc = m_tSync.tVelPlan.dTime[TACC];
		m_tVelPlanPara[i].dTDec = m_tSync.tVelPlan.dTime[TDEC];
		m_tVelPlanPara[i].dTAll = m_tSync.tVelPlan.dTime[TALL];
        m_tVelPlanPara[i].dTSmoothOff = m_dTSmoothOff[m_iGroupNum];
		m_tVelPlanPara[i].dDis = m_tSync.tPreHandle.dSetDis[i];
		m_tVelPlanPara[i].dEVel = m_tSync.tVelPlan.dEVel[i];
        if(tFilterControl.bFilterOpenFlag)
        {
            m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i],tFilterControl.tFilterPara);
        }
        else
            m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i]);	
	}

    //初始化
	//memcpy(m_dMovePos,m_tSync.tPreHandle.dSPos,sizeof(m_dMovePos));
    memset(m_dMovePos,0,sizeof(m_dMovePos));
	memset(m_dMoveVel,0,sizeof(m_dMoveVel));
	memset(m_dMoveAcc,0,sizeof(m_dMoveAcc));
	m_bStopFlag = false;

	m_dTCur = m_dCycle + m_dTSmoothOff[m_iGroupNum];
	m_dTSmoothOff[m_iGroupNum] = 0;

    // //如果是与前一段同步段平滑，则不清空时间数据
    // if(!(m_eTrackModePre == TRM_Sync&&m_bSmoothPreFlag))
    //     m_dTAddTrack = m_dTCur;
	// //打印设定参数
	// HS_geninfo *m_pMCur = m_pMCurBase +  m_iBuffP;

    // //传送带跟踪状态打印

	BaseMoveData tBaseMoveData = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum];

	LOG_ALGO("CoorperMove = %d;GroupSync = %d;WristQyFlag = %d",\
		(int)m_tSync.tPreHandle.bCoorperMoveFlag,(int)m_tSync.tPreHandle.bGroupSyncFlag,(int)m_tSync.tPreHandle.bWristQyFlag);

	LOG_ALGO("SetVel = %.0lf,%.0lf---%.0lf,%.0lf,%.0lf;SetAcc = %.0lf,%.0lf;Smooth = %d;TSmooth = %.3lf;",\
         tBaseMoveData.dVel,tBaseMoveData.dVort,m_dJVelPara[6],m_dJVelPara[7],m_dJVelPara[8],\
		 tBaseMoveData.dAcc,tBaseMoveData.dDec,m_tGTrajData[m_iIndex].tMotionData.iSmooth,m_dTFreProtect);
	LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSPos[0],m_tSync.tPreHandle.dSPos[1],m_tSync.tPreHandle.dSPos[2],m_tSync.tPreHandle.dSPos[3],m_tSync.tPreHandle.dSPos[4],m_tSync.tPreHandle.dSPos[5],\
		m_tSync.tPreHandle.dSPos[6],m_tSync.tPreHandle.dSPos[7],m_tSync.tPreHandle.dSPos[8]);
	LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dEPos[0],m_tSync.tPreHandle.dEPos[1],m_tSync.tPreHandle.dEPos[2],m_tSync.tPreHandle.dEPos[3],m_tSync.tPreHandle.dEPos[4],m_tSync.tPreHandle.dEPos[5],\
		m_tSync.tPreHandle.dEPos[6],m_tSync.tPreHandle.dEPos[7],m_tSync.tPreHandle.dEPos[8]);
	LOG_ALGO("Dis  = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dDis[0],m_tSync.tPreHandle.dDis[1],m_tSync.tPreHandle.dDis[2],m_tSync.tPreHandle.dDis[3],m_tSync.tPreHandle.dDis[4],m_tSync.tPreHandle.dDis[5],\
		m_tSync.tPreHandle.dDis[6],m_tSync.tPreHandle.dDis[7],m_tSync.tPreHandle.dDis[8]);
	LOG_ALGO("Length= %.6lf,Angle = %.3lf;LSmooth = %.3lf,%.3lf",\
        m_tSync.tPreHandle.dOrigDis[0],m_tSync.tPreHandle.dOrigDis[1],m_tSync.tPreHandle.dDisSmooth[PRE],m_tSync.tPreHandle.dDisSmooth[NEX]);
    LOG_ALGO("StopPos = %.3lf, %.3lf---%.3lf,%.3lf,%.3lf",\
        m_dStopPos[0],m_dStopPos[1],m_dStopPos[1],m_dStopPos[2],m_dStopPos[3]);
	LOG_ALGO("Ratio = %.2lf;KJVel = %.3lf;Cnt = %.3lf",m_dSetRatio,m_tSync.tPreHandle.dRealKJVel,m_tGTrajData[m_iIndex].tMotionData.dCnt);
    LOG_ALGO("Firter :Open = %d,Type = %d,Fre = %.3lf,Grade = %d",\
        tFilterControl.bFilterOpenFlag,tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade);

	//打印规划参数
	LOG_ALGO("RealVel = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tVelPlan.dEVel[0],m_tSync.tVelPlan.dEVel[1],m_tSync.tVelPlan.dEVel[2],m_tSync.tVelPlan.dEVel[3],m_tSync.tVelPlan.dEVel[4]);
	LOG_ALGO("SetKVel = %.3lf,%.3lf--%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSetKVel[0],m_tSync.tPreHandle.dSetKVel[1],m_tSync.tPreHandle.dSetKVel[2],m_tSync.tPreHandle.dSetKVel[3],m_tSync.tPreHandle.dSetKVel[4]);	
    LOG_ALGO("Acc = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf;KAcc = %.3lf,%.3lf",\
        m_tSync.tVelPlan.dAcc[0],m_tSync.tVelPlan.dAcc[1],m_tSync.tVelPlan.dAcc[2],m_tSync.tVelPlan.dAcc[3],m_tSync.tVelPlan.dAcc[4],
		m_tSync.tPreHandle.dSetKAcc[0],m_tSync.tPreHandle.dSetKAcc[1]);
    LOG_ALGO("Jerk= %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dJerk[0],m_tSync.tVelPlan.dJerk[1],m_tSync.tVelPlan.dJerk[2],m_tSync.tVelPlan.dJerk[3],m_tSync.tVelPlan.dJerk[4]);	
    LOG_ALGO("TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
        m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);

    
    return 0;
}

/************************************************
函数功能：计算速度，生成曲线【执行一次运动插补】
参    数：
        iErrorId------报警码
        bLastCycle----是否是多点插补的最后一个周期
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Line::Move(int &iErrorId,bool bLastCycle)
{
	return IntMove(iErrorId,bLastCycle);				
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Line::execIntMove(IntData &intdata,int &iErrorId)
{
    if(m_bDynSmoothFlag)
    {
        DynSmoothPlan();
    }

	HS_MStatus eMStatus = UpdateIntMove(intdata,iErrorId);	
	return eMStatus;
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_Line::Stop()
{
	return StopPlan();
}
/************************************************
函数功能：运动调速
参    数：设定的速度倍率
返 回 值：错误码
*************************************************/
int HS_Int_Line::setRatio(double dRatio)
{
    if(m_bStopFlag)	//执行减速停止，响应调速
        return 0;

	return Ratio(dRatio);
}