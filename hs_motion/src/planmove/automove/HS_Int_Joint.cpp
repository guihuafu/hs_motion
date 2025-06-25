
/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Joint.cpp
* 摘    要：关节运动同步插补算法

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HS_Int_Joint.h"
/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Int_Joint::HS_Int_Joint(HS_GroupKin *pGroupKin)
{
	m_HS_GroupKin = pGroupKin;
	m_eMoveType = MP_Joint;
}

HS_Int_Joint::~HS_Int_Joint()
{

}
/************************************************
函数功能：参数复位
参   数：    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::ResetData()
{
	return 0;
}
/************************************************
函数功能：执行关节运动的预处理过程
参   数：elemt----运动输入信息
		trajout---规划缓存信息    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum)
{
	int iErrorId = 0;

	BaseMoveData *tMoveData = &tGroupMotionData.tBaseMoveData[iGroupNum];   

	if(tMoveData->dVel > 100)
		tMoveData->dVel = 100;

	Para_PreHandle tPH_Joint;

	GlobalPreHandle(tGroupMotionData,tTrajData,iIndex,iGroupNum,tPH_Joint,tMoveData);

	while(1)
	{
		iErrorId = GetStartPos(*tMoveData,tPH_Joint,tGroupMotionData.bStartMove);
		if(iErrorId != 0) break;

		iErrorId = GetEndPos(*tMoveData,tPH_Joint);
		if(iErrorId != 0) break;

		iErrorId = GetMoveDis(tPH_Joint);
		if(iErrorId != 0) break;

		iErrorId = CheckEndPosLimit(tPH_Joint);
		if(iErrorId != 0) 
		{
			LOG_ALGO("Check Pos Limit ErrorNum = %d",iErrorId);
			break;
		}

		if(!tPH_Joint.bRepeatPos)
		{
			iErrorId = AutoAdjustPara(*tMoveData,tPH_Joint);
			if(iErrorId != 0) break;
		}
		break;
	}

	//存储预处理数据
	tPH_Joint.iPreHandledError = iErrorId;
	memcpy(tTrajData[iIndex].iData[iGroupNum],&tPH_Joint,sizeof(Para_PreHandle));	

	return iErrorId;
}
/************************************************
函数功能：获取关节运动起点的坐标
参   数：tMoveData----运动输入信息
		tPH_Joint---预处理输出信息	   
		bStartMoveFlag----启动行标识
返 回 值：错误码
*************************************************/
int HS_Int_Joint::GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint,bool bStartMoveFlag)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sStartPos.hs_coordinate;
	if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
        double dSJPos[MaxAxisNum] = {0};
        memcpy(dSJPos,tMoveData.sStartPos.dPos,sizeof(double)*MaxAxisNum);
		memcpy(tPH_Joint.dSPos,tMoveData.sStartPos.dPos,sizeof(double)*MaxAxisNum);	
        LOG_ALGO("Set Start JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;bStartMove = %d",\
            dSJPos[0],dSJPos[1],dSJPos[2],dSJPos[3],dSJPos[4],dSJPos[5],dSJPos[6],dSJPos[7],dSJPos[8],(int)bStartMoveFlag);
	}
	else
	{	
		double dSCPos[MaxAxisNum] = {0};
		memcpy(dSCPos,tMoveData.sStartPos.dPos,sizeof(double)*MaxAxisNum);
        
        m_eStartState = tMoveData.sStartPos.iPose&0x0F;

        LOG_ALGO("Set Start CPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;Tool = %d,Work = %d",\
            dSCPos[0],dSCPos[1],dSCPos[2],dSCPos[3],dSCPos[4],dSCPos[5],dSCPos[6],dSCPos[7],dSCPos[8],sCoord.iToolNum,sCoord.iWorkNum);

		iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(dSCPos,sCoord.iToolNum,sCoord.iWorkNum,m_eStartState,tPH_Joint.dSPos);
		if(iErrorId != 0)
		{
			LOG_ALGO("Error StartPos C2J!");
			//m_HS_Kinematic->HS_PrintCoord(sCoord.iToolNum,sCoord.iWorkNum);
			return ERROR_CPOSTOJPOS;
		}
	}

    //前后段坐标匹配
    if(!(bStartMoveFlag&&sCoord.iCoordinate == JOINT_COORD_SYSTEM))
    {
        //获取前一段的数据
        double dEndJPos[MaxAxisNum] = {0};
        bool bRet = GetPreMovePara(m_tGTrajData,m_iIndex,dEndJPos);
        if(bRet)
        {
            memcpy(tPH_Joint.dSPos,dEndJPos,sizeof(double)*MaxAxisNum);
        }
        else
        {
            LOG_ALGO("Error NoStartMove Can't Get Pre Data!");
        }
    }

    memcpy(tPH_Joint.dSetJPos[0],tPH_Joint.dSPos,sizeof(double)*MaxAxisNum);

    m_eStartState = m_HS_Kinematic->HS_JPosToAState(tPH_Joint.dSetJPos[0]);

	//协同点位
	m_HS_Kinematic->HS_JPosToCPos(tPH_Joint.dSetJPos[0],m_iToolNum,-1,tPH_Joint.dWTSPos);
	return iErrorId;
}

/************************************************
函数功能：获取关节运动终点的坐标
参   数：tMoveData----运动输入信息
		tPH_Joint---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sEndPos.hs_coordinate;
	if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
        double dEJPos[MaxAxisNum] = {0};
        memcpy(dEJPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		memcpy(tPH_Joint.dEPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);	
        LOG_ALGO("End JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
            dEJPos[0],dEJPos[1],dEJPos[2],dEJPos[3],dEJPos[4],dEJPos[5],dEJPos[6],dEJPos[7],dEJPos[8]);
	}
	else
	{
		double dECPos[MaxAxisNum] = {0};
		memcpy(dECPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);

        m_eState = tMoveData.sEndPos.iPose&0x0F;
        LOG_ALGO("End CPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;Tool = %d,Work = %d;eState = %d",\
            dECPos[0],dECPos[1],dECPos[2],dECPos[3],dECPos[4],dECPos[5],dECPos[6],dECPos[7],dECPos[8],sCoord.iToolNum,sCoord.iWorkNum,(int)m_eState);

		
		iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(dECPos,sCoord.iToolNum,sCoord.iWorkNum,m_eState,tPH_Joint.dSPos,tPH_Joint.dEPos);
		if(iErrorId != 0)
		{
			if(m_eHS_RobotType == HSROB_SCARA&&iErrorId == E_C2J_ATTITUDE)
			{
				LOG_ALGO("C2J Error! Scara Attitude UnReachable!");
				return E_J_ATTUNREABLE_Scara;
			}
			
            LOG_ALGO("C2J Error! UnReachable!");
			return E_J_TARGETUNREABLE;
		}

        if(!(m_eHS_RobotType == HSROB_SCARA&&m_HS_Kinematic->GetA360Flag()))
        {
            LOG_ALGO("Nearest JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;",\
                tPH_Joint.dEPos[0],tPH_Joint.dEPos[1],tPH_Joint.dEPos[2],tPH_Joint.dEPos[3],tPH_Joint.dEPos[4],tPH_Joint.dEPos[5],tPH_Joint.dEPos[6],tPH_Joint.dEPos[7],tPH_Joint.dEPos[8]);

            //类直线运动的末端轴选择
            NearestAngleMoveHandle(tPH_Joint,tMoveData.sEndPos);

            //Turn和Alarm处理
            TurnAlarmHandle(tMoveData.tRevolve,tPH_Joint.dSPos,tPH_Joint.dEPos);
        }
	}

    memcpy(tPH_Joint.dSetJPos[1],tPH_Joint.dEPos,sizeof(double)*MaxAxisNum);

	//协同点位
	m_HS_Kinematic->HS_JPosToCPos(tPH_Joint.dSetJPos[1],m_iToolNum,-1,tPH_Joint.dWTEPos);
	return iErrorId;
}

/************************************************
函数功能：最小姿态角度变化【类直线运动】的多转轴关节角度选择处理
参   数：	    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::NearestAngleMoveHandle(Para_PreHandle &tPH_Joint,PosInfo sEndPos)
{
    //如果起点形态与目标点形态不一致，则按照最短路径处理【无法进行直线运动】
    if((m_eStartState&0x0F) != (m_eState&0x0F))
    {
        LOG_ALGO("State Not Match! StartState = %d,TargetState = %d",(int)m_eStartState,(int)m_eState);
        return 0;
    }

    //如果6轴机器人起点的5轴坐标很小，或者4轴机器人的2轴坐标很小【奇异位置】，则不进行判断处理
    int iQYAxis = 0;
    if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA)
        iQYAxis = 4;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA_5)
        iQYAxis = 3;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_SCARA||m_HS_Kinematic->GetRobotType() == HSROB_MD410)
        iQYAxis = 1;

    const double QYLimit = 5.0;

    if(fabs(tPH_Joint.dSetJPos[0][iQYAxis]) < QYLimit)
    {
        LOG_ALGO("Start JPos With QYLimit Axis[%d] = %.3lf",iQYAxis,tPH_Joint.dSetJPos[0][iQYAxis]);
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    double dSCPos[MaxAxisNum] = {0};
    double dECPos[MaxAxisNum]={0};
    memcpy(dSCPos,tPH_Joint.dSetJPos[0],sizeof(double)*MaxAxisNum);

    HS_Coordinate sCoord = sEndPos.hs_coordinate;
	m_HS_Kinematic->HS_JPosToCPos(tPH_Joint.dSetJPos[0],sCoord.iToolNum,sCoord.iWorkNum,dSCPos);

    double dDis[MaxAxisNum] = {0};
    for(int i = 0;i < MaxAxisNum;i++)
	    dDis[i] = sEndPos.dPos[i] - dSCPos[i];		

    m_HS_Kinematic->EulerZYX_CalcDis(&dSCPos[3],&sEndPos.dPos[3],&dDis[3],true);

    LOG_ALGO("CDis=%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",dDis[0],dDis[1],dDis[2],dDis[3],dDis[4],dDis[5]);
    /////////////////////////////////////////////
    //更改迭代：直线长度过大离散点位数小会导致误判终点角度
    int MaxNum = 20,iError = 0;
    double dLJPos[MaxAxisNum] = {0};
    double dAddCPos[MaxAxisNum] = {0};
    memcpy(dLJPos,tPH_Joint.dSetJPos[0],sizeof(double)*MaxAxisNum); 

    for(int i = 1;i <= MaxNum;i++)
    {
        for(int j = 0;j < MaxAxisNum;j++)
        {
            dAddCPos[j] = dSCPos[j] + dDis[j]*i/MaxNum;
        }

		iError = m_HS_Kinematic->HS_CPosToJPos_LJ(dAddCPos,sCoord.iToolNum,sCoord.iWorkNum,CP_ToolWork,dLJPos,dLJPos);
		if(iError != 0) 
        {
            LOG_ALGO("ErrorC2J AddCPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",dAddCPos[0],dAddCPos[1],dAddCPos[2],dAddCPos[3],dAddCPos[4],dAddCPos[5]);
            return 0;
        }
    }						

	//直线运动方向判断
    int iAxisId = 0;
    const double MAXANGLE = 10.0;      //超出一定的角度变化时才进行处理//0302，阀值改小，按照直线方式运动
    if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA)
        iAxisId = 5;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA_5)
        iAxisId = 4;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_SCARA||m_HS_Kinematic->GetRobotType() == HSROB_MD410)
        iAxisId = 3;

	bool bDir = false;
	if(dLJPos[iAxisId] - tPH_Joint.dSetJPos[0][iAxisId] > Eps) //直线运动正方向
		bDir = true;

	if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA||m_HS_Kinematic->GetRobotType() == HSROB_PUMA_5
		||m_HS_Kinematic->GetRobotType() == HSROB_SCARA||m_HS_Kinematic->GetRobotType() == HSROB_MD410)
	{
		if(bDir) 
		{
			//位移超出一定的值才进行修正
			if(tPH_Joint.dEPos[iAxisId] - tPH_Joint.dSPos[iAxisId] < -Eps&&fabs(tPH_Joint.dEPos[iAxisId] - tPH_Joint.dSPos[iAxisId]) > MAXANGLE)
			{
				tPH_Joint.dEPos[iAxisId] += 360;
				LOG_ALGO("ReCalc JPos[%d] Forward= %.3lf",iAxisId,tPH_Joint.dEPos[iAxisId]);
			}
		}
		else 
		{
			if(tPH_Joint.dEPos[iAxisId] - tPH_Joint.dSPos[iAxisId] > Eps&&fabs(tPH_Joint.dEPos[iAxisId] - tPH_Joint.dSPos[iAxisId]) > MAXANGLE)
			{
				//修正偏差
				tPH_Joint.dEPos[iAxisId] -= 360;
				LOG_ALGO(" ReCalc JPos[%d] Backward= %.3lf",iAxisId,tPH_Joint.dEPos[iAxisId]);
			}
		}
    }

    return 0;
}

/************************************************
函数功能：Turn和Alarm对多转轴角度的选解
参   数：	    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::TurnAlarmHandle(HS_Revolve tRevolve,double dSetJPos[6],double dEndJPos[6])
{
    int iAxisId = 0;
    if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA)
        iAxisId = 5;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA_5)
        iAxisId = 4;
    else if(m_HS_Kinematic->GetRobotType() == HSROB_SCARA||m_HS_Kinematic->GetRobotType() == HSROB_MD410)
        iAxisId = 3;
    else 
        return 0;

    bool bDir = false;
    if(dEndJPos[iAxisId] - dSetJPos[iAxisId] > Eps) //直线运动正方向
        bDir = true;

    LOG_ALGO("TURN = %d",tRevolve.iTurn);

    //turn = 1，最短路径处理
    if(tRevolve.iTurn == 1)
    {
		m_HS_Kinematic->HS_NearestPoint(dEndJPos[iAxisId],dSetJPos[iAxisId],-1);
    }
    else if(tRevolve.iTurn == 2)
    {
        //trun=2，保证运行角度范围在+-180度
        if(dEndJPos[iAxisId] > 180)
        {
            dEndJPos[iAxisId] -= 360;
            while(dEndJPos[iAxisId] > 180)
                dEndJPos[iAxisId] -= 360;
            LOG_ALGO("ReCalc JPos Limit>180:= %.3lf",dEndJPos[iAxisId]);
        }
        if(dEndJPos[iAxisId] < -180)
        {
            dEndJPos[iAxisId] += 360;
            while(dEndJPos[iAxisId] < -180)
                dEndJPos[iAxisId] += 360;
            LOG_ALGO("ReCalc JPos Limit<-180:= %.3lf",dEndJPos[iAxisId]);
        }
    }
    else if(tRevolve.iTurn == 3)
    {
        //保证正转
        if(!bDir)
        {
            dEndJPos[iAxisId] += 360;
            LOG_ALGO("ReCalc JPos = %.3lf",dEndJPos[iAxisId]);
        }
    }
    else if(tRevolve.iTurn == 4)
    {
        //保证反转
        if(bDir)
        {
            dEndJPos[iAxisId] -= 360;
            LOG_ALGO("ReCalc JPos = %.3lf:",dEndJPos[iAxisId]);
        }
    }
    return 0;
}
/************************************************
函数功能：获取关节运动位移量等信息
参   数：tPH_Joint---预处理输入输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Joint::GetMoveDis(Para_PreHandle &tPH_Joint)
{
	int iErrorId = 0;
	double dSumDis = 0;		//总的位移长度
	double dMaxDis = 0;

	//起点末端点都是关节坐标
	for(int i = 0;i < MaxAxisNum;i++)
	{
		tPH_Joint.dDis[i]  = tPH_Joint.dEPos[i] - tPH_Joint.dSPos[i];
		dSumDis += fabs(tPH_Joint.dDis[i]);
		if(fabs(tPH_Joint.dDis[i]) > dMaxDis)
		{
			dMaxDis = fabs(tPH_Joint.dDis[i]);
			tPH_Joint.iMaxAxis = i;
		}
	}
	if(dSumDis < Eps)
	{
		tPH_Joint.bRepeatPos = true;
        LOG_ALGO("Repeat Pos!!!");
	}

    LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Joint.dSPos[0],tPH_Joint.dSPos[1],tPH_Joint.dSPos[2],tPH_Joint.dSPos[3],tPH_Joint.dSPos[4],tPH_Joint.dSPos[5],
        tPH_Joint.dSPos[6],tPH_Joint.dSPos[7],tPH_Joint.dSPos[8]);
    LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
        tPH_Joint.dEPos[0],tPH_Joint.dEPos[1],tPH_Joint.dEPos[2],tPH_Joint.dEPos[3],tPH_Joint.dEPos[4],tPH_Joint.dEPos[5],
        tPH_Joint.dEPos[6],tPH_Joint.dEPos[7],tPH_Joint.dEPos[8]);

	return iErrorId;
}
/************************************************
函数功能：关节设定参数的自适应调整，主要是对关节速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：elemt------运动信息
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Joint::CheckEndPosLimit(Para_PreHandle &tPH_Joint)
{
	int iErrorId = 0;

	iErrorId = m_HS_Kinematic->HS_JPosLimitCheck(tPH_Joint.dEPos);

    if(iErrorId != 0)
        iErrorId = E_J_TARGETOVERLIMIT;
	
	return iErrorId;
}
/************************************************
函数功能：关节设定参数的自适应调整，主要是对关节速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：tMoveData------运动信息
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Joint::AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint)
{
  	//初始参数设置
    tPH_Joint.dSetKAcc[0] = 1.0;
    tPH_Joint.dSetKAcc[1] = 1.0;
    
	if(tMoveData.dAcc < tMoveData.dDec)
		tPH_Joint.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPH_Joint.dSetMaxAcc = tMoveData.dDec/100;

	SyncPara tSyncPara; 
    memset(&tSyncPara,0,sizeof(tSyncPara));
	//普通加减速比例约束模式，仅考虑规划后的速度
	for(int i = 0;i < MaxAxisNum;i++)
	{
		tPH_Joint.dSetVel[i] = m_dJVelPara[i]*tMoveData.dVel/100;
		tPH_Joint.dSetAcc[i] = m_dJAccPara[i]*tPH_Joint.dSetMaxAcc*tPH_Joint.dSetKAcc[0];		
		tPH_Joint.dSetDec[i] = m_dJAccPara[i]*tPH_Joint.dSetMaxAcc*tPH_Joint.dSetKAcc[1];
		tPH_Joint.dSetDis[i] = tPH_Joint.dDis[i];
	}
    
	tSyncPara.tPreHandle = tPH_Joint;
	SyncByAcc(tSyncPara);

    //LOG_ALGO("RealVel = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
    //    tSyncPara.tVelPlan.dEVel[0],tSyncPara.tVelPlan.dEVel[1],tSyncPara.tVelPlan.dEVel[2],tSyncPara.tVelPlan.dEVel[3],tSyncPara.tVelPlan.dEVel[4],tSyncPara.tVelPlan.dEVel[5],
    //    tSyncPara.tVelPlan.dEVel[6],tSyncPara.tVelPlan.dEVel[7],tSyncPara.tVelPlan.dEVel[8]);

    tPH_Joint.dRealKJVel = 0;
	for(int i = 0;i < MaxAxisNum;i++)
	{
		if(m_dJVelPara[i] > Eps)
			tPH_Joint.dSetKVel[i] = fabs(tSyncPara.tVelPlan.dEVel[i]/m_dJVelPara[i]);
        tPH_Joint.dRealKJVel = Max(tPH_Joint.dSetKVel[i],tPH_Joint.dRealKJVel);
	}
	return 0;
}
/************************************************
函数功能：运动规划
参    数：trajout-----运行段信息
		 index-------索引号
		 ratio-------速度比例因子
		 realjpos----当前实际的关节位置
返 回 值：错误ID
*************************************************/
int HS_Int_Joint::Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	int iErrorId = 0;

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

	WeaveClose();

	m_dPercent = 0;
	m_bWristQYHandleFlag[m_iGroupNum] = false;
	return iErrorId;
}
/************************************************
函数功能：运动规划的起点点位检测以及重新规划
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Joint::StartPosCheckAndReplan()
{
	int iErrorId = 0;

	if(m_bSmoothMoveFlag[m_iGroupNum])
		return iErrorId;

	bool bRepeatFlag = bCheckRepeatJPos(m_dRealSJPos,m_tSync.tPreHandle.dSetJPos[0]);

	bool bReplanFlag = false;
	if(!bRepeatFlag)
	{
		if(m_bWristQYHandleFlag[m_iGroupNum])
		{
			LOG_ALGO("Wrist QYHandle RePlan!");
			bReplanFlag = true;
			m_bRepeatPosFlag = false;
		}		
		else 
		{
			LOG_ALGO("SetJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_tSync.tPreHandle.dSetJPos[0][0],m_tSync.tPreHandle.dSetJPos[0][1],m_tSync.tPreHandle.dSetJPos[0][2],m_tSync.tPreHandle.dSetJPos[0][3],m_tSync.tPreHandle.dSetJPos[0][4],m_tSync.tPreHandle.dSetJPos[0][5],\
				m_tSync.tPreHandle.dSetJPos[0][6],m_tSync.tPreHandle.dSetJPos[0][7],m_tSync.tPreHandle.dSetJPos[0][8]);
			LOG_ALGO("RealJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_dRealSJPos[0],m_dRealSJPos[1],m_dRealSJPos[2],m_dRealSJPos[3],m_dRealSJPos[4],m_dRealSJPos[5],\
				m_dRealSJPos[6],m_dRealSJPos[7],m_dRealSJPos[8]);
			iErrorId = E_J_STARTPOS;
			LOG_ALGO("ErrorNum:%d,Joint Error StartPos!",iErrorId);
			return iErrorId;
		}
	}

	if(bReplanFlag)
	{
		Para_PreHandle tPH_Joint = m_tSync.tPreHandle;

		memcpy(tPH_Joint.dSPos,m_dRealSJPos,sizeof(double)*MaxAxisNum);

		for(int i = 0;i < MaxAxisNum;i++)
		{
			tPH_Joint.dDis[i]  = tPH_Joint.dEPos[i] - tPH_Joint.dSPos[i];
		}

		AutoAdjustPara(m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum],tPH_Joint);

		memcpy(m_tGTrajData[m_iIndex].iData[m_iGroupNum],&tPH_Joint,sizeof(Para_PreHandle));	
		m_tSync.tPreHandle = tPH_Joint;		
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
int HS_Int_Joint::RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll)
{
    int iErrorId = 0;

    m_dSetRatio = dRatio;
    
    Para_PreHandle tPH_Joint = m_tSync.tPreHandle;
    //方案1：重设起点
    /*
    memcpy(tPH_Joint.dSPos,m_dStopDoneJPos,sizeof(double)*MaxAxisNum);	

    iErrorId = GetMoveDis(tPH_Joint);

    if(!tPH_Joint.bRepeatPos)
    {
        iErrorId = AutoAdjustPara(m_tTrajData[m_iIndex].elemt,tPH_Joint);
    }
    */
    LOG_ALGO("Restart Plan,Id = %d,GroupNum = %d,RstState = %d!",m_iIndex,m_iGroupNum,eRstState);

    memset(m_tSyncAhead[m_iGroupNum],0,sizeof(m_tSyncAhead[m_iGroupNum]));
    m_eRstState = eRstState;
    m_dTRSTPreAll = dTAll;
    m_iLookAhead[m_iGroupNum] = 0;

    //直接缓存停止距离，重新规划
    iErrorId = TrajectoryPlan();
    if(iErrorId != 0) return iErrorId;  

    VelPlan();

    dTAll = m_dTAll;

    return iErrorId;
}

/************************************************
函数功能：从上层给定的运动结构体数据中获取待使用的信息
参    数：tTrajData---运动结构体
         index-----索引号
		 dRatio--速度比例因子
返 回 值：错误码
*************************************************/
int HS_Int_Joint::GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio)
{	
	m_tGTrajData = tTrajData;
	m_iIndex = iIndex;
	m_dSetRatio = dRatio;

	COORD_SYSTEM iCoordinate = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iCoordinate;	

	if(iCoordinate == BASE_COORD_SYSTEM||iCoordinate == JOINT_COORD_SYSTEM)
	{
		m_iToolNum = -1;
	}
	else if(iCoordinate == USER_COORD_SYSTEM||iCoordinate == TOOL_COORD_SYSTEM)
	{
		m_iToolNum = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iToolNum;
	}
	return 0;
}
/************************************************
函数功能：插补规划，根据给定的位置速度加速度等信息进行速度、轨迹的插补规划
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_Joint::TrajectoryPlan(void)
{						
	m_bStopDoneFlag = false;

	memset(&m_tSync,0,sizeof(SyncPara));
    GetInputPara(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSync.tPreHandle,1.0);
    SyncByAcc(m_tSync,true,true,true);

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
					if(WeaveMoveCheck(tTrajDataNex))
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
				if(WeaveMoveCheck(tTrajDataNex))
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
int HS_Int_Joint::GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Joint,double dRatio)
{
	int iErrorId = 0;

	memcpy(&tPH_Joint,tTrajData[iIndex].iData[iGroupNum],sizeof(Para_PreHandle));    

	for(int i = 0;i < MaxAxisNum;i++)
	{
		tPH_Joint.dSetVel[i] = m_dJVelPara[i]*dRatio*tPH_Joint.dSetKVel[i];
		tPH_Joint.dSetAcc[i] = m_dJAccPara[i]*dRatio*tPH_Joint.dSetMaxAcc*tPH_Joint.dSetKAcc[0];		
		tPH_Joint.dSetDec[i] = m_dJAccPara[i]*dRatio*tPH_Joint.dSetMaxAcc*tPH_Joint.dSetKAcc[1];
		tPH_Joint.dSetDis[i] = tPH_Joint.dDis[i] - m_dStopPos[i];
	}

    m_dTFreProtect = m_HS_BasicPara->HS_GetTFre(tPH_Joint.dSetJPos[0],tTrajData[iIndex].tMotionData.iSmooth,m_iGroupNum);
	return iErrorId;
}

/************************************************
函数功能：速度规划
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Joint::VelPlan(void)
{
	//执行速度规划
	memset(m_tVelPlanPara,0,sizeof(m_tVelPlanPara));	

    FilterControl tFilterControl = m_tGTrajData[m_iIndex].tMotionData.tFilterControl;

	//暂不使用
	tFilterControl.bFilterOpenFlag = false;

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

	BaseMoveData tBaseMoveData = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum];

	LOG_ALGO("SetVel = %.3lf;SetAcc = %.0lf,%.0lf;Smooth = %d;TSmooth = %.3lf;",\
         tBaseMoveData.dVel,tBaseMoveData.dAcc,tBaseMoveData.dDec,
		 m_tGTrajData[m_iIndex].tMotionData.iSmooth,m_dTFreProtect);
	LOG_ALGO("Set JVel = %.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf---%.0lf,%.0lf,%.0lf",\
		m_dJVelPara[0],m_dJVelPara[1],m_dJVelPara[2],m_dJVelPara[3],m_dJVelPara[4],m_dJVelPara[5],
		m_dJVelPara[6],m_dJVelPara[7],m_dJVelPara[8]);
	LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSPos[0],m_tSync.tPreHandle.dSPos[1],m_tSync.tPreHandle.dSPos[2],m_tSync.tPreHandle.dSPos[3],m_tSync.tPreHandle.dSPos[4],m_tSync.tPreHandle.dSPos[5],\
		m_tSync.tPreHandle.dSPos[6],m_tSync.tPreHandle.dSPos[7],m_tSync.tPreHandle.dSPos[8]);
	LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dEPos[0],m_tSync.tPreHandle.dEPos[1],m_tSync.tPreHandle.dEPos[2],m_tSync.tPreHandle.dEPos[3],m_tSync.tPreHandle.dEPos[4],m_tSync.tPreHandle.dEPos[5],\
		m_tSync.tPreHandle.dEPos[6],m_tSync.tPreHandle.dEPos[7],m_tSync.tPreHandle.dEPos[8]);
	LOG_ALGO("Dis  = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dDis[0],m_tSync.tPreHandle.dDis[1],m_tSync.tPreHandle.dDis[2],m_tSync.tPreHandle.dDis[3],m_tSync.tPreHandle.dDis[4],m_tSync.tPreHandle.dDis[5],\
		m_tSync.tPreHandle.dDis[6],m_tSync.tPreHandle.dDis[7],m_tSync.tPreHandle.dDis[8]);
    LOG_ALGO("StopPos  = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_dStopPos[0],m_dStopPos[1],m_dStopPos[2],m_dStopPos[3],m_dStopPos[4],m_dStopPos[5],\
        m_dStopPos[6],m_dStopPos[7],m_dStopPos[8]);
	LOG_ALGO("Ratio = %.2lf;KJVel = %.3lf;Cnt = %.3lf",m_dSetRatio,m_tSync.tPreHandle.dRealKJVel,m_tGTrajData[m_iIndex].tMotionData.dCnt);
    LOG_ALGO("Firter :Open = %d,Type = %d,Fre = %.3lf,Grade = %d",\
        tFilterControl.bFilterOpenFlag,tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade);

	//打印规划参数
	LOG_ALGO("RealVel = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tVelPlan.dEVel[0],m_tSync.tVelPlan.dEVel[1],m_tSync.tVelPlan.dEVel[2],m_tSync.tVelPlan.dEVel[3],m_tSync.tVelPlan.dEVel[4],m_tSync.tVelPlan.dEVel[5],\
		m_tSync.tVelPlan.dEVel[6],m_tSync.tVelPlan.dEVel[7],m_tSync.tVelPlan.dEVel[8]);
	LOG_ALGO("SetKVel = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSetKVel[0],m_tSync.tPreHandle.dSetKVel[1],m_tSync.tPreHandle.dSetKVel[2],m_tSync.tPreHandle.dSetKVel[3],m_tSync.tPreHandle.dSetKVel[4],m_tSync.tPreHandle.dSetKVel[5],\
		m_tSync.tPreHandle.dSetKVel[6],m_tSync.tPreHandle.dSetKVel[7],m_tSync.tPreHandle.dSetKVel[8]);	
    LOG_ALGO("Acc = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dAcc[0],m_tSync.tVelPlan.dAcc[1],m_tSync.tVelPlan.dAcc[2],m_tSync.tVelPlan.dAcc[3],m_tSync.tVelPlan.dAcc[4],m_tSync.tVelPlan.dAcc[5],\
        m_tSync.tVelPlan.dAcc[6],m_tSync.tVelPlan.dAcc[7],m_tSync.tVelPlan.dAcc[8]);
    LOG_ALGO("Jerk= %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dJerk[0],m_tSync.tVelPlan.dJerk[1],m_tSync.tVelPlan.dJerk[2],m_tSync.tVelPlan.dJerk[3],m_tSync.tVelPlan.dJerk[4],m_tSync.tVelPlan.dJerk[5],\
        m_tSync.tVelPlan.dJerk[6],m_tSync.tVelPlan.dJerk[7],m_tSync.tVelPlan.dJerk[8]);	
	LOG_ALGO("TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
		m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);
    
    return 0;
}

/************************************************
函数功能：计算速度，生成曲线【执行一次运动插补】
参    数：bLastCycle----是否是多点插补的最后一个周期
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Joint::Move(int &iErrorId,bool bLastCycle)
{
	return IntMove(iErrorId,bLastCycle);			
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Joint::execIntMove(IntData &intdata,int &iErrorId)
{
	HS_MStatus eMStatus = M_Busy;

    if(m_bDynSmoothFlag)
    {
        DynSmoothPlan();
    }

    if(m_bRepeatPosFlag)
    {
        for(int i = 0;i < m_iInterMultCnt;i++)
        {
		    memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
        }
        intdata.dPercent = 100;
        return M_Done;
    }

	//LOG_ALGO("------------------------------------------------------------------------");
	for(int i = 0;i < m_iInterMultCnt;i++)
	{
        if(i == m_iInterMultCnt -1)
        {
           //获取最后一个周期的状态和运行百分比
           eMStatus = Move(iErrorId,true);
           intdata.dPercent = m_dPercent;
        }
        else
        {
            Move(iErrorId);
        }

		//LOG_ALGO("InterJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
			m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],\
			m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);
		memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_dRJPos,sizeof(double)*MaxAxisNum);

		if(m_tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Master)
		{
			m_HS_Kinematic->HS_JPosToCPos(m_dRJPos,m_iToolNum,-1,m_dMasterCPos[i]);
		}
	}
	//LOG_ALGO("------------------------------------------------------------------------");
	return eMStatus;
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_Joint::Stop()
{
	return StopPlan();
}
/************************************************
函数功能：运动调速
参    数：设定的速度倍率
返 回 值：错误码
*************************************************/
int HS_Int_Joint::setRatio(double dRatio)
{
    if(m_bStopFlag)	//执行减速停止，响应调速
        return 0;
	return Ratio(dRatio);
}