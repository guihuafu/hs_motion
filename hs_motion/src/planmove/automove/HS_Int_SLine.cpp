
/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_SLine.cpp
* 摘    要：直线运动同步插补算法

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HS_Int_SLine.h"

double HS_Int_SLine::m_dDynPlanLength = 0;									//动态规划当前规划长度【距离起点，如果是平滑，则为平滑拐入点】
double HS_Int_SLine::m_dDynLOffset = 0;                            
double HS_Int_SLine::m_dDynPreEPos[MaxAxisNum] = {0};         
DynPlanState HS_Int_SLine::m_eDynPlanState = DYNPLAN_Start;    
Para_PreHandle HS_Int_SLine::m_tBasePH_Line = Para_PreHandle();
Para_PreHandle HS_Int_SLine::m_tBasePH_LinePre = Para_PreHandle();
BezierCurve HS_Int_SLine::m_tBezierCurve = BezierCurve();
int HS_Int_SLine::m_iLookAhead_SL = 0;
bool HS_Int_SLine::m_bBezierHandle = false;
double HS_Int_SLine::m_dDivFoundJPos[MaxAxisNum] = {0};
bool HS_Int_SLine::m_bSLStopFlag = false;
double HS_Int_SLine::m_dDynDivLength;
double HS_Int_SLine::m_dDynDivLengthMax;						
/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Int_SLine::HS_Int_SLine(HS_GroupKin *pGroupKin)
{
	m_HS_GroupKin = pGroupKin;
	m_eMoveType = MP_BLine;
	m_dKVelMax = 1.15;                          //提速约束
	m_dDynDivLength = 3.0;						//决定最大约束速度,m_dDynDivLength/m_dCycle 8.0---2000 12.0---3000,靠近奇异位置长度要减小，使得速度控制更加精准
	m_eDynPlanState = DYNPLAN_Start;
	m_iLookAhead_SL = 0;
	m_bSLLookAheadPlanFlag = false;

	m_HS_VelPlan_ParaPre = new HS_VelPlan_Para *[SpaceAxisNum];
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		m_HS_VelPlan_ParaPre[i] = new HS_VelPlan_Para(m_dCycle);
	}
}

HS_Int_SLine::~HS_Int_SLine()
{
    for(int i = 0;i < SpaceAxisNum;i++)
        delete m_HS_VelPlan_ParaPre[i];
    delete[] m_HS_VelPlan_ParaPre; 
    m_HS_VelPlan_ParaPre = NULL;
}

/************************************************
函数功能：参数复位
参   数：    
返 回 值：错误码
*************************************************/
int HS_Int_SLine::ResetData()
{
	m_bBezierHandle = false;
	m_iLookAhead_SL = 0;
	m_bBezierHandle = false;
	m_dDynPlanLength = 0;
	m_dDynLOffset = 0; 
	m_bSLStopFlag = false;
	m_iDynIndex = -1;
	return 0;
}
/************************************************
函数功能：执行 运动的预处理过程
参   数：elemt----运动输入信息
		trajout---规划缓存信息    
返 回 值：错误码
*************************************************/
int HS_Int_SLine::PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum)
{
	int iErrorId = 0;

	tTrajData[iIndex].tMotionData = tGroupMotionData;

	BaseMoveData *tMoveData = &tGroupMotionData.tBaseMoveData[iGroupNum];

	m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate; 

	Para_PreHandle tPH_Line;
	memset(&tPH_Line,0,sizeof(tPH_Line));
	tPH_Line.eMoveType = MP_Line;
	tPH_Line.bPreHandled = true;
	m_tGTrajData = tTrajData;
	m_iIndex = iIndex;
	m_iGroupNum = iGroupNum;
	m_HS_Kinematic = m_HS_GroupKin->GetKinematicsByNum(iGroupNum);
	m_eHS_RobotType = m_HS_Kinematic->GetRobotType();
	m_bWristQYFlag = m_HS_Kinematic->HS_SetWristQYHandle(tGroupMotionData.bWristQYFlag);
	GetToolWorkNum(*tMoveData);

	while(1)
	{
		iErrorId = GetStartPos(*tMoveData,tPH_Line);
		if(iErrorId != 0) break;

		iErrorId = GetEndPos(*tMoveData,tPH_Line);
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
		break;
	}

	HandleParaWithPre(tPH_Line);

	//存储预处理数据
	tPH_Line.iPreHandledError = iErrorId;
	tPH_Line.eState = m_eState;
	memcpy(tTrajData[iIndex].iData[iGroupNum],&tPH_Line,sizeof(Para_PreHandle));

	return iErrorId;
}
/************************************************
函数功能：获取设置的工具工件号
参   数：elemt----运动输入信息
返 回 值：无
*************************************************/
void HS_Int_SLine::GetToolWorkNum(BaseMoveData tMoveData)
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

    LOG_ALGO("Move Tool = %d,Work = %d",m_iToolNum,m_iWorkNum);
}
/************************************************
函数功能：获取 运动起点的坐标
参   数：elemt----运动输入信息
		tPH_Line---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
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

	return iErrorId;
}

/************************************************
函数功能：获取 运动终点的坐标
参   数：elemt----运动输入信息
		tPH_Line---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line)
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
	}
	return iErrorId;
}
/************************************************
函数功能：获取 运动位移量等信息
参   数：tPH_Line---预处理输入输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetMoveDis(Para_PreHandle &tPH_Line)
{
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

	int iErrorId = 0;
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
    else
    {
#ifdef _LINUX_
		struct timeval l_beginTime_Auto, l_endTime_Auto;
		gettimeofday(&l_beginTime_Auto, NULL);
#endif
        MoveAdjust(tPH_Line);
#ifdef _LINUX_
		gettimeofday(&l_endTime_Auto, NULL);
		double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
		LOG_ALGO("Time_MoveAdjust = %.3lf\n",de_us_Auto);
#endif
    }

	memcpy(tPH_Line.dOrigDis,tPH_Line.dSetDis,sizeof(double)*SpaceAxisNum);

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
函数功能：对运动规划的参数信息进行调整，包含结束点关节坐标，以及A360处理等
参    数：
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::MoveAdjust(Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;

	if(m_eHS_RobotType == HSROB_SCARA)
	{
		double dEJPos[6] = {0};
		iErrorId = JPosPrediction(tPH_Line,dEJPos);
		if(iErrorId != 0) return iErrorId;

		if(m_HS_Kinematic->GetA360Flag())
		{
			//当前姿态角度旋转对应关节4轴旋转的方向
			bool bAngleDir = true;
			if(dEJPos[3] - tPH_Line.dSetJPos[0][3] < 0)
			{
				bAngleDir = false;
			}
			//A360模式下，根据目标点位置以及预测的关节角度值，通过修改姿态运动位移量的方式使得运动达到目标点的4轴坐标值
			while(fabs(dEJPos[3] - tPH_Line.dSetJPos[1][3]) > 181)
			{
				if(dEJPos[3] > tPH_Line.dSetJPos[1][3])
				{
					dEJPos[3] -= 360;
					if(bAngleDir)
						tPH_Line.dSetDis[1] -= 360;
					else
						tPH_Line.dSetDis[1] += 360;
				}
				else
				{
					dEJPos[3] += 360;
					if(bAngleDir)
						tPH_Line.dSetDis[1] += 360;
					else
						tPH_Line.dSetDis[1] -= 360;
				}
			}
		}
		else
		{
			m_HS_Kinematic->HS_NearestPoint(tPH_Line.dSetJPos[1][3],dEJPos[3],-1);
			JPosAutoHandle(tPH_Line,dEJPos,3);
		}
	}
	else if(m_eHS_RobotType == HSROB_PUMA)
	{
		double dEJPos[6] = {0};
		iErrorId = JPosPrediction(tPH_Line,dEJPos);
		if(iErrorId == -1)
		{
			iErrorId = 0;
		}
		else
		{
			if(iErrorId != 0) return iErrorId;
			m_HS_Kinematic->HS_NearestPoint(tPH_Line.dSetJPos[1][3],dEJPos[3],-1);
			m_HS_Kinematic->HS_NearestPoint(tPH_Line.dSetJPos[1][5],dEJPos[5],-1);

			JPosAutoHandle(tPH_Line,dEJPos,3);
			JPosAutoHandle(tPH_Line,dEJPos,5);
		}
	}

    return iErrorId;
}

/************************************************
函数功能：多转轴角度的修正
参    数：
		 tPH_Line---处理缓存结构体
         dEJPos-----预测得到的关节点位
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::JPosAutoHandle(Para_PreHandle &tPH_Line,double dEJPos[6],int iAxis)
{
	const double dErrorLimit = 2.0;
	
	if(tPH_Line.dSetJPos[1][iAxis] - tPH_Line.dSetJPos[0][iAxis] > dErrorLimit&&
		dEJPos[iAxis] - tPH_Line.dSetJPos[0][iAxis] < -dErrorLimit)
	{
		tPH_Line.dSetJPos[1][iAxis] -= 360;
		LOG_ALGO("Axis%d ReCalc JPos Backward = %.3lf,PreJPos = %.3lf",
			iAxis,tPH_Line.dSetJPos[1][iAxis],dEJPos[iAxis]);
	}
	else if(tPH_Line.dSetJPos[1][iAxis] - tPH_Line.dSetJPos[0][iAxis] < -dErrorLimit&&
		dEJPos[iAxis] - tPH_Line.dSetJPos[0][iAxis] > dErrorLimit)
	{
		tPH_Line.dSetJPos[1][iAxis] += 360;
		LOG_ALGO("Axis%d ReCalc JPos Backward = %.3lf,PreJPos = %.3lf",
			iAxis,tPH_Line.dSetJPos[1][iAxis],dEJPos[iAxis]);
	}

	return 0;
}
/************************************************
函数功能：对运动的结束点进行粗预测
参    数：
		 tPH_Line---处理缓存结构体
         dEJPos-----预测得到的关节点位
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::JPosPrediction(Para_PreHandle &tPH_Line,double dEJPos[6])
{
    int iErrorId = 0;

    SyncPara tSync;
    tSync.tPreHandle = tPH_Line;

    double dPlanPos[SpaceAxisNum] = {0};
    double dMPos[5][4] = {0};

    const int DIFFCNT = 15;
    double dSJPos[6] = {0};
    
    memcpy(dSJPos,tPH_Line.dSetJPos[0],sizeof(double)*6);

    double dJPosCalc[6] = {0};
    for(int i = 0;i < DIFFCNT;i++)
    {
        for(int j = 0;j < 2;j++)
        {
            dPlanPos[j] = tPH_Line.dSetDis[j]*(i+1)/DIFFCNT; 
        }

        PlanPos2MPos(tSync,dPlanPos,dMPos);

        iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos,m_iToolNum,m_iWorkNum,CP_ToolWork,dSJPos,dJPosCalc);
        if(iErrorId != 0)
            return iErrorId;
		if(dJPosCalc[4]*tPH_Line.dSetJPos[0][4] < -Eps)
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
函数功能：Scara机型对运动的结束点进行粗预测
参    数：
		 tPH_Line---处理缓存结构体
         dEJPos-----预测得到的关节点位
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::JPosPrediction_Scara(Para_PreHandle &tPH_Line,double dEJPos[6])
{
    int iErrorId = 0;

    SyncPara tSync;
    tSync.tPreHandle = tPH_Line;

    double dPlanPos[SpaceAxisNum] = {0};
    double dMPos[5][4] = {0};

    const int DIFFCNT = 10;
    double dSJPos[6] = {0};
    
    memcpy(dSJPos,tPH_Line.dSetJPos[0],sizeof(double)*6);

    double dJPosCalc[6] = {0};
    for(int i = 0;i < DIFFCNT;i++)
    {
        for(int j = 0;j < 2;j++)
        {
            dPlanPos[j] = tPH_Line.dSetDis[j]*(i+1)/DIFFCNT; 
        }

        PlanPos2MPos(tSync,dPlanPos,dMPos);

        iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos,m_iToolNum,m_iWorkNum,CP_ToolWork,dSJPos,dJPosCalc);
        if(iErrorId != 0)
            return iErrorId;
        memcpy(dSJPos,dJPosCalc,sizeof(double)*6);
    }

    memcpy(dEJPos,dJPosCalc,sizeof(double)*6);

    return iErrorId;
}
/************************************************
函数功能：计算与前段运动有关的信息
参    数：
		 tPreHandle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::HandleParaWithPre(Para_PreHandle &tPH_Line)
{
    int iBuffPre = m_iIndex - 1;
    if(iBuffPre < 0)
        iBuffPre = MaxBuffSize -1;
    GroupTrajData *pMPre =  m_tGTrajData + iBuffPre;
    Para_PreHandle tPHPre;					
    memcpy(&tPHPre,pMPre->iData,sizeof(Para_PreHandle));
    if(pMPre->tMotionData.dCnt > Eps&&(tPHPre.eMoveType == MP_Line||tPHPre.eMoveType == MP_BLine))
    {
        //计算夹角值
        double dL1[3] = {tPHPre.dDis[0],tPHPre.dDis[1],tPHPre.dDis[2]};
        double dL2[3] = {tPH_Line.dDis[0],tPH_Line.dDis[1],tPH_Line.dDis[2]};
        double dDot = -(dL1[0]*dL2[0] + dL1[1]*dL2[1] + dL1[2]*dL2[2]);
        double dNormL1 = sqrt(dL1[0]*dL1[0] + dL1[1]*dL1[1] + dL1[2]*dL1[2]);
        double dNormL2 = sqrt(dL2[0]*dL2[0] + dL2[1]*dL2[1] + dL2[2]*dL2[2]);
        double dTemp = 1.0;
        if(dNormL1 > Eps&&dNormL2 > Eps)
        {
            dTemp = dDot/dNormL1/dNormL2;
            if(dTemp > 1.0)
                dTemp = 1.0;
            else if(dTemp < -1.0)
                dTemp = -1.0;
        }
        tPH_Line.dSmoothCAngle = dTemp;	
    }
    return 0;
}
/************************************************
函数功能：点位检测，判断是否超限位、奇异等
参    数：
		 tPH_Line---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::CheckPosLimit(Para_PreHandle &tPH_Line)
{
	int iErrorId = 0;

	iErrorId = m_HS_Kinematic->HS_JPosLimitCheck(tPH_Line.dSetJPos[1]);
    if(iErrorId != 0) 
        return E_L_TARGETOVERLIMIT;

    iErrorId = m_HS_Kinematic->HS_QYLimitCheck(tPH_Line.dSetJPos[0]);

    if(!(m_bWristQYFlag&&iErrorId == ERROR_QY_WRIST))
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
		 tPreHandle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPreHandle)
{
    int iErrorId = 0;
  	//初始参数设置
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPreHandle.dSetKVel[i] = 1.0;					
	}
	tPreHandle.dSetKAcc[0] = 1.0;
	tPreHandle.dSetKAcc[1] = 1.0;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPreHandle.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPreHandle.dSetMaxAcc = tMoveData.dDec/100;

	tPreHandle.dSetVel[0] = tMoveData.dVel;
	tPreHandle.dSetVel[1] = m_dCVelPara[1]*tMoveData.dVort/100;

	Para_PreHandle tPH = tPreHandle;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPreHandle.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPreHandle.dSetMaxAcc = tMoveData.dDec/100;	
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	if (gettimeofday(&l_beginTime_Auto, NULL) == -1)
		return -1;
#endif
	iErrorId = AutoAdjustJAcc(tPreHandle);
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
int HS_Int_SLine::AutoAdjustJAcc(Para_PreHandle &tPreHandle)
{
	Para_PreHandle tPH = tPreHandle;
	SyncPara tSyncPara;
    memset(&tSyncPara,0,sizeof(tSyncPara));
	int iErrorId = 0;
	double dMaxKJVel[3] = {0};					//运动的 速度比例系数，速度的非线性处理	
	for(int iCnt = 0;iCnt < MAXCNT;iCnt++)
	{
/************STEP1：规划参数的设定******************************/
		for(int i = 0;i < SpaceAxisNum;i++)
		{
			tPH.dSetVel[i] = tPreHandle.dSetVel[i]*tPH.dSetKVel[i];
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
		for(int i = 0;i < MaxAxisNum;i++)
		{
			double dTemp = Max(dKVel[0][i],dKVel[1][i]);
			dTemp = Max(dTemp,dKVel[2][i]);
			dMaxKVel = Max(dMaxKVel,dTemp);
		}

		//缓存当前的 速度比例值
		tPreHandle.dOrigKJVel = dMaxKVel;

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
    memcpy(tPreHandle.dPreHandleVel,tSyncPara.tVelPlan.dEVel,sizeof(double)*SpaceAxisNum);
	const double MAXKJACC = 50.0;
	const double MINKJACC = 0.001;
	//对最大最小比例进行限制
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPreHandle.dSetKVel[i] = tPH.dSetKVel[i];
	}
	for(int i = 0;i < 2;i++)
	{
		tPreHandle.dSetKAcc[i] = tPH.dSetKAcc[i];
		if(tPreHandle.dSetKAcc[i] < MINKJACC)
			tPreHandle.dSetKAcc[i] = MINKJACC;
		else if(tPreHandle.dSetKAcc[i] > MAXKJACC)
			tPreHandle.dSetKAcc[i] = MAXKJACC;
	}
	return 0;
}


/************************************************
函数功能：运动规划
参    数：dSDece-----减速运行时的减速度
		 pMCur------运行段信息
		 dRatio-----速度比例因子
		 dRealJPos--当前实际的 位置
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
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

	//提前获取
	m_tSyncBase = m_tSync;
	m_eState = m_tSync.tPreHandle.eState;

	iErrorId = GetSysPara(tTrajData,iIndex,dRatio);
	if(iErrorId != 0) return iErrorId;  

	DivLengthHandle();

	//普通小线段规划
	//iErrorId = TrajectoryPlan();
	//if(iErrorId != 0) return iErrorId;  

	//点位匀顺处理
	iErrorId = LengthAheadHandle();

	//动态规划处理
	if(m_eDynPlanState == DYNPLAN_End&&m_iLookAhead_SL == 0)
	{
		m_eDynPlanState = DYNPLAN_Start;
		m_bBezierHandle = false;
	}

	if(m_iLookAhead_SL > 0)
		m_iLookAhead_SL--;

	iErrorId = DynPlan();
	if(iErrorId != 0) return iErrorId;  

	memcpy(m_dRJPos,m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);

	VelPlan();

	m_bPlanFlag = true;
	return iErrorId;
}
/************************************************
函数功能：对动态规划的离散化间隔进行处理
	离散化间隔的间距需要在不同的条件下进行一定的调整
	间距越大，空间位移的最大速度就越大，最大速度 = 离散化间隔/插补周期
	间距越小，关节速度的约束有效性越好
参    数：
返 回 值：无
*************************************************/
void HS_Int_SLine::DivLengthHandle()
{
	m_dDynDivLength = 6.0;

	double dDynDivBase = 6.0;

	//起末点的处理
	double dSDegree = GetQYDegree(m_tSync.tPreHandle.dSetJPos[0]);

	double dEDegree = GetQYDegree(m_tSync.tPreHandle.dSetJPos[1]);

	m_dDynDivLengthMax = dDynDivBase*dEDegree;

	//如果与前段无平滑
	int iIndexPre = m_iIndex - 1;
	if(iIndexPre == -1)
		iIndexPre = MaxBuffSize - 1;

	if(!(m_tGTrajData[iIndexPre].tMotionData.dCnt > Eps&&m_tGTrajData[iIndexPre].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_BLine))
	{
		m_dDynDivLength = dDynDivBase*dSDegree;
	}

}
/************************************************
函数功能：获取给定点位的离散化细分间隔参考
参    数：dJPos-----实际关节位置

返 回 值：无
*************************************************/
double HS_Int_SLine::GetQYDegree(double *dJPos)
{
	double dDegree = 1.0;
	if(m_HS_Kinematic->GetRobotType() == HSROB_PUMA)
	{
		//腕部奇异位置处理
		double dWristLimit[3] = {1.0,5.0,10.0};
		double dKLimit[3] = {0.1,0.2,1.0};

		for(int i = 0;i < 3;i++)
		{
			if(fabs(dJPos[4]) < dWristLimit[i])
			{
				dDegree = dKLimit[i];
				if(i > 0)
				{
					dDegree = dKLimit[i-1] + (fabs(dJPos[4]) - dWristLimit[i-1])/(dWristLimit[i] - dWristLimit[i-1])*(dKLimit[i] - dKLimit[i-1]);
				}
				break;
			}
		}

	}
	return dDegree;
}
/************************************************
函数功能：暂停重启运动规划
参    数：
		 ratio-------速度比例因子
		 realjpos----当前实际的关节位置
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll)
{
    int iErrorId = 0;
	double dSetRatioPre = m_dSetRatio;

    m_dSetRatio = dRatio;    
	m_bSLStopFlag = false;

    LOG_ALGO("Restart Plan,Id = %d!",m_iIndex);
	m_eRstState = eRstState;
	m_dTRSTPreAll = dTAll;

    //直接缓存停止距离StopDis，重新规划
    //iErrorId = TrajectoryPlan();
    //if(iErrorId != 0) return iErrorId;  

	if(m_tSync.tPreHandle.bSmallLineFlag)
	{
		//方案A：小线段模式，清除所有的后续点位数据，重新开始规划，且当前段不进行运动【停止时已运动完成】
		memset(m_tSyncAhead,0,sizeof(m_tSyncAhead));			
		m_iLookAhead[m_iGroupNum] = 0;
		m_iMaxLookAheadLine = 1;
		m_iLookAhead_SL = 0;
		m_bSLStopFlag = false;
		m_eDynPlanState = DYNPLAN_Start;
		return 1;
	}
	
	//方案B，利用之前的数据，继续进行运动，点位预处理数据可以保留，速度规划需要重新进行	
	//速度倍率可能不同，需要重新处理

	LOG_ALGO("SLine Restart Plan,AheadCnt = %d!",m_iLookAhead[m_iGroupNum]);

	if(m_iLookAhead[m_iGroupNum] == 1)
	{
		int iBreak = 1;
	}

	double dKRatioChange = 1.0;

	if(dSetRatioPre > Eps)
		dKRatioChange = m_dSetRatio/dSetRatioPre;

	for(int iLookAhead = 0;iLookAhead < m_iMaxLookAheadLine;iLookAhead++)
	{		
		if(iLookAhead < m_iLookAhead[m_iGroupNum])
		{
			m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];

			//修改速度倍率
			for(int i = 0;i < SpaceAxisNum;i++)
			{
				m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetVel[i] *= dKRatioChange;
				m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetAcc[i] *= dKRatioChange;
				m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetDec[i] *= dKRatioChange;
			}
		}
	}
	if(m_iMaxLookAheadLine > 1)
		m_iMaxLookAheadLine--;

	if(m_iLookAhead[m_iGroupNum] > 1)
	{
		m_iLookAhead[m_iGroupNum]--;

		SyncByAcc(m_tSyncAhead[m_iGroupNum][0]);

		//重新速度规划
		for(int iLookAhead = 0;iLookAhead < m_iMaxLookAheadLine;iLookAhead++)
		{		
			bool bStopSmooth = false;
				
			SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
			
			//运动结束，分平滑结束和总运动结束
			if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eDynState == DYNPLAN_End)
			{         
				bStopSmooth = true;
			}			

			bool bSLModeFlag = CheckSmallLimeUpMode(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);

			if(bSLModeFlag&&!bStopSmooth)
			{
				if(!m_bSLPlanFlag)
					iErrorId = SmoothHandle_Forward(iLookAhead,true);
				else
					iErrorId = SmoothHandle_Forward(iLookAhead);

				m_bSLPlanFlag = true;
			}
			else
			{                   
				if(m_bSLPlanFlag)
				{
					int iRet = 0;
					if(bSLModeFlag)
						iRet = SmoothHandle_Backward(iLookAhead+1);
					else
						iRet = SmoothHandle_Backward(iLookAhead);
					m_bSLPlanFlag = false;
					LOG_ALGO("BackWardPlan!SLModeFlag = %d",bSLModeFlag);
				}
				else
				{
					iErrorId = SmoothHandle(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
				}
			}
		}			
		m_bSmoothNextFlag = true;
	}
	else
	{
		SyncByAcc(m_tSyncAhead[m_iGroupNum][0],true,true);
		m_tSync.tVelPlan.dTSmooth[NEX] = 0;
		m_bSmoothNextFlag = false;
		m_iLookAhead[m_iGroupNum] = 0;
	}

	m_tSync = m_tSyncAhead[m_iGroupNum][0];
	m_tSync.tVelPlan.dTSmooth[PRE] = 0;

	m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
	m_dTCon = m_tSync.tVelPlan.dTime[TCON];
	m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
	m_dTAll = m_tSync.tVelPlan.dTime[TALL];

    VelPlan();

    dTAll = m_dTAll;

    return iErrorId;
}

/************************************************
函数功能：从上层给定的运动结构体数据中获取待使用的信息
参    数：pMCur---运动结构体
		 dRatio--速度比例因子
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio)
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

	m_HS_Kinematic->HS_SetCoordinate(m_iToolNum,m_iWorkNum);
	return 0;
}
/************************************************
函数功能：插补规划，根据给定的位置速度加速度等信息进行速度、轨迹的插补规划
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_SLine::TrajectoryPlan(void)
{						
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

    for(iLookAhead = 0;iLookAhead < m_iMaxLookAheadLine;iLookAhead++)
    {
        //已规划部分
        if(iLookAhead < m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
            continue;
        }
        else if(iLookAhead == m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
            //如果无小线段规划，则逐步将前瞻段数压缩至1，保证后续的前瞻段数有效性
            if(!m_bSLPlanFlag)
            {
                if(iLookAhead > 0&&m_iMaxLookAheadLine > 1)
                {
                    m_iMaxLookAheadLine--;
                    break;
                }
            }
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
                if(m_tGTrajData[m_iIndex].tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,(m_iIndex + 1)%MaxBuffSize) == 0)
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true);	
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
            m_bSLPlanFlag = false;
        }

        int iBuffCur = (m_iIndex + iLookAhead)%MaxBuffSize;
        GroupTrajData tTrajDataCur = m_tGTrajData[iBuffCur];
        int iBuffNext = (m_iIndex + iLookAhead + 1)%MaxBuffSize;
        GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext];   

        if(tTrajDataCur.tMotionData.dCnt > Eps)
        {
            int iRet = CheckTrajDataAvailable(m_tGTrajData,iBuffNext);
            if(iRet == 0)
            {
                //混合平滑处理
                memset(&m_tSyncAhead[m_iGroupNum][iLookAhead+1],0,sizeof(SyncPara));
                GetInputParaGE(m_tGTrajData,iBuffNext,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle,m_dSetRatio);

                int iBuffNext2 = (m_iIndex + iLookAhead + 2)%MaxBuffSize;
                bool bStopSmooth = false;
                if(tTrajDataNex.tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iBuffNext2) == 0)
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                else
                {
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1],false,true);
                    bStopSmooth = true;
                }

                if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Joint)
                    LOG_ALGO("Smooth Handle With Joint!");
                else if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Line)
                    LOG_ALGO("Smooth Handle With Line!");
                else if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eMoveType == MP_Arc)
                    LOG_ALGO("Smooth Handle With Cicle!");

                bool bSLModeFlag = CheckSmallLimeUpMode(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);

                if(bSLModeFlag&&!bStopSmooth)
                {
                    if(!m_bSLPlanFlag)
                        iErrorId = SmoothHandle_Forward(iLookAhead,true);
                    else
                        iErrorId = SmoothHandle_Forward(iLookAhead);

                    //增加前瞻段数
                    if(m_iMaxLookAheadLine < MAXLOOKAHEADSLINE)
                    {
                        m_iMaxLookAheadLine++;
                    }                  
                    m_bSLPlanFlag = true;
                }
                else
                {                   
                    if(m_bSLPlanFlag)
                    {
                        int iRet = 0;
                        if(bSLModeFlag)
                            iRet = SmoothHandle_Backward(iLookAhead+1);
                        else
                            iRet = SmoothHandle_Backward(iLookAhead);
                        m_bSLPlanFlag = false;
                    }
                    else
                    {
                         iErrorId = SmoothHandle(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                    }
                }
                if(iErrorId != 0)
                    return iErrorId;
            }
            else if(iRet == -1)
            {
                if(iLookAhead == 0)
                {
                    //无点位，动态平滑规划
                    m_bDynSmoothFlag = true;
                    LOG_ALGO("Need Dyn Smooth Plan!");
                }
                else
                {
                    if(m_bSLPlanFlag)
                    {
                        int iRet = SmoothHandle_Backward(iLookAhead);
                        m_bSLPlanFlag = false;
                    }
                }
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

    if(m_tGTrajData[m_iIndex].tMotionData.dCnt > Eps)
    {
        m_bSmoothNextFlag = true;
    }
    else 
        m_bSmoothNextFlag = false;

	m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
	m_dTCon = m_tSync.tVelPlan.dTime[TCON];
	m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
	m_dTAll = m_tSync.tVelPlan.dTime[TALL];
	return 0;
}
/************************************************
函数功能：点位前瞻处理，进行小线段识别以及匀顺处理优化等
参    数：		 
返 回 值：报警码
*************************************************/
int HS_Int_SLine::LengthAheadHandle()
{
	int iErrorId = 0;
	int iAheadIndex = m_iIndex;

	//先简单处理下
	int iCnt = 0;
	while(iCnt < MaxBuffSize)
	{
		if(m_tGTrajData[iAheadIndex].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_BLine)
		{
			Para_PreHandle tPreHandle = Para_PreHandle();
			memcpy(&tPreHandle,m_tGTrajData[iAheadIndex].iData[m_iGroupNum],sizeof(Para_PreHandle));  

			if(tPreHandle.dLength < m_dDynDivLength)
				tPreHandle.bSmallLineFlag = true;

			memcpy(m_tGTrajData[iAheadIndex].iData[m_iGroupNum],&tPreHandle,sizeof(Para_PreHandle));
		}
		else break;

		iAheadIndex = (iAheadIndex + 1)%MaxBuffSize;
		iCnt++;
	}

	return iErrorId;
}
/************************************************
函数功能：动态规划：通过动态点位的离散化完成约束与提速
         分动态规划状态和实际执行段状态
         无平滑时：规划启动---结束
         有平滑时：启动---过渡至下一段【返回信号】
参    数：		 
返 回 值：小线段离散化动态处理
*************************************************/
int HS_Int_SLine::DynPlan()
{
    int iErrorId = 0;    
    int iLookAhead = 0;
    m_bDynSmoothFlag = false;

	int iDynLookCnt = 0;		//限制动态规划的个数，防止超时
	const int MAXDYNLOOKCNT = 2;

    //运行段信息
    int iAheadIndex = m_iIndex + m_iLookAhead_SL;

	//小线段前瞻规划运动信息
    for(iLookAhead = 0;iLookAhead < m_iMaxLookAheadLine&&iDynLookCnt < MAXDYNLOOKCNT;iLookAhead++)
    {
        //已规划部分
        if(iLookAhead < m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];

            continue;
        }
        else if(iLookAhead == m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];

            if(m_eDynPlanState == DYNPLAN_End)
            {
                if(iLookAhead > 0&&m_iMaxLookAheadLine > 1)
                {
                    m_iMaxLookAheadLine--;                    
                }
                break;
            }
        }
        else if(iLookAhead == 0)
        {
            if(m_eDynPlanState == DYNPLAN_End)
                break;

            memset(&m_tSyncAhead[m_iGroupNum][iLookAhead],0,sizeof(SyncPara));
            GetDynData(m_tGTrajData,m_iIndex,m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle,m_dSetRatio);

			DynPlanState eDynState = m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.eDynState;

            if(eDynState == DYNPLAN_Move||eDynState == DYNPLAN_Start)
            {
				SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead]);
			}
			else if(eDynState == DYNPLAN_EndSmooth)
			{
				SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true);
				m_iLookAhead_SL++;
				iAheadIndex = m_iIndex + m_iLookAhead_SL;
				m_bBezierHandle = true;
			}
            else
                SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	          

            m_bSLPlanFlag = false;
        }

        if(m_eDynPlanState == DYNPLAN_End)
            break;

		iErrorId = DynLookAheadHandle(iLookAhead);
        if(iErrorId != 0)
            return iErrorId;   
		iDynLookCnt++;
        
    }

    m_iLookAhead[m_iGroupNum] = iLookAhead;

    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dAcc,m_tSync.tVelPlan.dAcc,sizeof(m_tSync.tVelPlan.dAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dDec,m_tSync.tVelPlan.dDec,sizeof(m_tSync.tVelPlan.dDec));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dJerk,m_tSync.tVelPlan.dJerk,sizeof(m_tSync.tVelPlan.dJerk));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopAcc,m_tSync.tVelPlan.dStopAcc,sizeof(m_tSync.tVelPlan.dStopAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopJerk,m_tSync.tVelPlan.dStopJerk,sizeof(m_tSync.tVelPlan.dStopJerk));
    m_tSync = m_tSyncAhead[m_iGroupNum][0];

	if(m_bSLStopFlag)
	{
		if(m_tSync.tVelPlan.dTSmooth[NEX] < Eps)
		{
			m_tSync.tPreHandle.eDynState = DYNPLAN_End;
		}
	}

    if(m_tSync.tPreHandle.eDynState == DYNPLAN_End)
    {
        m_bSmoothNextFlag = false;
    }
    else 
        m_bSmoothNextFlag = true;

	m_bSLLookAheadPlanFlag = true;

    m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
    m_dTCon = m_tSync.tVelPlan.dTime[TCON];
    m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
    m_dTAll = m_tSync.tVelPlan.dTime[TALL];

    return iErrorId;
}
/************************************************
函数功能：动态规划前瞻处理，对前瞻数据进行处理
参    数：iLookAhead---前瞻段数	 
返 回 值：错误码
*************************************************/
int HS_Int_SLine::DynLookAheadHandle(int iLookAhead)
{
	int iErrorId = 0;
	int iAheadIndex = m_iIndex + m_iLookAhead_SL;
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	gettimeofday(&l_beginTime_Auto, NULL);
#endif
	memset(&m_tSyncAhead[m_iGroupNum][iLookAhead+1],0,sizeof(SyncPara));
	GetDynData(m_tGTrajData,iAheadIndex,m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle,m_dSetRatio);

	//平滑轨迹处理
	BezierSmoothHandle(iAheadIndex,m_bBezierHandle);

	bool bStopSmooth = false;
	if(m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eDynState == DYNPLAN_Move)
		SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
	else
	{
		//运动结束，分平滑结束和总运动结束
		if(m_bBezierHandle&&m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle.eDynState != DYNPLAN_End)
		{
			m_bBezierHandle = false;
			m_iLookAhead_SL++;
			iAheadIndex = m_iIndex + m_iLookAhead_SL;
			// GetDynData(m_tGTrajData,iAheadIndex,m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle,m_dSetRatio);
			m_eDynPlanState = DYNPLAN_Start;
			SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);          
		}
		else
		{
			SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1],false,true);          
			bStopSmooth = true;
		}
	}

	bool bSLModeFlag = CheckSmallLimeUpMode(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);

	if(bSLModeFlag&&!bStopSmooth)
	{
		if(!m_bSLPlanFlag)
			iErrorId = SmoothHandle_Forward(iLookAhead,true);
		else
			iErrorId = SmoothHandle_Forward(iLookAhead);

		//增加前瞻段数
		if(m_iMaxLookAheadLine < MAXLOOKAHEADSLINE)
		{
			m_iMaxLookAheadLine++;
		}                  
		m_bSLPlanFlag = true;
	}
	else
	{                   
		if(m_bSLPlanFlag)
		{
			int iRet = 0;
			if(bSLModeFlag)
				iRet = SmoothHandle_Backward(iLookAhead+1);
			else
				iRet = SmoothHandle_Backward(iLookAhead);
			m_bSLPlanFlag = false;
			LOG_ALGO("BackWardPlan!SLModeFlag = %d",bSLModeFlag);
		}
		else
		{
			iErrorId = SmoothHandle(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
		}
	}
#ifdef _LINUX_
	gettimeofday(&l_endTime_Auto, NULL);
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
	if(de_us_Auto > m_dCycle*1000*1000/2)
		LOG_ALGO("DynLookAheadHandle = %.3lf\n",de_us_Auto);
#endif
	return iErrorId;
}
/************************************************
函数功能：运动过程中的动态规划处理，一次处理一段动态规划
参    数：		 
返 回 值：
*************************************************/
int HS_Int_SLine::MoveDynPlan()
{
	int iErrorId = 0;
	int iLookAhead = m_iLookAhead[m_iGroupNum];
	if(iLookAhead < m_iMaxLookAheadLine)
	{
		if(m_eDynPlanState == DYNPLAN_End)
			return iErrorId;

		iErrorId = DynLookAheadHandle(iLookAhead);
		if(iErrorId != 0)
			return iErrorId;  
		iLookAhead++;
		m_iLookAhead[m_iGroupNum] = iLookAhead;
	}
	return iErrorId;
}
/************************************************
函数功能：贝塞尔轨迹平滑规划
参    数：平滑处理的运行行信息		 
返 回 值：
*************************************************/
void HS_Int_SLine::BezierSmoothHandle(int iAheadIndex,bool &bBezierHandle)
{
    if(!bBezierHandle)
    {
        int iBuffCur = iAheadIndex;
        GroupTrajData tTrajDataCur = m_tGTrajData[iBuffCur];
        int iBuffNext = (iAheadIndex + 1)%MaxBuffSize;
        GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext];   

        if(tTrajDataCur.tMotionData.dCnt > Eps&&tTrajDataCur.tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_BLine)
        {
            int iRet = CheckTrajDataAvailable(m_tGTrajData,iBuffNext);
            if(iRet == 0)
            {
                //求解以及缓存控制点【暂平滑系数代表平滑长度】
                double dLSmooth = tTrajDataCur.tMotionData.dCnt;

                Para_PreHandle tPH_Cur;
                Para_PreHandle tPH_Nex;
	            memcpy(&tPH_Cur,m_tGTrajData[iBuffCur].iData,sizeof(Para_PreHandle));   
                memcpy(&tPH_Nex,m_tGTrajData[iBuffNext].iData,sizeof(Para_PreHandle)); 

				double dLSmoothPre = dLSmooth;
				double dLSmoothNex = dLSmooth;

                if(dLSmoothPre > tPH_Cur.dLength/2)
                    dLSmoothPre = tPH_Cur.dLength/2;
                if(dLSmoothNex > tPH_Nex.dLength/2)
                    dLSmoothNex = tPH_Nex.dLength/2;

				//等长方案，取较小值
				//if(dLSmoothPre > dLSmoothNex)
				//	dLSmoothPre = dLSmoothNex;
				//else
				//	dLSmoothNex = dLSmoothPre;

                double dConPos[6][6] = {0};

                for(int i = 0;i < 3;i++)
                {
                    dConPos[0][i] = tPH_Cur.dEPos[i] - dLSmoothPre*tPH_Cur.dKXYZ[i];
                    dConPos[1][i] = tPH_Cur.dEPos[i] - dLSmoothPre*tPH_Cur.dKXYZ[i]*2/3;
                    dConPos[2][i] = tPH_Cur.dEPos[i] - dLSmoothPre*tPH_Cur.dKXYZ[i]/3;
                    dConPos[3][i] = tPH_Nex.dSPos[i] + dLSmoothNex*tPH_Nex.dKXYZ[i]/3;
                    dConPos[4][i] = tPH_Nex.dSPos[i] + dLSmoothNex*tPH_Nex.dKXYZ[i]*2/3;
                    dConPos[5][i] = tPH_Nex.dSPos[i] + dLSmoothNex*tPH_Nex.dKXYZ[i];
                }

                tPH_Cur.dDisSmooth[NEX] = dLSmoothPre;
                tPH_Nex.dDisSmooth[PRE] = dLSmoothNex;

                memcpy(tPH_Nex.dBzConPos,dConPos,sizeof(dConPos));

                memcpy(m_tGTrajData[iBuffCur].iData,&tPH_Cur,sizeof(Para_PreHandle)); 
                memcpy(m_tGTrajData[iBuffNext].iData,&tPH_Nex,sizeof(Para_PreHandle)); 
                bBezierHandle = true;

                LOG_ALGO("Id = %d With Nex LSmooth = %.3lf,%.3lf",iAheadIndex,dLSmoothPre,dLSmoothNex);
            }
        }
    }
}

/************************************************
函数功能：检测是否满足小线段提速模式
参    数：		 
返 回 值：是否满足小线段条件
*************************************************/
bool HS_Int_SLine::CheckSmallLimeUpMode(SyncPara tSyncCur,SyncPara tSyncNex)
{
    bool bSLFlag = false;

    if(tSyncCur.tPreHandle.eMoveType == MP_Line&&tSyncCur.tPreHandle.eMoveType == MP_Line)
    {
        const double dLengthLimit = 10.0;
        if(tSyncNex.tPreHandle.dSmoothCAngle < 0        //-0.95
            && tSyncCur.tPreHandle.dLength < dLengthLimit && tSyncNex.tPreHandle.dLength < dLengthLimit)
        {
            double dK = 1.0;
            if(tSyncNex.tPreHandle.dLength > Eps)
                dK = tSyncCur.tPreHandle.dLength/tSyncNex.tPreHandle.dLength;

            if(dK < 1.5&&dK > 0.7)
                bSLFlag = true;
        }
    }

    bSLFlag = true;

    return bSLFlag;
}
/************************************************
函数功能：获取输出信息进行数据的存储
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Line,double dRatio)
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
函数功能：离散化处理，得到离散化点位数据，并进行最大速度自适应约束处理
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetDynData(GroupTrajData *trajout,int index,Para_PreHandle &tPH_Line,double dRatio)
{
    double dLengthCur = m_dDynDivLength;        //当前段离散化长度
	/******************************************************************************************/
	double dDivAdd = 0.5;
	if(m_dDynDivLength < m_dDynDivLengthMax - dDivAdd)
	{
		m_dDynDivLength += dDivAdd;
	}
	//前瞻判断优化
	else if(m_dDynDivLength > m_dDynDivLengthMax + dDivAdd)
	{
		m_dDynDivLength -= dDivAdd;
	}
	/******************************************************************************************/

    Para_PreHandle tPreHandle;                  //每次获取，可能动态改变
    memcpy(&tPreHandle,trajout[index].iData,sizeof(Para_PreHandle));  

    /******************************************************************************************/
	while(1)
	{
		m_eDynPlanState = DYNPLAN_Move;
		//规划行切换
		if(m_iDynIndex != index)
		{
			m_iDynIndex = index;
			LOG_ALGO("Dyn Plan Id = %d",m_iDynIndex);

			m_tBasePH_LinePre = m_tBasePH_Line;
			GetInputPara(m_tGTrajData,index,m_iGroupNum,m_tBasePH_Line,dRatio);

			m_dDynPlanLength = 0;

			if(tPreHandle.dDisSmooth[PRE] > Eps)
			{
				//平滑段启动
#ifdef _LINUX_
				//struct timeval l_beginTime_Auto, l_endTime_Auto;
				//gettimeofday(&l_beginTime_Auto, NULL);
#endif
				m_tBezierCurve.init(tPreHandle.dBzConPos);

				dLengthCur = m_dDynDivLength - m_dDynLOffset;
				m_dDynLOffset = 0;

				//将起点的关节坐标重置为当前实际位置
				memcpy(tPreHandle.dSPos,m_dDynPreEPos,sizeof(double)*MaxAxisNum);

				int iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(tPreHandle.dSPos,m_iToolNum,m_iWorkNum,m_eState,tPreHandle.dSetJPos[0]);

				memcpy(trajout[index].iData,&tPreHandle,sizeof(Para_PreHandle)); 
				memcpy(&m_tBasePH_Line.dSetJPos[0],&tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
#ifdef _LINUX_
				//gettimeofday(&l_endTime_Auto, NULL);
				//double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
				//if(de_us_Auto > m_dCycle*1000*1000/8)
				//	LOG_ALGO("BezierCurveinit = %.3lf\n",de_us_Auto);
#endif
			}
			else
			{
				//与前段无平滑启动
				memcpy(m_dDynPreEPos,tPreHandle.dSPos,sizeof(double)*MaxAxisNum);

				m_eDynPlanState = DYNPLAN_Start;
				memcpy(&m_dDivFoundJPos,&tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
				
				if(tPreHandle.bSmallLineFlag)
				{
					tPH_Line = m_tBasePH_Line;
					m_bBezierHandle = true;	
					bool bLineSmoothNexFlag = false;
					int iIndexNex = (m_iDynIndex + 1)%MaxBuffSize;

					if(trajout[m_iDynIndex].tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iIndexNex) == 0
						&&m_tGTrajData[iIndexNex].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_BLine)
						bLineSmoothNexFlag = true;

					//无下一段平滑
					if(!bLineSmoothNexFlag)
					{
						m_eDynPlanState = DYNPLAN_End;
					}
					else
					{	
						m_eDynPlanState = DYNPLAN_EndSmooth;
					}
					tPH_Line.eDynState = m_eDynPlanState;
					break;
				}
			}
		}

		/******************************************************************************************/
		//运行段状态切换改变
		tPH_Line = m_tBasePH_Line;
		tPH_Line.dSmoothCAngle = -1.0;

		double dLengthAll = m_tBasePH_Line.dLength;

		if(tPreHandle.dDisSmooth[PRE] > Eps)
			dLengthAll += m_tBezierCurve.getLength() - tPreHandle.dDisSmooth[PRE];

		if(dLengthAll - m_dDynPlanLength - tPreHandle.dDisSmooth[NEX] < 2*dLengthCur - Eps)
		{
			if(tPreHandle.dDisSmooth[NEX] > Eps)
			{
				//平滑结束
				m_eDynPlanState = DYNPLAN_EndSmooth;
				m_dDynLOffset = dLengthAll - m_dDynPlanLength - tPreHandle.dDisSmooth[NEX] - dLengthCur;
			}
			else
			{
				//无平滑结束
				m_eDynPlanState = DYNPLAN_End;
				dLengthCur = dLengthAll - m_dDynPlanLength;
			}
		}    

		tPH_Line.eDynState = m_eDynPlanState;
		tPH_Line.dSetDis[0] = dLengthCur;
		tPH_Line.dLength = dLengthCur;
		//姿态以及附加轴同步处理
		if(m_tBasePH_Line.dLength > Eps)
		{
			double dKL = dLengthCur/m_tBasePH_Line.dLength;
			for(int i = 1;i < SpaceAxisNum;i++)
			{
				tPH_Line.dSetDis[i] = m_tBasePH_Line.dSetDis[i]*dKL;
			}
		}

		/******************************************************************************************/
		//位置获取及输出
		memcpy(tPH_Line.dSPos,m_dDynPreEPos,sizeof(double)*MaxAxisNum);

		if(fabs(tPH_Line.dSPos[0] - 200) < 1.0&&fabs(tPH_Line.dSPos[1] - 300) < 1.0&&fabs(tPH_Line.dSPos[2] - 252) < 1.0)
		{
			bool bRet = false;
		}

		m_dDynPlanLength += dLengthCur;

		if(tPreHandle.dDisSmooth[PRE] > Eps)
		{
			//平滑段获取
			double dBezierLength = m_tBezierCurve.getLength();
			if(m_dDynPlanLength < dBezierLength)
			{
#ifdef _LINUX_
				//struct timeval l_beginTime_Auto, l_endTime_Auto;
				//gettimeofday(&l_beginTime_Auto, NULL);
#endif
				double dEPos[MaxAxisNum] = {0};
				m_tBezierCurve.calcPosByLenght(m_dDynPlanLength,dEPos);
#ifdef _LINUX_
				//gettimeofday(&l_endTime_Auto, NULL);
				//double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
				////if(de_us_Auto > m_dCycle*1000*1000/20)
				//	LOG_ALGO("calcPosByLenght = %.3lf\n",de_us_Auto);
#endif

				GetPosByMixPos(m_tBasePH_Line,m_tBasePH_LinePre,dEPos);

				memcpy(tPH_Line.dEPos,dEPos,sizeof(double)*MaxAxisNum);
				memcpy(m_dDynPreEPos,tPH_Line.dEPos,sizeof(double)*MaxAxisNum);

				//重新计算位置和位移量等规划信息
				for(int i = 0;i < 3;i++)
					tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];

				tPH_Line.dLength = sqrt(tPH_Line.dDis[0]*tPH_Line.dDis[0] + tPH_Line.dDis[1]*tPH_Line.dDis[1] + tPH_Line.dDis[2]*tPH_Line.dDis[2]);
				tPH_Line.dSetDis[0] = tPH_Line.dLength;

				if(tPH_Line.dLength  > Eps)
				{
					tPH_Line.dKXYZ[0] = tPH_Line.dDis[0]/tPH_Line.dLength;
					tPH_Line.dKXYZ[1] = tPH_Line.dDis[1]/tPH_Line.dLength;
					tPH_Line.dKXYZ[2] = tPH_Line.dDis[2]/tPH_Line.dLength;
				}

				m_HS_Kinematic->EulerZYX_CalcQ(tPH_Line.dSPos,tPH_Line.dEPos,tPH_Line.dQ);
				m_HS_Kinematic->EulerZYX_CalcDis(&tPH_Line.dSPos[3],&tPH_Line.dEPos[3],&tPH_Line.dDis[3],true);

				for(int i = 6;i < MaxAxisNum;i++)
				{
					tPH_Line.dDis[i] = tPH_Line.dEPos[i] - tPH_Line.dSPos[i];
				}

				tPH_Line.dSetDis[1] = Rad2angle(tPH_Line.dQ[0]);
				//附加轴
				for(int i = 2;i < SpaceAxisNum;i++)
				{
					tPH_Line.dSetVel[i] = m_dJVelPara[i+4];
					tPH_Line.dSetDis[i] = tPH_Line.dDis[i+4];
				}

				memcpy(tPH_Line.dOrigDis,tPH_Line.dSetDis,sizeof(double)*SpaceAxisNum);

			}
			else
			{
				double dLengthNew = (m_dDynPlanLength - m_tBezierCurve.getLength()) + tPreHandle.dDisSmooth[PRE];

				GetPosByLength(m_tBasePH_Line,dLengthNew,tPH_Line.dEPos);
				memcpy(m_dDynPreEPos,tPH_Line.dEPos,sizeof(double)*MaxAxisNum);
			}
		}
		else
		{
			GetPosByLength(m_tBasePH_Line,m_dDynPlanLength,tPH_Line.dEPos);
			memcpy(m_dDynPreEPos,tPH_Line.dEPos,sizeof(double)*MaxAxisNum);
		} 
		break;
	}
    
    /*SyncPara tSyncPara;
    tSyncPara.tPreHandle = m_tBasePH_Line;
    double dMPos[5][4] = {0};
    PlanPos2MPos(tSyncPara,dPlanPos,dMPos);
    m_HS_Kinematic->HS_MPosToCPos(dMPos,tPH_Line.dSPos);*/

    /******************************************************************************************/
    //速度自适应限制
    tPH_Line.dSetVel[0] = trajout[index].tMotionData.tBaseMoveData[m_iGroupNum].dVel;
    tPH_Line.dSetVel[1] = m_dCVelPara[1]*trajout[index].tMotionData.tBaseMoveData[m_iGroupNum].dVort/100;

#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	gettimeofday(&l_beginTime_Auto, NULL);
#endif
	//启停段需要严格计算加速度，中间段通过平滑约束处理
	if(tPH_Line.eDynState == DYNPLAN_Start ||tPH_Line.eDynState == DYNPLAN_End)
		AutoAdjustPara(trajout[index].tMotionData.tBaseMoveData[m_iGroupNum],tPH_Line);
	else
		SLAutoPara(tPH_Line);  
#ifdef _LINUX_
		gettimeofday(&l_endTime_Auto, NULL);
		double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
		if(de_us_Auto > m_dCycle*1000*1000/2)
			LOG_ALGO("SLAutoPara = %.3lf\n",de_us_Auto);
#endif

	tPH_Line.dSetVel[0] = trajout[index].tMotionData.tBaseMoveData[m_iGroupNum].dVel*dRatio*tPH_Line.dSetKVel[0];
	tPH_Line.dSetVel[1] = m_dCVelPara[1]*trajout[index].tMotionData.tBaseMoveData[m_iGroupNum].dVort/100*dRatio*tPH_Line.dSetKVel[1];

	for(int i = 0;i < 2;i++)
	{
		tPH_Line.dSetAcc[i] = m_dCAccPara[i]*tPH_Line.dSetMaxAcc*tPH_Line.dSetKAcc[0]*dRatio;
		tPH_Line.dSetDec[i] = m_dCAccPara[i]*tPH_Line.dSetMaxAcc*tPH_Line.dSetKAcc[1]*dRatio;
	}
    return 0;
}
/************************************************
函数功能：混合轨迹，基于运动位移量获取姿态以及附加轴位置信息
参    数：
		tPH_Line------规划信息【当前段】
		tPH_LinePre---规划信息【前一段】
		dCPos---------获取空间位置【包含输入】
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetPosByMixPos(Para_PreHandle &tPH_Line,Para_PreHandle &tPH_LinePre,double dCPos[MaxAxisNum])
{
	int iErrorId = 0;
	double dMixPos[3] = {0};

	memcpy(dMixPos,dCPos,sizeof(dMixPos));

	//求解混合位置处于前后两段的运行长度
	/*算法错误，对垂直的两段轨迹正常，其他求解错误
	double dVecA[3] = {tPH_LinePre.dDis[0],tPH_LinePre.dDis[1],tPH_LinePre.dDis[2]};
	double dVecB[3] = {dMixPos[0] - tPH_LinePre.dSPos[0],dMixPos[1] - tPH_LinePre.dSPos[1],dMixPos[2] - tPH_LinePre.dSPos[2]};

	double dLengthPre = (dVecA[0]*dVecB[0] + dVecA[1]*dVecB[1] + dVecA[2]*dVecB[2])/tPH_LinePre.dLength;

	double dVecC[3] = {tPH_Line.dDis[0],tPH_Line.dDis[1],tPH_Line.dDis[2]};
	double dVecD[3] = {dMixPos[0] - tPH_Line.dSPos[0],dMixPos[1] - tPH_Line.dSPos[1],dMixPos[2] - tPH_Line.dSPos[2]};

	double dLegthCur = (dVecC[0]*dVecD[0] + dVecC[1]*dVecD[1] + dVecC[2]*dVecD[2])/tPH_Line.dLength;*/

	double dLengthPre = 0;
	double dLegthCur = 0;

	//计算位移长度，向量合成法进行计算
	double dMixVec[3] = {dMixPos[0] - tPH_Line.dSPos[0],dMixPos[1] - tPH_Line.dSPos[1],dMixPos[2] - tPH_Line.dSPos[2]};
	double dK1Vec[3] = {-tPH_LinePre.dKXYZ[0],-tPH_LinePre.dKXYZ[1],-tPH_LinePre.dKXYZ[2]};
	double dK2Vec[3] = {tPH_Line.dKXYZ[0],tPH_Line.dKXYZ[1],tPH_Line.dKXYZ[2]};

	double dTempA = dK1Vec[1]*dK2Vec[0] - dK1Vec[0]*dK2Vec[1];
	double dTempB = dK1Vec[2]*dK2Vec[0] - dK1Vec[0]*dK2Vec[2];
	double dTempC = dK1Vec[2]*dK2Vec[1] - dK1Vec[1]*dK2Vec[2];

	double dTemp = 0;
	int iIdA = 0;
	int iIdB = 0;
	
	if(fabs(dTempA) > Eps)
	{
		dTemp = dTempA;
		iIdA = 0;
		iIdB = 1;
	}
	else if(fabs(dTempB) > Eps)
	{
		dTemp = dTempB;
		iIdA = 0;
		iIdB = 2;
	}
	else if(fabs(dTempC) > Eps)
	{
		dTemp = dTempC;
		iIdA = 1;
		iIdB = 2;
	}

	if(fabs(dTemp) > Eps)
	{
		dLengthPre = -(dMixVec[iIdA]*dK2Vec[iIdB] - dMixVec[iIdB]*dK2Vec[iIdA])/dTemp;

		dLengthPre = tPH_LinePre.dLength - dLengthPre;

		dLegthCur = (dMixVec[iIdA]*dK1Vec[iIdB] - dMixVec[iIdB]*dK1Vec[iIdA])/dTemp;
	}
	else
	{
		//距离目标点的距离
		double dLengthTarget = sqrt(dMixVec[0]*dMixVec[0] + dMixVec[1]*dMixVec[1] + dMixVec[2]*dMixVec[2]);

		//判断方向
		double dVecDir =  dMixVec[0]*dK1Vec[0] + dMixVec[1]*dK1Vec[1] + dMixVec[2]*dK1Vec[2];

		if(dVecDir > Eps)
		{
			dLengthPre = tPH_LinePre.dLength - dLengthTarget;
			dLegthCur = 0;
		}
		else
		{
			dLengthPre = tPH_LinePre.dLength;
			dLegthCur = dLengthTarget;
		}
	}

	double dCPosPre[MaxAxisNum] = {0};
	double dCPosCur[MaxAxisNum] = {0};
	double dCPosMix[MaxAxisNum] = {0};

	GetPosByLength(tPH_LinePre,dLengthPre,dCPosPre);
	GetPosByLength(tPH_Line,dLegthCur,dCPosCur);

	for(int i = 0;i < MaxAxisNum;i++)
	{
		dCPosMix[i] = dCPosPre[i] + dCPosCur[i] - tPH_Line.dSPos[i];

		if(i > 2&&i < 6)
		{
			if(dCPosMix[i] > 180)
				dCPosMix[i] -= 360;
			else if(dCPosMix[i] < -180)
				dCPosMix[i] += 360;
		}
	}

	memcpy(dCPos,dCPosMix,sizeof(double)*MaxAxisNum);

	return iErrorId;
}
/************************************************
函数功能：单段轨迹，基于运动位移量获取姿态以及附加轴位置信息
参    数：
		tPH_Line------规划信息
		dLength-------位移量
		dCPos---------获取空间位置
返 回 值：错误码
*************************************************/
int HS_Int_SLine::GetPosByLength(Para_PreHandle &tPH_Line,double dLength,double dCPos[MaxAxisNum])
{
	int iErrorId = 0;

	double dMPos[5][4] = {0};
	for(int i = 0;i < 3;i++)
	{
		dMPos[i][3] = tPH_Line.dSPos[i] + dLength*tPH_Line.dKXYZ[i];
	}

	double dMoveDis[SpaceAxisNum] = {0};
	if(tPH_Line.dLength > Eps)
	{
		double dKL = dLength/tPH_Line.dLength;
		for(int i = 1;i < SpaceAxisNum;i++)
		{
			dMoveDis[i] = tPH_Line.dSetDis[i]*dKL;
		}
	}

	double dSMPos[4][4] = {0};
	m_HS_Kinematic->HS_CPosToMPos(tPH_Line.dSPos,dSMPos);

	double dQ[4] = {0};
	memcpy(dQ,tPH_Line.dQ,sizeof(double)*4);
	dQ[0] = angle2Rad(dMoveDis[1]);
	double dQMPos[4][4] = {0};	

	HS_Math::Matrix_AToM(dQ,dQMPos);
	HS_Math::Matrix_Multi(4,3,&dSMPos[0][0],&dQMPos[0][0],&dMPos[0][0]);

	m_HS_Kinematic->HS_MPosToCPos(dMPos,dCPos);

	for(int i = 0;i < 3;i++)
	{
		dCPos[6+i] = tPH_Line.dSPos[6+i] + dMoveDis[2+i]; 
	}

	return iErrorId;
}

/************************************************
函数功能：小线段粗插补过程中的速度加速度限制完善
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_SLine::SLAutoPara(Para_PreHandle &tPH_Line)
{
    //求解当前起点位置的关节位置以及空间速度约束
    int iErrorId = 0;
    double dPlanPos[3][SpaceAxisNum] = {0};
    double dTAdd = 0.001;
    //求解关节坐标
    double dJPos[3][MaxAxisNum] = {0};

    double dInitVel[SpaceAxisNum];
    memcpy(dInitVel,tPH_Line.dSetVel,sizeof(double)*SpaceAxisNum);
    tPH_Line.dSetKAcc[0] = 1.0;
    tPH_Line.dSetKAcc[1] = 1.0;

    for(int iCnt = 0;iCnt < 3;iCnt++)
    {
        for(int i = 0;i < 2;i++)
        {
            tPH_Line.dSetVel[i] = dInitVel[i]*tPH_Line.dSetKVel[i];
            //tPH_Line.dSetAcc[i] = m_dCAccPara[i]*tPH_Line.dSetMaxAcc*tPH_Line.dSetKAcc[0];

            dPlanPos[0][i] = tPH_Line.dSetDis[i]/2;
            dPlanPos[1][i] = dPlanPos[0][i] + tPH_Line.dSetVel[i]*dTAdd;
            //dPlanPos[2][i] = dPlanPos[1][i] + (tPH_Line.dSetVel[i] + tPH_Line.dSetAcc[i]*dTAdd)*dTAdd;
        }

        SyncPara tSync;
        tSync.tPreHandle = tPH_Line;
        double dMPos[3][5][4] = {0};
        for(int i = 0;i < 2;i++)
        {
            PlanPos2MPos(tSync,dPlanPos[i],dMPos[i]);
        }
        
        //解析解和数值解具有差异化，数值解可能存在精度误差，导致点位求解精度不足，从而后续速度求解异常，但耗时增大较多，后续需优化完善
        iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[0],m_iToolNum,m_iWorkNum,CP_ToolWork,m_dDivFoundJPos,dJPos[0]);

		iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[0],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[0]);

		//iErrorId = m_HS_Kinematic->HS_MPosToJPos_JXJ(dMPos[0],m_iToolNum,m_iWorkNum,m_eState,CP_ToolWork,dJPos[0]);
        if(iErrorId != 0) return ERROR_CPOSTOJPOS;

        m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[1],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[1]);

        double dJVel[MaxAxisNum] = {0};
        double dJAcc[MaxAxisNum] = {0};
        for(int i = 0;i < 6;i++)
        {
            //dJAcc[i] = (dJPos[0][i] + dJPos[2][i] - 2*dJPos[1][i])/dTAdd/dTAdd;	
            dJVel[i] = (dJPos[1][i] - dJPos[0][i])/dTAdd;
        }

        //比例求解 	
        double dKAcc = 0;
        double dKVel = 0;
        for(int i = 0;i < MaxAxisNum;i++)
        {
			//double dAccTemp = 0;
			//if(fabs(m_dJAccPara[i]) > Eps)
			//	dAccTemp = fabs(dJAcc[i]/m_dJAccPara[i]);
			//double dKAccSet = tPH_Line.dSetMaxAcc;		
			//dAccTemp = dAccTemp/dKAccSet;		
			//dKAcc = Max(dKAcc,dAccTemp);

            double dVelTemp = 0;
            if(fabs(m_dJVelPara[i]) > Eps)
                dVelTemp = fabs(dJVel[i]/m_dJVelPara[i]);
			dVelTemp /= m_dKVelMax;

            dKVel = Max(dKVel,dVelTemp);
        }	

        for(int i = 0;i < 2;i++)
        {
            if(dKVel > Eps)
                tPH_Line.dSetKVel[i] = tPH_Line.dSetKVel[i]/dKVel;	

            if(tPH_Line.dSetKVel[i] > 1.0)
                tPH_Line.dSetKVel[i] = 1.0;
            else if(tPH_Line.dSetKVel[i] < 0.0001)		//避免值太小
                tPH_Line.dSetKVel[i] = 0.0001;
        }
    }

	//memcpy(tPH_Line.dSetJPos[0],dJPos[0],sizeof(double)*MaxAxisNum);

    memcpy(m_dDivFoundJPos,dJPos[1],sizeof(double)*MaxAxisNum);
    return 0;
}
/************************************************
函数功能：速度规划
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_SLine::VelPlan(void)
{
	//执行速度规划
	memset(m_tVelPlanPara,0,sizeof(m_tVelPlanPara));	

    FilterControl tFilterControl = m_tGTrajData[m_iIndex].tMotionData.tFilterControl;

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

    m_bSLPlagFlag = true;

	m_dTCur = m_dCycle + m_dTSmoothOff[m_iGroupNum];
	m_dTSmoothOff[m_iGroupNum] = 0;

	BaseMoveData tBaseMoveData = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum];

	LOG_ALGO("SetVel = %.0lf,%.0lf---%.0lf,%.0lf,%.0lf;SetAcc = %.0lf,%.0lf;Smooth = %d;TSmooth = %.3lf;",\
		tBaseMoveData.dVel,tBaseMoveData.dVort,m_dJVelPara[6],m_dJVelPara[7],m_dJVelPara[8],\
		tBaseMoveData.dAcc,tBaseMoveData.dDec,m_tGTrajData[m_iIndex].tMotionData.iSmooth,m_dTFreProtect);
	LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSyncBase.tPreHandle.dSPos[0],m_tSyncBase.tPreHandle.dSPos[1],m_tSyncBase.tPreHandle.dSPos[2],m_tSyncBase.tPreHandle.dSPos[3],m_tSyncBase.tPreHandle.dSPos[4],m_tSyncBase.tPreHandle.dSPos[5],\
		m_tSyncBase.tPreHandle.dSPos[6],m_tSyncBase.tPreHandle.dSPos[7],m_tSyncBase.tPreHandle.dSPos[8]);
	LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSyncBase.tPreHandle.dEPos[0],m_tSyncBase.tPreHandle.dEPos[1],m_tSyncBase.tPreHandle.dEPos[2],m_tSyncBase.tPreHandle.dEPos[3],m_tSyncBase.tPreHandle.dEPos[4],m_tSyncBase.tPreHandle.dEPos[5],\
		m_tSyncBase.tPreHandle.dEPos[6],m_tSyncBase.tPreHandle.dEPos[7],m_tSyncBase.tPreHandle.dEPos[8]);
	LOG_ALGO("Dis  = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSyncBase.tPreHandle.dDis[0],m_tSyncBase.tPreHandle.dDis[1],m_tSyncBase.tPreHandle.dDis[2],m_tSyncBase.tPreHandle.dDis[3],m_tSyncBase.tPreHandle.dDis[4],m_tSyncBase.tPreHandle.dDis[5],\
		m_tSyncBase.tPreHandle.dDis[6],m_tSyncBase.tPreHandle.dDis[7],m_tSyncBase.tPreHandle.dDis[8]);
	LOG_ALGO("Length= %.3lf,Angle = %.3lf;SmoothCAngle = %.3lf",\
        m_tSyncBase.tPreHandle.dOrigDis[0],m_tSyncBase.tPreHandle.dOrigDis[1],m_tSyncBase.tPreHandle.dSmoothCAngle);
    LOG_ALGO("StopPos = %.3lf, %.3lf;",\
        m_dStopPos[0],m_dStopPos[1]);
	LOG_ALGO("Ratio = %.2lf;KJVel = %.3lf;Cnt = %.3lf,LSmooth = %.3lf,%.3lf",\
		m_dSetRatio,m_tSync.tPreHandle.dRealKJVel,m_tGTrajData[m_iIndex].tMotionData.dCnt,m_tSyncBase.tPreHandle.dDisSmooth[PRE],m_tSyncBase.tPreHandle.dDisSmooth[NEX]);
    LOG_ALGO("Firter :Open = %d,Type = %d,Fre = %.3lf,Grade = %d",\
        tFilterControl.bFilterOpenFlag,tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade);

	//打印规划参数
	LOG_ALGO("RealVel = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tVelPlan.dEVel[0],m_tSync.tVelPlan.dEVel[1],m_tSync.tVelPlan.dEVel[2],m_tSync.tVelPlan.dEVel[3],m_tSync.tVelPlan.dEVel[4]);
	LOG_ALGO("SetKVel = %.3lf,%.3lf--%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSetKVel[0],m_tSync.tPreHandle.dSetKVel[1],m_tSync.tPreHandle.dSetKVel[2],m_tSync.tPreHandle.dSetKVel[3],m_tSync.tPreHandle.dSetKVel[4]);	
    LOG_ALGO("StopAcc = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dStopAcc[0],m_tSync.tVelPlan.dStopAcc[1],m_tSync.tVelPlan.dStopAcc[2],m_tSync.tVelPlan.dStopAcc[3],m_tSync.tVelPlan.dStopAcc[4]);
    LOG_ALGO("StopJerk= %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dStopJerk[0],m_tSync.tVelPlan.dStopJerk[1],m_tSync.tVelPlan.dStopJerk[2],m_tSync.tVelPlan.dStopJerk[3],m_tSync.tVelPlan.dStopJerk[4]);	
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
HS_MStatus HS_Int_SLine::Move(int &iErrorId,bool bLastCycle)
{
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	gettimeofday(&l_beginTime_Auto, NULL);
#endif
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
		if(bLastCycle&&m_bSLLookAheadPlanFlag)
		{
			MoveDynPlan();
		}
        if(m_bSmoothPreFlag)
        {
            for(int i = 0;i < SpaceAxisNum;i++)
            {
                HS_MStatus Status = m_HS_VelPlan_ParaPre[i]->Move(dMovePosPre[i]);

                m_HS_VelPlan_ParaPre[i]->GetVel(m_dMoveVelPre[i]);
                m_HS_VelPlan_ParaPre[i]->GetAcc(m_dMoveAccPre[i]);

                if(Status == M_Done||Status == M_UnInit)
                    iNum_Done++;

                if(Status == M_Error)
                    return M_Error;
            }

            if(iNum_Done == SpaceAxisNum)
            {
                m_bSmoothPreFlag = false;
            }

            for(int i = 0;i < SpaceAxisNum;i++)
            {
                HS_MStatus Status = m_HS_VelPlan_Para[i]->Move(dMovePos[i]);
                m_HS_VelPlan_Para[i]->GetVel(m_dMoveVel[i]);
                m_HS_VelPlan_Para[i]->GetAcc(m_dMoveAcc[i]);
            }

            PlanPos2MPos(m_tSyncPre,m_tSync,dMovePosPre,dMovePos,m_dRMPos);
        }
        else
        {
            for(int i = 0;i < SpaceAxisNum;i++)
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
            if(iNum_Done == SpaceAxisNum)
            {
                //LOG_ALGO("Move Done!");
                eMStatus = M_Done;	
            }

            for(int i = 0;i < SpaceAxisNum;i++)
            {
                dMovePos[i] += m_dStopPos[i];
                dOrignPos[i] += m_dStopPos[i];
            }

            PlanPos2MPos(m_tSync,dMovePos,m_dRMPos);

            //m_dMovePos代表相对于当前运动起点的位移量，提供给减速停止使用
            for(int i = 0;i < MaxAxisNum;i++)
            {
                m_dMovePos[i] = dMovePos[i];
                m_dMoveRatioPos[i] = dOrignPos[i];
            }	
        }

		if(!m_bSmoothNextFlag&&bLastCycle&&eMStatus == M_Done&&m_bSLStopFlag)
		{
			eMStatus = M_StopDone;
			//如果当前段为小线段，则直接运动完成
			if(m_tSync.tPreHandle.bSmallLineFlag)
				eMStatus = M_StopDone_F;
		}

		bool bCheckLastFlag = false;
		if((m_tSync.tPreHandle.eDynState== DYNPLAN_EndSmooth&&bLastCycle)||m_tSync.tPreHandle.eDynState != DYNPLAN_EndSmooth)
			bCheckLastFlag = true;

        //平滑处理
        if(m_bSmoothNextFlag&&bCheckLastFlag)
        {
            if(m_dTCur - (m_dTAll - m_tSync.tVelPlan.dTSmooth[NEX] - m_dCycle) > Eps)
            { 
                m_dTSmoothOff[m_iGroupNum] = m_dTCur - (m_dTAll - m_tSync.tVelPlan.dTSmooth[NEX]);
                //LOG_ALGO("TSmoothOff= %.3lf,TAll = %.3lf,TSmooth = %.3lf",\
                    m_dTSmoothOff[m_iGroupNum],m_dTAll,m_tSync.tVelPlan.dTSmooth[NEX]);

                if(m_tSync.tPreHandle.eDynState == DYNPLAN_EndSmooth)
                {
                    //平滑运行段结束，进入下一段运动
                    m_bSmoothNextFlag = false;
                    eMStatus = M_Done;
					m_bSmoothPreFlag = false;
                }
                else
                {
					if(m_dTSmoothOff[m_iGroupNum] > Eps || m_dTSmoothOff[m_iGroupNum] < -m_dCycle)	
					{
						m_dTSmoothOff[m_iGroupNum] = 0;
					}

                    for(int i = 0;i < SpaceAxisNum;i++)
                    {
                        *m_HS_VelPlan_ParaPre[i] = *m_HS_VelPlan_Para[i];
                    }	
                    m_tSyncPre = m_tSync;
                    m_bSmoothPreFlag = true;	

                    DynPlan();
                    memset(m_tVelPlanPara,0,sizeof(m_tVelPlanPara));
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
                        m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i]);	
                    }
                    eMStatus = M_Busy;
                    m_dTCur = m_dTSmoothOff[m_iGroupNum];
					LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
                        m_tSync.tPreHandle.dSPos[0],m_tSync.tPreHandle.dSPos[1],m_tSync.tPreHandle.dSPos[2],m_tSync.tPreHandle.dSPos[3],m_tSync.tPreHandle.dSPos[4],m_tSync.tPreHandle.dSPos[5],\
                        m_tSync.tPreHandle.dSPos[6],m_tSync.tPreHandle.dSPos[7],m_tSync.tPreHandle.dSPos[8]);
					LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
						m_tSync.tPreHandle.dEPos[0],m_tSync.tPreHandle.dEPos[1],m_tSync.tPreHandle.dEPos[2],m_tSync.tPreHandle.dEPos[3],m_tSync.tPreHandle.dEPos[4],m_tSync.tPreHandle.dEPos[5],\
						m_tSync.tPreHandle.dEPos[6],m_tSync.tPreHandle.dEPos[7],m_tSync.tPreHandle.dEPos[8]);
                    LOG_ALGO("RealVel = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
                        m_tSync.tVelPlan.dEVel[0],m_tSync.tVelPlan.dEVel[1],m_tSync.tVelPlan.dEVel[2],m_tSync.tVelPlan.dEVel[3],m_tSync.tVelPlan.dEVel[4]);
                    LOG_ALGO("TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
                        m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);
                }
            }
        }
    }	
	
	//转化为 坐标处理
	double dLJPos[MaxAxisNum];
	memcpy(dLJPos,m_dRJPos,sizeof(double)*MaxAxisNum);
	iErrorId = m_HS_Kinematic->HS_MPosToJPos(m_dRMPos,CP_ToolWork,dLJPos,m_dRJPos);
	if(iErrorId != 0) 
	{
		return M_Error;
	}
	//附加轴
	for(int i = 0;i < 3;i++)
	{
		m_dRJPos[6+i] = m_dRMPos[4][i];
	}

	m_dTCur += m_dCycle;

    if(fabs(m_tSync.tPreHandle.dSetDis[m_tSync.tPreHandle.iMaxAxis]) > Eps)
        m_dPercent = fabs(dMovePos[m_tSync.tPreHandle.iMaxAxis]/m_tSync.tPreHandle.dSetDis[m_tSync.tPreHandle.iMaxAxis]);

    if(eMStatus == M_Done)
    {
		m_bSLLookAheadPlanFlag = false;
        MoveDoneJPosCorrect();
    }
#ifdef _LINUX_
	gettimeofday(&l_endTime_Auto, NULL);
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
	if(de_us_Auto > m_dCycle*1000*1000/2)
	{
		LOG_ALGO("Time_SLMove = %.3lf\n",de_us_Auto);
	}
#endif
	return eMStatus;				
}
/************************************************
函数功能：停止运动运行
参   数：
返 回 值：运行状态
*************************************************/
HS_MStatus HS_Int_SLine::StopMoveHandle(double dMovePos[MaxAxisNum])
{
	HS_MStatus eMStatus = M_Busy;
	if(m_dTCur < m_dTStop)
	{
		for(int i = 0;i < SpaceAxisNum;i++)
		{
			dMovePos[i] = m_dTCur*m_dTCur*m_dTCur*(m_dStopKa[i]*m_dTCur + m_dStopKb[i]) + 
				m_dTCur*(m_dStopKc[i]*m_dTCur + m_dMoveVel[i]);
		}
	}
	else
	{
		for(int i = 0;i < SpaceAxisNum;i++)
		{
			dMovePos[i] = m_dStopDis[i];
		}
		eMStatus = M_Done;
	}

    for(int i = 0;i < SpaceAxisNum;i++)
    {
        dMovePos[i] += m_dStopSPos[i];
    }

	return eMStatus;
}
/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_SLine::execIntMove(IntData &intdata,int &iErrorId)
{
    if(m_bDynSmoothFlag)
    {
        DynSmoothPlan();
    }

	HS_MStatus eMStatus = M_Busy;
	//LOG_ALGO("------------------------------------------------------------------------");
    for(int i = 0;i < m_iInterMultCnt;i++)
    {
        if(!m_bPlanFlag)
        {
            //存在重复点位，不进行后续的运动，防止报错
            eMStatus = M_Done;
            memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);
            continue;
        }

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
        if(iErrorId != 0)
        {
            eMStatus = M_Error;
            break;
        }
        //LOG_ALGO("InterJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],\
        m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);
        memcpy(intdata.tGJPos[i].dJPos[m_iGroupNum],m_dRJPos,sizeof(double)*MaxAxisNum);
    }
	//LOG_ALGO("------------------------------------------------------------------------");
	return eMStatus;
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_SLine::Stop()
{
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	gettimeofday(&l_beginTime_Auto, NULL);
#endif
	if(m_bSLStopFlag)
	{
		//如果是平滑段的停止，则仅对当前段响应，前一段则不处理
		return 0;
	}

	//对小线段停止的功能处理
	bool bSLStopType = true;
	if(bSLStopType)
	{
		int iStopAhead = m_iLookAhead[m_iGroupNum];
		if(iStopAhead > 10)
			iStopAhead = 10;

		if(iStopAhead == 0)
		{
			m_bSmoothNextFlag = false;
			m_tSync.tVelPlan.dTSmooth[NEX] = 0;
			m_tSync.tPreHandle.eDynState = DYNPLAN_End;
		}
		else
		{
			//通过两步处理，先获取当前减速停止对应的段数，再按照这个段进行减速停止处理
			SyncPara tSync[MAXLOOKAHEADSLINE+1] = {SyncPara()};
			memcpy(tSync,m_tSyncAhead[m_iGroupNum],sizeof(m_tSyncAhead[m_iGroupNum]));

			//速度设定与当前一致
			for(int i = 1;i < iStopAhead;i++)
			{
				m_tSyncAhead[m_iGroupNum][i].tVelPlan.dEVel[0] = m_tSyncAhead[m_iGroupNum][0].tVelPlan.dEVel[0];
			}

			SyncByAcc(m_tSyncAhead[m_iGroupNum][iStopAhead],false,true);
			int iRet = SmoothHandle_Backward(iStopAhead);
			if(iRet < 1)
				iRet = 1;

			//根据后溯停止的运行段，进行当前的减速停止
			iStopAhead = iRet;
			memcpy(m_tSyncAhead[m_iGroupNum],tSync,sizeof(m_tSyncAhead[m_iGroupNum]));
			SyncByAcc(m_tSyncAhead[m_iGroupNum][iStopAhead],false,true);
			SmoothHandle_Backward(iStopAhead);
		}

		//前瞻停止后，不进行后续运动
		m_bSLStopFlag = true;
		m_tSyncAhead[m_iGroupNum][iStopAhead].tVelPlan.dTSmooth[NEX] = 0;

		double dTStop = 0;
		for(int i = 0;i <= iStopAhead;i++)
		{
			dTStop += m_tSyncAhead[m_iGroupNum][i].tVelPlan.dTime[TALL] - m_tSyncAhead[m_iGroupNum][i].tVelPlan.dTSmooth[PRE];
		}

		LOG_ALGO("Small Line Stop Move:LookAheadCnt = %d,StopAhead = %d,Time = %.3lf",m_iLookAhead[m_iGroupNum],iStopAhead,dTStop);
		
		return 1;
	}
	else
	{
		LOG_ALGO("Small Line Stop Move Only Cur:m_iMaxLookAheadLine = %d",m_iMaxLookAheadLine);
		return StopPlan();
	}
#ifdef _LINUX_
	gettimeofday(&l_endTime_Auto, NULL);
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
	if(de_us_Auto > m_dCycle*1000*1000/2)
	{
		LOG_ALGO("Time_Stop = %.3lf\n",de_us_Auto);
	}
#endif
}
/************************************************
函数功能：运动调速
参    数：设定的速度倍率
返 回 值：错误码
*************************************************/
int HS_Int_SLine::setRatio(double dRatio)
{
    if(m_bStopFlag||m_bSLStopFlag)	//执行减速停止，响应调速
        return 0;

	//小线段响应调速的方式，对已经处理的信息重新进行速度的设置以及前瞻后溯规划
	//对后续的运动处理，当前段不进行处理
	int iErrorId = 0;

	if((fabs(dRatio - m_dSetRatio) > 0.01)&&fabs(dRatio) > Eps)
	{		
		//当前段与下一段的提速处理未增加，会有延迟
		if(m_iLookAhead[m_iGroupNum] > 2)
		{	
			//按照新的倍率获取设置参数后重新进行规划
			//调速时：加速过程需将加减速度增大
			//减速过程要考虑减速的比例，需要跨多段减速
			for(int iLookAhead = 1;iLookAhead <= m_iLookAhead[m_iGroupNum];iLookAhead++)
			{			
				for(int i = 0;i < SpaceAxisNum;i++)
				{
					m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetVel[i] *= dRatio/m_dSetRatio;
					m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetAcc[i] *= dRatio/m_dSetRatio;
					m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle.dSetDec[i] *= dRatio/m_dSetRatio;
				}
			}

			for(int iLookAhead = 1;iLookAhead < m_iLookAhead[m_iGroupNum];iLookAhead++)
			{	
				iErrorId = SmoothHandle_Forward(iLookAhead);
			}

			if(m_tSyncAhead[m_iGroupNum][m_iLookAhead[m_iGroupNum]].tPreHandle.eDynState == DYNPLAN_End)
			{
				SyncByAcc(m_tSyncAhead[m_iGroupNum][m_iLookAhead[m_iGroupNum]],false,true); 
				iErrorId = SmoothHandle_Backward(m_iLookAhead[m_iGroupNum]);
			}
		}

		LOG_ALGO("Ratio OK:%.3lf To %.3lf;m_iLookAhead[m_iGroupNum] = %d",m_dSetRatio,dRatio,m_iLookAhead[m_iGroupNum]);
		m_dSetRatio = dRatio;
	}
	return 0;

	//return Ratio(dRatio);
}

/************************************************
函数功能：小线段模式的前瞻加速规划处理
参    数：iLookAhead---当前规划行
         bStart-------是否为启动段
返 回 值：错误码
*************************************************/
int HS_Int_SLine::SmoothHandle_Forward(int iLookAhead,bool bStart)
{
    SyncPara tSyncCur = m_tSyncAhead[m_iGroupNum][iLookAhead];
    SyncPara tSyncNex = m_tSyncAhead[m_iGroupNum][iLookAhead+1];

    if(bStart)      //启动段处理
    {
        //前段的加速度比例值
        double dMaxKAcc = 0;
        for(int i = 0;i < 2;i++)
        {
            double dAcc = tSyncCur.tVelPlan.dEVel[i]/tSyncCur.tVelPlan.dTime[TACC]*1.5;
            double dKAcc = dAcc/tSyncCur.tPreHandle.dSetAcc[i];
            dMaxKAcc = Max(dKAcc,dMaxKAcc);
        }

        double dKAccLimit = dMaxKAcc*2;
        if(dKAccLimit > 1.0)
            dKAccLimit = 1.0;
        double dTMin = GetConVTime(tSyncNex);
        //下一段匀速
        SyncByTime(tSyncCur.tVelPlan.dTime[TACC],dTMin,tSyncCur);
        SyncByTime(dTMin,dTMin,tSyncNex);

        double dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);
        if(dKAcc > dKAccLimit)
        {
            //调整时间，同时进行了约束，约束完，下一段不进行调整
            double dTMax = tSyncCur.tVelPlan.dTime[TACC];
            double dTSmooth = S2STSmoothCalc_Up(tSyncCur,tSyncNex,dTMin,dTMax,dKAccLimit,false,true);   
        }
    }
    else
    {
        double dTSmooth = tSyncCur.tVelPlan.dTime[TACC];
        SyncByTime(dTSmooth,dTSmooth,tSyncCur);
        SyncByTime(dTSmooth,dTSmooth,tSyncNex);
        double dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);
        if(dKAcc < 1.0)
        {
            //判断当前段与前一段是否已经满足限制&如果已经进行提速，则下一段再进行提速处理
            if(!tSyncCur.tVelPlan.bSpeedUpOK)
            {
                //处理与前一段以及与下一段的综合调整
                double dTMin = GetConVTime(tSyncNex);
                if(dTSmooth > dTMin)
                {
                    //需要逐步提速至匀速
                    SyncByTime(tSyncCur.tVelPlan.dTime[TACC],dTMin,tSyncCur);
                    SyncByTime(dTMin,dTMin,tSyncNex);

                    SyncPara tSyncPre = m_tSyncAhead[m_iGroupNum][iLookAhead-1];

                    double dKAccLimit = 1.0;
                    if(tSyncPre.tVelPlan.dKSmoothAcc > Eps)
                    {
                        //有需求才进行限制
                        dKAccLimit = tSyncPre.tVelPlan.dKSmoothAcc*4;
                        if(dKAccLimit > 1.0)
                            dKAccLimit = 1.0;
                    }

                    double dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);         
                    if(dKAcc > dKAccLimit)
                    {
                        //调整时间，同时进行了约束，约束完，下一段不进行调整
                        double dTMax = tSyncCur.tVelPlan.dTime[TACC];
                        double dTSmooth = S2STSmoothCalc_Up(tSyncCur,tSyncNex,dTMin,dTMax,dKAccLimit);   
                    }
                    else
                    {
                        //直接提速至匀速
                    }
                }
                else
                {
                    //当前时间已经能够保证下一段匀速，同样也无需增大时间，可能会导致当前段降速
                    double dTMinCur = GetConVTime(tSyncCur);
                    if(dTMinCur > dTSmooth&&dTMinCur < dTMin)
                    {
                        //满足前后两段匀速的时间可以适当提升，保留时间空间
                        SyncByTime(dTSmooth,dTMinCur,tSyncCur);
                        SyncByTime(dTMinCur,dTMinCur,tSyncNex);
                    }
                }
            }
        }
        else
        {
			const double dKAccLimit = 2.0;
            if(dKAcc > dKAccLimit)
            {
                //进行合理的约束降速处理
                bool bBackFlag = true;
                //当前段是否已达到匀速段进行不同的处理

                SyncByAcc(tSyncNex,true,true);
                double dTMax = tSyncNex.tVelPlan.dTime[TACC];
                if(dTMax < m_dTFreProtect)
                    dTMax = m_dTFreProtect;
                double dKLimit = 1.0;
                //先对超出段进行降速等优化处理
                double dTLimit = S2STSmoothCalc_Down(tSyncCur,tSyncNex,dTMax,dKLimit);     
                m_tSyncAhead[m_iGroupNum][iLookAhead+1] = tSyncNex;
                SmoothHandle_Backward(iLookAhead+1);
				LOG_ALGO("ForWard Plan OverAcc!KAcc = %.3lf",dKAcc);
                return 0;
            }
        }
    }

    m_tSyncAhead[m_iGroupNum][iLookAhead] = tSyncCur;
    m_tSyncAhead[m_iGroupNum][iLookAhead+1] = tSyncNex;
    double dTSmooth = m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TDEC];
    m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTSmooth[NEX] = dTSmooth;
    m_tSyncAhead[m_iGroupNum][iLookAhead+1].tVelPlan.dTSmooth[PRE] = dTSmooth;
    m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.bSLinePlan = true;
    return 0;
}
/************************************************
函数功能：小线段模式的后溯加速规划处理，并与正向运动进行拼接，完成规划修改
参    数：iLookAhead---当前规划行
返 回 值：回溯的个数
*************************************************/
int HS_Int_SLine::SmoothHandle_Backward(int iLookAhead)
{
    int iBackward = 0;  
    const double dBackKAccLimit = 1.0;    
    while(iBackward < iLookAhead)
    {
        SyncPara tSyncCur = m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward];
        SyncPara tSyncNex = m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1];
        if(iBackward == 0)
        {
            //前段的加速度比例值
            double dMaxKAcc = 0;
            for(int i = 0;i < 2;i++)
            {
                double dAcc = tSyncCur.tVelPlan.dEVel[i]/tSyncCur.tVelPlan.dTime[TDEC]*1.5;
                double dKAcc = dAcc/tSyncCur.tPreHandle.dSetDec[i];
                dMaxKAcc = Max(dKAcc,dMaxKAcc);
            }

            double dKAccLimit = dMaxKAcc*2;
            if(dKAccLimit > dBackKAccLimit)
                dKAccLimit = dBackKAccLimit;
            double dTMin = GetConVTime(tSyncNex);
            //下一段匀速
            SyncByTime(dTMin,tSyncCur.tVelPlan.dTime[TDEC],tSyncCur);
            SyncByTime(dTMin,dTMin,tSyncNex);

            double dKAcc = MaxKAccCalc(tSyncNex,tSyncCur);
            if(dKAcc > dKAccLimit)
            {
                //调整时间，同时进行了约束，约束完，下一段不进行调整
                double dTMax = tSyncCur.tVelPlan.dTime[TDEC];
                double dTSmooth = S2STSmoothCalc_Up(tSyncCur,tSyncNex,dTMin,dTMax,dKAccLimit,true,true);   
            }
        }
        else
        {
            double dTSmooth = tSyncCur.tVelPlan.dTime[TACC];
            SyncByTime(dTSmooth,dTSmooth,tSyncCur);
            SyncByTime(dTSmooth,dTSmooth,tSyncNex);
            double dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);
            if(dKAcc < dBackKAccLimit)
            {
                //判断当前段与前一段是否已经满足限制&如果已经进行提速，则下一段再进行提速处理
                if(!tSyncCur.tVelPlan.bSpeedUpOK)
                {
                    //处理与前一段以及与下一段的综合调整
                    double dTMin = GetConVTime(tSyncNex);
                    if(dTSmooth > dTMin)
                    {
                        //需要逐步提速至匀速
                        SyncByTime(dTMin,tSyncCur.tVelPlan.dTime[TDEC],tSyncCur);
                        SyncByTime(dTMin,dTMin,tSyncNex);

                        SyncPara tSyncPre = m_tSyncAhead[m_iGroupNum][iLookAhead-1];

                        double dKAccLimit = dBackKAccLimit;
                        if(tSyncPre.tVelPlan.dKSmoothAcc > Eps)
                        {
                            //有需求才进行限制
                            dKAccLimit = tSyncPre.tVelPlan.dKSmoothAcc*4;
                            if(dKAccLimit > dBackKAccLimit)
                                dKAccLimit = dBackKAccLimit;
                        }

                        double dKAcc = MaxKAccCalc(tSyncNex,tSyncCur);         
                        if(dKAcc > dKAccLimit)
                        {
                            //调整时间，同时进行了约束，约束完，下一段不进行调整
                            double dTMax = tSyncCur.tVelPlan.dTime[TDEC];
                            double dTSmooth = S2STSmoothCalc_Up(tSyncCur,tSyncNex,dTMin,dTMax,dKAccLimit,true);   
                        }
                        else
                        {
                            //直接提速至匀速
                        }
                    }
                    else
                    {
                        //当前时间已经能够保证下一段匀速，同样也无需增大时间，可能会导致当前段降速
                        double dTMinCur = GetConVTime(tSyncCur);
                        if(dTMinCur > dTSmooth&&dTMinCur < dTMin)
                        {
                            //满足前后两段匀速的时间可以适当提升，保留时间空间
                            SyncByTime(dTMinCur,dTSmooth,tSyncCur);
                            SyncByTime(dTMinCur,dTMinCur,tSyncNex);
                        }
                    }
                }
            }
        }
		
		/***********************************************************************************************
		后溯成功条件判断：
		tSyncCur为由后向前的后溯行【已完成】，例如前瞻第10行
		tSyncNex为后溯的下一行：例如前瞻第9行
		后溯成功条件为，tSyncNex与原前瞻的速度信息tSyncNexOrgin进行对比
		如果后溯的速度比前瞻速度更大或者相等【前瞻为匀速段等】，此时判定后溯成功，正常衔接
		后溯的规划速度为V1，时间对应为A1，D1；前瞻对应的速度为V2，时间对应为A2，D2
		新的段规划速度为V3，时间为A3，D3
		>>正常处理，用A3 = A2【与原前段前瞻匹配】，D3 = D1【与新后溯段匹配】，则V2 <= V3 <= V1
		>>对比第8行的运动信息，如果第8第9行是匀速或者减速，由于增加了第9行的速度，则对加速度影响不大
		>>如果第8行到第9行也是加速段，则由于增加了第9行的速度，会是的8和9的加速度增加甚至超加速等，故而需要进行一定的优化处理
		************************************************************************************************/
        //正向拼接，通过速度进行判定
        SyncPara tSyncNexOrgin = m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1];
        if(tSyncNex.tVelPlan.dEVel[0] - tSyncNexOrgin.tVelPlan.dEVel[0] > -Eps ||iBackward == iLookAhead - 1)
        {            
            //后溯完成
            SyncByTime(tSyncNexOrgin.tVelPlan.dTime[TACC],tSyncNex.tVelPlan.dTime[TDEC],tSyncNex);
            m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward] = tSyncCur;
            m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1] = tSyncNex;
            double dTSmooth = tSyncCur.tVelPlan.dTime[TACC];
            m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward].tVelPlan.dTSmooth[PRE] = dTSmooth;
            m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1].tVelPlan.dTSmooth[NEX] = dTSmooth;
            break;
        }
		else
		{
			int iIdPre = iLookAhead - iBackward - 2;
			if(iIdPre > 0)
			{
				SyncPara tSyncPreOrgin = m_tSyncAhead[m_iGroupNum][iIdPre];
				if(tSyncNex.tVelPlan.dEVel[0] - tSyncPreOrgin.tVelPlan.dEVel[0] > -Eps)
				{
					SyncByTime(tSyncNexOrgin.tVelPlan.dTime[TACC],tSyncNex.tVelPlan.dTime[TDEC],tSyncNex);
					m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward] = tSyncCur;
					m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1] = tSyncNex;
					double dTSmooth = tSyncCur.tVelPlan.dTime[TACC];
					m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward].tVelPlan.dTSmooth[PRE] = dTSmooth;
					m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1].tVelPlan.dTSmooth[NEX] = dTSmooth;
					break;
				}
			}
		}

        m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward] = tSyncCur;
        m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1] = tSyncNex;
        double dTSmooth = tSyncCur.tVelPlan.dTime[TACC];
        m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward].tVelPlan.dTSmooth[PRE] = dTSmooth;
        m_tSyncAhead[m_iGroupNum][iLookAhead - iBackward - 1].tVelPlan.dTSmooth[NEX] = dTSmooth;

        iBackward++;
    }
    return iBackward;
}

/***********************************************
函数功能：获取满足匀速的最大时间
参    数：
返 回 值：时间
*************************************************/
double HS_Int_SLine::GetConVTime(SyncPara &tSyncCur)
{
    //先求解下一段运动如果要匀速运动对应的时间
    double dVConTMin[2] = {0};
    for(int i = 0;i < 2;i++)
    {
        if(tSyncCur.tPreHandle.dSetVel[i] > Eps)
            dVConTMin[i] = tSyncCur.tPreHandle.dSetDis[i]/tSyncCur.tPreHandle.dSetVel[i];
    }
    double dTime = Max(dVConTMin[0],dVConTMin[1]);
    return dTime;
}

/***********************************************
函数功能：前后两段的加速度比例求解
参    数：
返 回 值：比例值
*************************************************/
double HS_Int_SLine::MaxKAccCalc(SyncPara &tSyncCur,SyncPara &tSyncNex)
{
    double dSmoothKAcc = 0;
	
    double dTSmooth = tSyncCur.tVelPlan.dTime[TDEC];

//#define SIMPLECALC
	
#ifdef SIMPLECALC
	
    double dEVelCur = tSyncCur.tVelPlan.dEVel[0];
    double dEVelNex = tSyncNex.tVelPlan.dEVel[0];

	double dCAccLimit[2] = {12000,3000};

	dCAccLimit[0] = tSyncCur.tPreHandle.dSetDec[0];
	dCAccLimit[1] = tSyncCur.tPreHandle.dSetDec[1];

    double dDetaV = (dEVelNex*tSyncNex.tPreHandle.dKXYZ[0] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[0])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[0] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[0]) +
        (dEVelNex*tSyncNex.tPreHandle.dKXYZ[1] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[1])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[1] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[1]) +
        (dEVelNex*tSyncNex.tPreHandle.dKXYZ[2] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[2])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[2] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[2]);
    dDetaV = sqrt(dDetaV);   
    double dCAcc = dDetaV/dTSmooth*1.5;
    double dKAcc = dCAcc/dCAccLimit[0];

    dSmoothKAcc = dKAcc;
    dCAcc = fabs(tSyncCur.tVelPlan.dEVel[1] - tSyncNex.tVelPlan.dEVel[1])/dTSmooth*1.5;
    dKAcc = dCAcc/dCAccLimit[1];

    dSmoothKAcc = Max(dSmoothKAcc,dKAcc);
	
#else	

	//double dTinPre = tSyncCur.tVelPlan.dTime[TALL] - dTSmooth/2;
	//double dTinNext = dTSmooth/2;		
	//double dKJVel = 0;
	//double dSmoothKAccT = 0;
	//SmoothMaxJParaCalc(tSyncCur,tSyncNex,dTinPre,dTinNext,dKJVel,dSmoothKAccT,true);
	//return dSmoothKAccT;

	double dCAcc[2] = {0};
	SLCAccCalc(tSyncCur,tSyncNex,dCAcc);

	double dCAccMax = dCAcc[0];
	if(dCAcc[0] < dCAcc[1])
		dCAccMax = dCAcc[1];
	
	double dCAccLimit = 100;
	if(dCAccMax < 100)
		return 0.5;

	if(dCAccMax > tSyncCur.tPreHandle.dSetDec[0]*10.0)
		return 3.0;


	//if(tSyncCur.tPreHandle.dSetDec[6] > Eps)
	//{
	//	dSmoothKAcc = dCAccMax/tSyncCur.tPreHandle.dSetDec[6];
	//}
	//else
	{
		//严格求解加速度比例的方式，耗时大
		double dTinPre = tSyncCur.tVelPlan.dTime[TALL] - dTSmooth/2;
		double dTinNext = dTSmooth/2;		
		//double dKJVel = 0;
		//double dSmoothKAccT = 1.0;
		//SmoothMaxJParaCalc(tSyncCur,tSyncNex,dTinPre,dTinNext,dKJVel,dSmoothKAccT,true);

		//简化计算过程
		SLMaxJAccCalc(tSyncCur,tSyncNex,dTinPre,dTinNext,dSmoothKAcc);
		//dSmoothKAcc = dSmoothKAccT;

		//缓存平滑段的加速度比例约束值用于简化计算
		double dKLimit = 0.2;
		if(dSmoothKAcc > dKLimit)
		{
			tSyncCur.tPreHandle.dSetDec[6] = dCAccMax/dSmoothKAcc;
		}
	}
	//dSmoothKAcc = dSmoothKAccT;
#endif

    //基于时间与加速度的补偿处理，对于平滑时间较小的情况，对比插补周期，其最大加速度会达不到，后续也会有滤波优化，故而进行一定的补偿
    double dKComp = 1.0;
	double dT[3] = {m_dCycle,m_dCycle*2,m_dCycle*3};
	//double dK[3] = {2.0,1.4,1.0};
	double dK[3] = {1.2,1.1,1.0};					//修改补偿量，降低影响			
	for(int i = 0;i < 3;i++)
	{
		if(dTSmooth < dT[i])
		{
			dKComp = (dK[i-1] - dK[i])/(dT[i-1] - dT[i])*(dTSmooth - dT[i]) + dK[i];
			break;
		}
	}
    dSmoothKAcc /= dKComp;    
    return dSmoothKAcc;
}
/************************************************
函数功能：空间加速度计算，包含位置和姿态【小线段专用】
参    数：tSyncCur---当前段规划信息
        tSyncNex---下一段规划信息
        dCAcc[2]-----空间加速度值【位置】【姿态】
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::SLCAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dCAcc[2])
{
	int iErrorId = 0;

	double dTSmooth = tSyncCur.tVelPlan.dTime[TDEC];
	double dEVelCur = tSyncCur.tVelPlan.dEVel[0];
	double dEVelNex = tSyncNex.tVelPlan.dEVel[0];

	double dDetaV = (dEVelNex*tSyncNex.tPreHandle.dKXYZ[0] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[0])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[0] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[0]) +
		(dEVelNex*tSyncNex.tPreHandle.dKXYZ[1] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[1])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[1] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[1]) +
		(dEVelNex*tSyncNex.tPreHandle.dKXYZ[2] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[2])*(dEVelNex*tSyncNex.tPreHandle.dKXYZ[2] - dEVelCur*tSyncCur.tPreHandle.dKXYZ[2]);
	dDetaV = sqrt(dDetaV);   
	dCAcc[0] = dDetaV/dTSmooth*1.5;

	dCAcc[1] = fabs(tSyncCur.tVelPlan.dEVel[1] - tSyncNex.tVelPlan.dEVel[1])/dTSmooth*1.5;

	return iErrorId;
}
/************************************************
函数功能：根据时间求解得到对应的时刻的关节速度、加速度比例信息【小线段专用】
参    数：tSyncCur---当前段规划信息
        tSyncNex---下一段规划信息
        dTInPre----待求时刻值(当前段）
        dTInNext---待求时刻值（下一段）
        dKJVel-----关节速度比例
返 回 值：错误ID
*************************************************/
int HS_Int_SLine::SLMaxJAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJAcc)
{
	int iErrorId = 0;

	double dPLanPre[3][SpaceAxisNum] = {0};	
	double dPLanNex[3][SpaceAxisNum] = {0};	
	double dTAdd = 0.001;
	double dTPre = dTInPre - dTAdd;
	double dTNex = dTInNext - dTAdd;

	for(int i = 0;i < 3;i++)
	{
		GetPlanPos(tSyncCur,dTPre,dPLanPre[i]);
		dTPre += dTAdd;
		GetPlanPos(tSyncNex,dTNex,dPLanNex[i]);
		dTNex += dTAdd;
	}

	double dMPos[3][5][4] = {0};
	for(int i = 0;i < 3;i++)	
		PlanPos2MPos(tSyncCur,tSyncNex,dPLanPre[i],dPLanNex[i],dMPos[i]);	

	double dJPos[3][MaxAxisNum] = {0};
	double dSJPos[MaxAxisNum] = {0};

	memcpy(dSJPos,tSyncNex.tPreHandle.dSetJPos[0],sizeof(double)*6);

	//iErrorId = m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[0],m_iToolNum,m_iWorkNum,CP_ToolWork,dSJPos,dJPos[0]);
	//m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[0],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[0]);

	iErrorId = m_HS_Kinematic->HS_MPosToJPos_JXJ(dMPos[0],m_iToolNum,m_iWorkNum,m_eState,CP_ToolWork,dJPos[0]);
	dJPos[0][6] = dMPos[0][4][0]; dJPos[0][7] = dMPos[0][4][1]; dJPos[0][8] = dMPos[0][4][2];

	if(iErrorId != 0) 
	{
		LOG_ALGO("Error C2J!,CPos = %.3lf,%.3lf,%.3lf",dMPos[0][0][3],dMPos[0][1][3],dMPos[0][2][3]);
		return ERROR_CPOSTOJPOS;
	}

	m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[1],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[0],dJPos[1]);
	dJPos[1][6] = dMPos[1][4][0]; dJPos[1][7] = dMPos[1][4][1]; dJPos[1][8] = dMPos[1][4][2];

	m_HS_Kinematic->HS_MPosToJPos_LJ(dMPos[2],m_iToolNum,m_iWorkNum,CP_ToolWork,dJPos[1],dJPos[2]);	
	dJPos[2][6] = dMPos[2][4][0]; dJPos[2][7] = dMPos[2][4][1]; dJPos[2][8] = dMPos[2][4][2];

	double dJAcc[MaxAxisNum] = {0};
	double dSetMaxAcc = tSyncCur.tPreHandle.dSetMaxAcc;
	for(int i = 0;i < MaxAxisNum;i++)
	{
		dJAcc[i] = (dJPos[0][i] + dJPos[2][i] - 2*dJPos[1][i])/dTAdd/dTAdd;		

		double dTemp = 0;
		if(m_dJAccPara[i] > Eps)
			dTemp = fabs(dJAcc[i]/m_dJAccPara[i]);
		double dKDec = dSetMaxAcc;		
		
		dTemp = dTemp/dKDec;
		dKJAcc = Max(dKJAcc,dTemp);
	}

	return iErrorId;
}
/************************************************
函数功能：小线段模式下对平滑段时间的快速求解
参    数：tSync---规划信息 
         bBack---是否回溯规划
		 bStart--是否启动段，启动段需要进行约束优化
返 回 值：求解得到的新的平滑时间
*************************************************/
double HS_Int_SLine::S2STSmoothCalc_Up(SyncPara &tSyncCur,SyncPara &tSyncNex,double dTMin,double dTMax,double dKLimit,bool bBack,bool bStart)
{
	//启停段，减小停止段的时间增加速度时会同时增加加速段的加速度值，需要进行约束
	//公式求解，减速段的时间比例为K，待求解的加速段的比例为y，则满足公式：
	// y^2 + K*y = 2;即可求解
	double dStartTAcc = tSyncCur.tVelPlan.dTime[TACC];
	double dStartTDec = dStartTAcc;
	if(bBack)
	{
		dStartTAcc = tSyncCur.tVelPlan.dTime[TDEC];
		dStartTDec = dStartTAcc;
	}

    double dTCalc = 0;    
    double dKAcc = 0;
	double dTAccNew = dStartTAcc;
    for(int i = 0;i < 8;i++)
    {
        dTCalc = (dTMax + dTMin)/2;
        if(bBack)
        {
			if(bStart)
			{
				double dK = dTCalc/dStartTDec;
				double dY = (-dK + sqrt(dK*dK + 8))/2;

				dTAccNew = dY*dStartTAcc;
			}

            SyncByTime(dTCalc,dTAccNew,tSyncCur);
            SyncByTime(dTCalc,dTCalc,tSyncNex);
            dKAcc = MaxKAccCalc(tSyncNex,tSyncCur);
        }
        else
        {
			if(bStart)
			{
				double dK = dTCalc/dStartTDec;
				double dY = (-dK + sqrt(dK*dK + 8))/2;
				
				dTAccNew = dY*dStartTAcc;
			}
            SyncByTime(dTAccNew,dTCalc,tSyncCur);
            SyncByTime(dTCalc,dTCalc,tSyncNex);
            dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);
        }

        if(dKAcc > dKLimit)
            dTMin = dTCalc;
        else
            dTMax = dTCalc;
    }
    
    tSyncNex.tVelPlan.bSpeedUpOK = true;
    tSyncNex.tVelPlan.dKSmoothAcc = dKAcc;
    
    return dTCalc;
}

/************************************************
函数功能：小线段模式下对平滑段时间的快速求解
参    数：tSync---规划信息 
返 回 值：求解得到的新的平滑时间
*************************************************/
double HS_Int_SLine::S2STSmoothCalc_Down(SyncPara &tSyncCur,SyncPara &tSyncNex,double dTMax,double dKLimit)
{
    double dTCalc = 0;
    double dTMin = tSyncCur.tVelPlan.dTime[TDEC];

    for(int i = 0;i < 5;i++)
    {
        dTCalc = (dTMax + dTMin)/2;
        SyncByTime(dTCalc,dTCalc,tSyncCur);
        SyncByTime(dTCalc,dTCalc,tSyncNex);
        double dKAcc = MaxKAccCalc(tSyncCur,tSyncNex);   

        if(dKAcc > dKLimit)
            dTMin = dTCalc;
        else
            dTMax = dTCalc;
    }
    return dTCalc;
}