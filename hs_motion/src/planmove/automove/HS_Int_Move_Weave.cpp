/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Move.h
* 摘    要：摆焊处理相关

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*			
*/
#include "HS_Int_Move.h"
#include "HS_Kinematics.h"

bool HS_Int_Move::m_bWeaveFlag = false;						
double HS_Int_Move::m_dWeavePosSave[3] = {0};				
double HS_Int_Move::m_dWeaveAmp = 0;						
double HS_Int_Move::m_dWeaveFreq = 0;						
double HS_Int_Move::m_dWeavePeriod = 0;					
double HS_Int_Move::m_dWeaveAccMAX = 0;
double HS_Int_Move::m_dWeaveVel = 0;						
double HS_Int_Move::m_dWeaveSVel = 0;	
bool HS_Int_Move::m_bSinglePlan = false;						
double HS_Int_Move::m_dWeaveTOff = 0;						

double HS_Int_Move::m_dWeaveTCur = 0;						
double HS_Int_Move::m_dWeaveKa = 0;
double HS_Int_Move::m_dWeaveKb = 0;
double HS_Int_Move::m_dWeaveKc = 0;
double HS_Int_Move::m_dWeaveStopSVel = 0;
double HS_Int_Move::m_dWeaveTStop = 0;
bool HS_Int_Move::m_bWeaveStopDoneFlag = false;
double HS_Int_Move::m_dWeaveTAll = 0;
double HS_Int_Move::m_dWeavePAll = 0;
double HS_Int_Move::m_dWeaveSPos = 0;
WEAVESTATE HS_Int_Move::m_eWeaveState;
bool HS_Int_Move::m_bWeaveStopFlag = false;
double HS_Int_Move::m_dWeaveStopAmp = 0;

double HS_Int_Move::m_dAheadTAll = 0;						
double HS_Int_Move::m_dMCoord[3][3] = {0};                  

int HS_Int_Move::m_WeaveType = 0;                         
double HS_Int_Move::m_dMainSavePos[2][MaxAxisNum] = {0};	
HS_WeavePara HS_Int_Move::m_tHS_WeavePara = HS_WeavePara();

bool HS_Int_Move::m_bDStopMode = false;	
bool HS_Int_Move::m_bWeaveStopDir = false;
bool HS_Int_Move::m_bWeaveWaitFlag = false;							
double HS_Int_Move::m_dWeaveWaitTime = 0;
double HS_Int_Move::m_dWeaveCicleVel = 0;
double HS_Int_Move::m_dWeaveCicleTAcc = 0;
double HS_Int_Move::m_dWeaveCiclePosAcc = 0;
double HS_Int_Move::m_dWeaveCicleTSCon = 0;
double HS_Int_Move::m_dWeaveEuler[3] = {0};			
bool HS_Int_Move::m_bWeaveFilterFlag = false;
bool HS_Int_Move::m_bStopSineNextFlag = false;
double HS_Int_Move::m_dCircleBaseVel = 0;
bool HS_Int_Move::m_bStopSineWaitTFlag = false;
bool HS_Int_Move::m_bStopWeaveMainWaitFlag = false;
int  HS_Int_Move::m_iWeaveNum = 0;

/************************************************
函数功能：检测当前段运动是否存在摆焊运动，用以清除与关节或者非摆焊的平滑
参    数：
		 tTrajData-----规划参数
返 回 值：true----------摆焊运动
*************************************************/
bool HS_Int_Move::WeaveMoveCheck(GroupTrajData tTrajData)
{
	bool bRet = false;

	BaseMoveData tMoveData = tTrajData.tMotionData.tBaseMoveData[m_iGroupNum];

	if((tMoveData.eTrajType == MP_Line||tMoveData.eTrajType == MP_Arc)&&tMoveData.tWeavePara.bWeaveEn)
		bRet = true;

	return bRet;
}
/************************************************
函数功能：检测是否为摆焊停止模式
		【240729】逐步增加三角摆焊的平滑处理方式
参    数：
返 回 值：true---端点摆焊停止模式
*************************************************/
bool HS_Int_Move::CheckStopWeaveType()
{
		
	HS_WeavePara tWeavePara = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum].tWeavePara;

	if(tWeavePara.bWeaveEn && fabs(tWeavePara.dAmplitude) > Eps
		&&(tWeavePara.eWeaveType == WEAVE_SINE||tWeavePara.eWeaveType == WEAVE_L||tWeavePara.eWeaveType == WEAVE_MOONB||tWeavePara.eWeaveType == WEAVE_MOONF)
		&&tWeavePara.eWeaveDType == WEAVE_DSTOP
		&&(tWeavePara.dLDT > m_dCycle||tWeavePara.dRDT > m_dCycle)) 
	{
		return true;
	}

	//当前段是L摆焊，端点不停留，但是不支持与圆弧平滑
	int iBuffNext = (m_iIndex + 1)%MaxBuffSize;

	if(tWeavePara.bWeaveEn&&m_tHS_WeavePara.eWeaveType == WEAVE_L&&
		m_tGTrajData[iBuffNext].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType == MP_Arc&&
		m_tGTrajData[iBuffNext].tMotionData.iLineNum != -1)
	{
		return true;
	}

	return false;
}

/************************************************
函数功能：关闭摆焊功能
参    数：		
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveClose()
{
	m_bWeaveFlag = false;
	m_iWeaveNum = 0;
	m_bWeaveStopDir = false;
	return 0;
}

/************************************************
函数功能：运动规划的起点点位检测以及重新规划
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Move::WeaveMovePlan()
{
	int iErrorId = 0;
	double dAheadTAll = m_tSync.tVelPlan.dTime[TALL];

	WeaveStartHandle(dAheadTAll,m_dRJPos,m_tSync.tPreHandle.dSPos,m_tSync.tPreHandle.dEPos);

	return iErrorId;
}
/************************************************
函数功能：摆焊启动功能
参    数：
		dTWeaveAhead----摆焊执行的总时间
		dInitJPos-------当前位置
		dSPos-----------直线起点位置【L摆焊使用】
		dEPos-----------直线结束点位置【L摆焊使用】
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveStartHandle(double dTWeaveAhead,double *dInitJPos,double *dSPos,double *dEPos)
{
	m_dAheadTAll = dTWeaveAhead;

	HS_WeavePara tWeavePara = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum].tWeavePara;

	//无摆焊标识符，不做摆焊处理
	if(!tWeavePara.bWeaveEn)
		m_bWeaveFlag = false;
	else
	{
		if(tWeavePara.eWeaveDType < 0||tWeavePara.eWeaveDType > 1)
			tWeavePara.eWeaveDType = WEAVE_DMOVE;
	}

	//修改：中断暂停重启要将m_bWeaveFlag置false
	//if(fabs(m_pMCurBase[m_iBuffP].dParaC - 1.0) < Eps)
	//{
	//	m_bWeaveFlag = false;
	//	LOG_ALGO("Weave Stop OFF,m_bWeaveFlag = %d",m_bWeaveFlag);			
	//}

	//m_HS_MotionParaLib->HS_SetWeaveState(false,0);

	//摆焊处理
	if(!m_bWeaveFlag&&tWeavePara.bWeaveEn && fabs(tWeavePara.dAmplitude) > Eps
		&&tWeavePara.dFrequency > Eps) 
	{
		m_bWeaveFlag = true;
		m_dWeaveAmp = tWeavePara.dAmplitude;
		m_dWeaveFreq = tWeavePara.dFrequency;

		//摆焊方向的选择
		m_WeaveType = tWeavePara.eWeaveType;
		m_dWeavePeriod = 1/m_dWeaveFreq;
		m_tHS_WeavePara = tWeavePara;

		m_dWeaveVel = 32*m_dWeaveAmp/5/m_dWeavePeriod;
		m_dWeaveTOff = 0;
		//新模式下的最大加速度
		m_dWeaveAccMAX = 192*m_dWeaveAmp/5/m_dWeavePeriod/m_dWeavePeriod;

		m_bDStopMode = false;
		m_dWeavePAll = 0;

		if((m_tHS_WeavePara.eWeaveType == WEAVE_SINE&&(m_tHS_WeavePara.dLDT > m_dCycle||m_tHS_WeavePara.dRDT > m_dCycle))
			||m_tHS_WeavePara.eWeaveType == WEAVE_L||m_tHS_WeavePara.eWeaveType == WEAVE_MOONB||m_tHS_WeavePara.eWeaveType == WEAVE_MOONF)
		{
			m_bDStopMode = true;
			m_dWeaveSPos = 0;

			double dSetT = m_dWeavePeriod/3;  //增大一定的时间，避免加速度过大m_dWeavePeriod/4

			if(m_tHS_WeavePara.eWeaveType == WEAVE_L)
			{
				dSetT = m_dWeavePeriod/4;

				//计算偏置向量
				LOG_ALGO("SPos =  %.3lf,%.3lf,%.3lf",dSPos[0],dSPos[1],dSPos[2]);
				LOG_ALGO("EPos =  %.3lf,%.3lf,%.3lf",dEPos[0],dEPos[1],dEPos[2]);
				LOG_ALGO("L1Pos =  %.3lf,%.3lf,%.3lf",
					m_tHS_WeavePara.dLoc1[0],m_tHS_WeavePara.dLoc1[1],m_tHS_WeavePara.dLoc1[2]);
				LOG_ALGO("L2Pos =  %.3lf,%.3lf,%.3lf",
					m_tHS_WeavePara.dLoc2[0],m_tHS_WeavePara.dLoc2[1],m_tHS_WeavePara.dLoc2[2]);

				double dVX[3] = {0};

				dVX[0] = dEPos[0] - dSPos[0];
				dVX[1] = dEPos[1] - dSPos[1];
				dVX[2] = dEPos[2] - dSPos[2];

				HS_Math::Normalize(3, dVX);
				double dVXZ[3] = {0};
				dVXZ[0] = m_tHS_WeavePara.dLoc1[0] - dSPos[0];
				dVXZ[1] = m_tHS_WeavePara.dLoc1[1] - dSPos[1];
				dVXZ[2] = m_tHS_WeavePara.dLoc1[2] - dSPos[2];

				//求解得到Y轴和Z轴向量
				double dVY[3] = {0};
				double dVZ[3] = {0};

				HS_Math::Matrix_VecCross(dVXZ,dVX,dVY);
				HS_Math::Normalize(3, dVY);
				HS_Math::Matrix_VecCross(dVX,dVY,dVZ);
				HS_Math::Normalize(3, dVZ);

				memcpy(m_dWeaveV1,dVZ,sizeof(m_dWeaveV1));

				dVXZ[0] = m_tHS_WeavePara.dLoc2[0] - dSPos[0];
				dVXZ[1] = m_tHS_WeavePara.dLoc2[1] - dSPos[1];
				dVXZ[2] = m_tHS_WeavePara.dLoc2[2] - dSPos[2];

				HS_Math::Matrix_VecCross(dVXZ,dVX,dVY);
				HS_Math::Normalize(3, dVY);
				HS_Math::Matrix_VecCross(dVX,dVY,dVZ);
				HS_Math::Normalize(3, dVZ);

				memcpy(m_dWeaveV2,dVZ,sizeof(m_dWeaveV2));
			}

			if(m_tHS_WeavePara.eWeaveType == WEAVE_SINE&&m_bStopSineNextFlag)
			{
				LOG_ALGO("Stop Sine Start With Pre!");
				dSetT = m_dWeavePeriod/2;
				if(!m_bWeaveStopDir)
				{
					m_dWeaveSPos = m_dWeaveAmp;
					m_dWeavePAll = -m_dWeaveAmp*2;
					WeavePlan(m_dWeavePAll,dSetT);
					m_eWeaveState = WEAVE_UP;
				}
				else
				{
					m_dWeaveSPos = -m_dWeaveAmp;
					m_dWeavePAll = m_dWeaveAmp*2;
					WeavePlan(m_dWeavePAll,dSetT);
					m_eWeaveState = WEAVE_DOWN;
				}
			}
			else
			{
				if(m_bWeaveStopDir)
				{
					m_dWeavePAll = -m_dWeaveAmp;
					WeavePlan(m_dWeavePAll,dSetT);
					m_eWeaveState = WEAVE_UP;
				}
				else
				{
					m_dWeavePAll = m_dWeaveAmp;
					WeavePlan(m_dWeavePAll,dSetT);
					m_eWeaveState = WEAVE_DOWN;
				}
			}			


			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				if(dSetT < m_dAheadTAll)
				{
					WeaveMainPlan(dSetT);
				}
				else
				{
					//时间不足完成规划，则不进行运动
					m_dWeaveTAll = 0;
					m_eWeaveState = WEAVE_DONE;
					m_dTLast = 0;
					LOG_ALGO("Stop Type Ignore Move!");
				}				
			}
			else
			{
				double dTimeAdd = m_tHS_WeavePara.dLDT;
				if(m_bWeaveStopDir)
				{
					dTimeAdd = m_tHS_WeavePara.dRDT;
				}

				//无法完成整段跟踪
				if(dSetT + dTimeAdd + m_dWeavePeriod/3 > m_dAheadTAll)
				{
					m_dWeaveTAll = 0;
					m_eWeaveState = WEAVE_DONE;
					m_dTLast = 0;
					LOG_ALGO("Move Type Ignore Move!");
				}
			}
			
			m_bWeaveStopFlag = false;
		}
		else if(m_tHS_WeavePara.eWeaveType == WEAVE_CIRCLE||m_tHS_WeavePara.eWeaveType == WEAVE_8)
		{
			m_dWeaveCicleVel = 2*PI/m_dWeavePeriod;

			WeavePlanCircle(true);
			double dTStart = m_dWeaveCicleTAcc;
			if(m_dWeaveCiclePosAcc < 2*PI)
				dTStart += (2*PI - m_dWeaveCiclePosAcc)/m_dWeaveCicleVel;

			if(dTStart + m_dWeavePeriod < m_dAheadTAll)
			{				
				m_eWeaveState = WEAVE_CIRCLESTART;
			}
			else
			{
				m_dWeaveTAll = 0;
				m_eWeaveState = WEAVE_DONE;
				LOG_ALGO("Circle Ignore Move!");
			}

			m_dCircleBaseVel = m_dWeaveMoveBVel;
			LOG_ALGO("CircleBaseVel = %.3lf",m_dCircleBaseVel);

			m_bStopSineNextFlag = false;
		}
		else
		{
			m_dWeavePAll = m_dWeaveAmp;
			WeavePlan(m_dWeavePAll);
			m_eWeaveState = WEAVE_STARTDOWN;
			m_dWeaveSPos = 0;
			m_bWeaveStopFlag = false;

			//规划的时间与总时间做对比，提前进行优化处理
			if(m_dWeaveTAll + 2*2*m_dWeaveAmp/m_dWeaveVel + sqrt(3*m_dWeaveAmp/2/(m_dWeaveAccMAX*2/3))*2 > m_dAheadTAll)
			{
				////此时仅能够进行一个波峰的规划，且其幅度要根据时间进行重新计算
				//double dAmpL = 0.001;	//设定一个最小值，避免求得较小，规划时有除零错误
				//double dAmpR = 2*m_dWeaveAmp;
				//double dAmp = (dAmpL + dAmpR)/2;
				//double dCalc = 0;
				//for(int iCnt = 0;iCnt < 30;iCnt++)
				//{
				//	//dCalc = m_dWeavePeriod*sqrt(dAmp/2/m_dWeaveAmp);				
				//	dCalc = (sqrt(3*fabs(dAmp)/2/(m_dWeaveAccMAX*2/3))*2)+m_dCycle;
				//	if(dCalc*2 > m_dAheadTAll)						
				//		dAmpR = dAmp;						
				//	else
				//		dAmpL = dAmp;
				//	dAmp = (dAmpL + dAmpR)/2;
				//}	
				//m_dWeaveStopAmp = dAmpL;
				//m_dWeavePAll = m_dWeaveStopAmp;

				//WeavePlan(m_dWeavePAll);
				//m_dWeaveSPos = -Eps;
				//m_eWeaveState = WEAVE_STOP;
				//m_bWeaveStopFlag = true;
				m_dWeaveTAll = 0;
				m_eWeaveState = WEAVE_DONE;
				LOG_ALGO("Quick Sine Time Not Reach!");
			}
			m_bStopSineNextFlag = false;
			LOG_ALGO("Quick Mode!");
		}

		double dCPos[MaxAxisNum] = {0};
		if(!m_bStopSineNextFlag)
		{
			memset(m_dWeavePosSave,0,sizeof(m_dWeavePosSave));

			//dInitJPos可能存在偏差，直接使用起点位置
			//m_HS_Kinematic->HS_JPosToCPos(dInitJPos,CP_ToolWork,dCPos);
			//memcpy(&m_dMainSavePos[0],dCPos,sizeof(double)*MaxAxisNum);

			memcpy(&m_dMainSavePos[0],dSPos,sizeof(double)*MaxAxisNum);
		}

		LOG_ALGO("WeaveType = %d,WeavePlane = %d,Pitch = %.3lf,Ori = %.3lf",
			m_tHS_WeavePara.eWeaveType,m_tHS_WeavePara.eWeavePlane,m_tHS_WeavePara.dPitch,m_tHS_WeavePara.dOri);
		LOG_ALGO("Weave On:Amp = %.3lf,Freq = %.3lf!",
			m_dWeaveAmp,m_dWeaveFreq);
		LOG_ALGO("DType = %d,LDT = %.3lf,RDT = %.3lf",
			m_tHS_WeavePara.eWeaveDType,m_tHS_WeavePara.dLDT,m_tHS_WeavePara.dRDT);
		LOG_ALGO("WeavePAll = %.3lf,m_dAheadTAll = %.3lf,m_bStopSineNextFlag = %d",
			m_dWeavePAll,m_dAheadTAll,(int)m_bStopSineNextFlag);
		m_bWeaveWaitFlag = false;

		//m_HS_MotionParaLib->HS_SetWeaveState(true,m_dWeaveFreq);

		if(!m_bStopSineNextFlag)
		{
			m_bWeaveFilterFlag = false;
			memset(m_dMCoord,0,sizeof(m_dMCoord));
		}
		m_iWeaveNum = 1;
		//m_HS_MotionParaLib->HS_SetWeaveNum(m_iWeaveNum);
	}
	else
	{
		LOG_ALGO("Weave No StartPlan,WeaveEn = %d,m_dAheadTAll = %.3lf!",(int)tWeavePara.bWeaveEn,m_dAheadTAll);
		m_iWeaveNum = 0;
		m_bWeaveStopDir = false;
		//m_HS_MotionParaLib->HS_SetWeaveNum(m_iWeaveNum);

		//判断起点与当前点位的一致性问题
		if(!m_bSmoothPreFlag&&!m_bStopSineNextFlag)
		{

		}
	}

	m_bStopSineNextFlag = false;
	m_bStopWeaveMainWaitFlag = false;
	m_bWeavePErrorFlag = true;
	m_bWeaveChangeYDir = false;
	return 0;
}
/************************************************
函数功能：圆弧摆焊启停规划
参    数：
		bStartFlag---启动或者停止段【true为启动段】
返 回 值：
*************************************************/
int HS_Int_Move::WeavePlanCircle(bool bStartFlag)
{
	double dCircleAcc = 200;
	double dTAcc = m_dWeaveCicleVel*m_tHS_WeavePara.dAmplitude/dCircleAcc;
	double dTLimit = 0.1;
	double dTLimitMax = m_dWeavePeriod*2;
	if(dTAcc < dTLimit)
		dTAcc = dTLimit;
	if(dTAcc > dTLimitMax)
		dTAcc = dTLimitMax;

	if(bStartFlag)
	{
		m_dWeaveKa = - m_dWeaveCicleVel/(2*dTAcc*dTAcc*dTAcc);
		m_dWeaveKb = m_dWeaveCicleVel/(dTAcc*dTAcc);	
		m_dWeaveTCur = m_dCycle;
	}

	m_dWeaveCicleTAcc = dTAcc;
	m_dWeaveCiclePosAcc = m_dWeaveCicleVel*dTAcc/2;
	return 0;
}
/************************************************
函数功能：设置主运动的时间【减小时间】
参    数：dTSet-------设置时间	
		 bStopFlag---最终停止运动段
返 回 值：主运动剩余时间
*************************************************/
double HS_Int_Move::WeaveMainPlan(double dTSet,bool bStopFlag)
{
	double dTLast = 0;
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		dTLast = m_HS_VelPlan_Para[i]->RePlanByTime(dTSet,bStopFlag);
	}	
	m_dTLast = dTLast;
	return dTLast;
}
/************************************************
函数功能：根据振幅求解相应的加减速参数
参    数：dWeaveAmp----振幅值		 
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeavePlan(double dWeaveAmp)
{
	//LOG_ALGO("WeaveAmp = %.3lf,m_dWeaveAmp = %.3lf,m_dWeavePeriod = %.3lf!",dWeaveAmp,m_dWeaveAmp,m_dWeavePeriod);

	//运行总时间(保证加速度恒定）
	//m_dWeaveTAll = m_dWeavePeriod/2*sqrt(fabs(dWeaveAmp/2/m_dWeaveAmp));
	double dWeaveAccMAX = m_dWeaveAccMAX*2/3;	//最大加速度小一些，减少抖动
	m_dWeaveTAll = sqrt(3*fabs(dWeaveAmp)/2/dWeaveAccMAX)*2;

	if(m_dWeaveTAll < Eps)
		m_dWeaveTAll = Eps;

	//运行速度
	double dWeaveVel = 2*dWeaveAmp/m_dWeaveTAll;

	//求解Ka、Kb值
	m_dWeaveKa = - 4*dWeaveVel/(m_dWeaveTAll*m_dWeaveTAll*m_dWeaveTAll);
	m_dWeaveKb = 4*dWeaveVel/(m_dWeaveTAll*m_dWeaveTAll);	
	m_dWeaveTCur = m_dCycle;

	m_bSinglePlan = false;
	return 0;
}
/************************************************
函数功能：根据振幅求解相应的加减速参数【限定时间】
参    数：dWeaveAmp----振幅值		 
		 dTAll--------限定的时间
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeavePlan(double dWeaveAmp,double dTAll)
{
	m_dWeaveTAll = dTAll;

	if(m_dWeaveTAll < Eps)
		m_dWeaveTAll = Eps;

	//运行速度
	double dWeaveVel = 2*dWeaveAmp/m_dWeaveTAll;

	//求解Ka、Kb值
	m_dWeaveKa = - 4*dWeaveVel/(m_dWeaveTAll*m_dWeaveTAll*m_dWeaveTAll);
	m_dWeaveKb = 4*dWeaveVel/(m_dWeaveTAll*m_dWeaveTAll);	
	m_dWeaveTCur = m_dCycle;

	m_bSinglePlan = false;
	return 0;
}
/************************************************
函数功能：根据起始速度和运行时间规划加减速参数
参    数：dSVel----启动速度
		 dEVel----结束速度
		 dTAll----运行时间
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeavePlan(double dSVel,double dEVel,double dTAll)
{
	m_dWeaveTAll = dTAll;
	m_dWeaveSVel = dSVel;

	if(m_dWeaveTAll < Eps)
		m_dWeaveTAll = Eps;

	//求解Ka、Kb值
	m_dWeaveKa = -(dEVel - dSVel)/(2*m_dWeaveTAll*m_dWeaveTAll*m_dWeaveTAll);
	m_dWeaveKb = (dEVel - dSVel)/(m_dWeaveTAll*m_dWeaveTAll);	

	m_dWeaveTCur = m_dCycle - m_dWeaveTOff;
	if(m_dWeaveTCur < Eps)
		m_dWeaveTCur = m_dCycle;

	int iCnt = (int)((m_dWeaveTAll + m_dWeaveTOff)/m_dCycle);
	m_dWeaveTOff = m_dWeaveTAll + m_dWeaveTOff - iCnt*m_dCycle;

	m_bSinglePlan = true;
	return 0;
}
/************************************************
函数功能：规划摆焊运动的停止，使得运动停止
参    数：
返 回 值：
*************************************************/
int HS_Int_Move::WeaveStop()
{
	//求解其速度，加速度值
	double dWeaveSVel = (m_dWeavePosSave[0] - m_dWeavePosSave[1])/m_dCycle;
	double dVelP = (m_dWeavePosSave[1] - m_dWeavePosSave[2])/m_dCycle;
	double dWeaveSAcc = (dWeaveSVel - dVelP)/m_dCycle;
	dWeaveSVel += dWeaveSAcc*m_dCycle/2;

	m_dWeaveStopSVel = dWeaveSVel;
	m_dWeaveSPos = m_dWeavePosSave[0];
	m_dWeaveTCur = m_dCycle;
	//直接按最大加速度减速停止
	//同步时间，根据加速度优化出最佳的时间，先按照初始加速度为0进行求解
	//位置轴
	double dMaxTime = fabs(3*dWeaveSVel/2/m_dWeaveAccMAX);
	if(dMaxTime < 0.1)
		dMaxTime = 0.1;
	////////////////////////////////////////////////////////////////////////////////////////////
	//最优时间求解
	double dTTemp = dMaxTime;			//求解时间值
	double dTRight = 2*dMaxTime;
	double dTLeft = Eps;
	double dAMax = {0};
	bool bOverAcc = false;				//是否超出加速度限制
	int MAXITERCNT = 10;				//迭代次数
	for(int iCnt = 0;iCnt < MAXITERCNT;iCnt++)
	{
		bOverAcc = false;		
		//求解各轴的最大加速度
		double dKa = (dWeaveSAcc*dTTemp + 2*dWeaveSVel)/4/dTTemp/dTTemp/dTTemp;
		double dKb = (-3*dWeaveSVel - 2*dWeaveSAcc*dTTemp)/3/dTTemp/dTTemp;

		if(fabs(dKa) < Eps)
			dAMax = 0;
		else
		{
			//判断最大值的时间位置
			double dTMax = -dKb/4/dKa;												
			dAMax = fabs(-3*dKb*dKb/dKa/4 + dWeaveSAcc);	
		}

		if(dAMax > m_dWeaveAccMAX*1.1)
		{		
			bOverAcc = true;
		}
					

		if(!bOverAcc)
		{				
			dTRight = dTTemp;
			dTTemp = (dTRight + dTLeft)/2;					
		}
		else
		{
			dTLeft = dTTemp;
			dTTemp = (dTRight + dTLeft)/2;					
		}
	}			
	dMaxTime = dTTemp;			
	if(dMaxTime < 4*m_dCycle)
		dMaxTime = 4*m_dCycle;
	LOG_ALGO("Weave StopTime = %.3lf",dMaxTime);
	m_dWeaveTStop = dMaxTime;
	m_bWeaveStopDoneFlag = false;
	m_dWeaveKa = (dWeaveSAcc*dMaxTime + 2*dWeaveSVel)/(4*dMaxTime*dMaxTime*dMaxTime);
	m_dWeaveKb = (-3*dWeaveSVel - 2*dWeaveSAcc*dMaxTime)/(3*dMaxTime*dMaxTime);
	m_dWeaveKc = dWeaveSAcc/2;	

	return 0;
}

/************************************************
函数功能：摆焊点位实时计算处理
参    数：
		dPosOut-----点位更改输出
		bStopFlag---停止状态
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeavePosCalc(double *dPosOut,bool bStopFlag)
{
	//获取当前插补位置值
	double dCPos[MaxAxisNum] = {0};
	m_HS_Kinematic->HS_MPosToCPos(m_dRMPos,dCPos);

	//摆焊位置计算获取
	double dWeavePos = 0;	
	WeaveMoveHandle(dWeavePos,bStopFlag);

	//m_HS_MotionParaLib->HS_SetWeavePos(dWeavePos);
	//m_HS_Printer->outDebugInfo("Motion_P","Move","WeaveMoveHandle",0,AllDeb,"dWeavePos = %.3lf!",dWeavePos);

	//点位缓存
	m_dWeavePosSave[2] = m_dWeavePosSave[1];
	m_dWeavePosSave[1] = m_dWeavePosSave[0];
	m_dWeavePosSave[0] = dWeavePos;	

	m_dWeaveTCur += m_dCycle;		
	m_dAheadTAll -= m_dCycle;		
		
	//叠加方向求解，工具Z轴向与当前运行方向的垂线方向为Z轴方向
	//当前运行方向认为是X轴
	double dVX[3] = {0};

	dVX[0] = dCPos[0] - m_dMainSavePos[0][0];
	dVX[1] = dCPos[1] - m_dMainSavePos[0][1];
	dVX[2] = dCPos[2] - m_dMainSavePos[0][2];

	double dVXLength = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);

	//VX方向位移量过小使得坐标系的求解值无法使用
	bool bCoordOKFlag = true;
	if(dVXLength < 1e-4)
		bCoordOKFlag = false;

	//m_HS_Printer->outDebugInfo("Motion_P","Move","WeavePosCalc",0,AllDeb,"dVX = %.6lf,%.6lf,%.6lf;Length = %.6lf",dVX[0],dVX[1],dVX[2],dVXLength);
		
	//归一化
	HS_Math::Normalize(3, dVX);

	if(m_bWeaveChangeYDir)
	{
		dVX[0] = -dVX[0];
		dVX[1] = -dVX[1];
		dVX[2] = -dVX[2];
	}

	//工具坐标系的Z轴认为是XZ平面向量
	double dVXZ[3] = {0};
	//摆焊坐标系修改【Z轴沿工具坐标系的Z轴方向，20241025永华需求修改】
	dVXZ[0] = m_dRMPos[0][2];dVXZ[1] = m_dRMPos[1][2];dVXZ[2] = m_dRMPos[2][2];//点位坐标系的Z轴矢量
	//dVXZ[0] = -m_dRMPos[0][2];dVXZ[1] = -m_dRMPos[1][2];dVXZ[2] = -m_dRMPos[2][2];//点位坐标系的Z轴矢量

	//求解得到Y轴和Z轴向量
	double dVY[3] = {0};
	double dVZ[3] = {0};

	HS_Math::Matrix_VecCross(dVXZ,dVX,dVY);
	HS_Math::Normalize(3, dVY);
	HS_Math::Matrix_VecCross(dVX,dVY,dVZ);
	HS_Math::Normalize(3, dVZ);

	double dMCoord[3][3] = {0};
	dMCoord[0][0] = dVX[0];dMCoord[0][1] = dVY[0];dMCoord[0][2] = dVZ[0];
	dMCoord[1][0] = dVX[1];dMCoord[1][1] = dVY[1];dMCoord[1][2] = dVZ[1];
	dMCoord[2][0] = dVX[2];dMCoord[2][1] = dVY[2];dMCoord[2][2] = dVZ[2];

	double dVWeave[3] = {0};
	
	if(m_tHS_WeavePara.eWeaveType == WEAVE_SINE)
	{
		switch(m_tHS_WeavePara.eWeavePlane)	
		{
		case WEAVE_X:
			dVWeave[0] = dWeavePos;//X方向
			break;
		case WEAVE_Y:
			dVWeave[1] = dWeavePos;//Y方向
			break;
		case WEAVE_Z:
			dVWeave[2] = dWeavePos;//Z方向
			break;
		default:
			break;
		}
	}
	else if(m_tHS_WeavePara.eWeaveType == WEAVE_MOONB||m_tHS_WeavePara.eWeaveType == WEAVE_MOONF)
	{
		//将L摆焊的数值转化为月牙处理
		double dAngle = 0;
		if(m_tHS_WeavePara.eWeaveType == WEAVE_MOONF)
			dAngle = dWeavePos/m_tHS_WeavePara.dAmplitude*PI/2;
		else
			dAngle = -dWeavePos/m_tHS_WeavePara.dAmplitude*PI/2 + PI;

		double dCirclePosX = m_tHS_WeavePara.dAmplitude*(1 - cos(dAngle));
		double dCriclePosY = m_tHS_WeavePara.dAmplitude*sin(dAngle);

		if(m_tHS_WeavePara.eWeaveType == WEAVE_MOONB)
			dCirclePosX -=  m_tHS_WeavePara.dAmplitude*2;

		switch(m_tHS_WeavePara.eWeavePlane)	
		{
		case WEAVE_X:			
		case WEAVE_Y:
			dVWeave[0] = dCirclePosX;
			dVWeave[1] = dCriclePosY;
			break;
		case WEAVE_Z:
			dVWeave[0] = dCirclePosX;
			dVWeave[2] = dCriclePosY;
			break;
		default:
			break;
		}
	}
	else if(m_tHS_WeavePara.eWeaveType == WEAVE_L)
	{
		if(dWeavePos > 0)
		{
			dVWeave[0] = m_dWeaveV1[0]*dWeavePos;
			dVWeave[1] = m_dWeaveV1[1]*dWeavePos;
			dVWeave[2] = m_dWeaveV1[2]*dWeavePos;
		}
		else
		{
			dVWeave[0] = -m_dWeaveV2[0]*dWeavePos;
			dVWeave[1] = -m_dWeaveV2[1]*dWeavePos;
			dVWeave[2] = -m_dWeaveV2[2]*dWeavePos;
		}
	}
	else if(m_tHS_WeavePara.eWeaveType == WEAVE_CIRCLE)
	{
		double dCirclePosX = m_tHS_WeavePara.dAmplitude*(1 - cos(dWeavePos));
		double dCriclePosY = m_tHS_WeavePara.dAmplitude*sin(dWeavePos);

		double dKXVel = 1.0;
		
		if(m_dCircleBaseVel > Eps)
			dKXVel = dVXLength/m_dCycle/m_dCircleBaseVel;

		switch(m_tHS_WeavePara.eWeavePlane)	
		{
		case WEAVE_X:			
		case WEAVE_Y:
			dVWeave[0] = dCirclePosX;
			dVWeave[1] = dCriclePosY;
			break;
		case WEAVE_Z:
			dVWeave[0] = dCirclePosX*dKXVel;
			//dVWeave[0] = dCirclePosX;
			dVWeave[2] = dCriclePosY;
			break;
		default:
			break;
		}
	}
	else if(m_tHS_WeavePara.eWeaveType == WEAVE_8)
	{
		dWeavePos += PI/2; 
		double dSinPos = sin(dWeavePos);
		double dCosPos = cos(dWeavePos);
		double dBase = 1 + dSinPos*dSinPos;

		double dPosX = m_tHS_WeavePara.dAmplitude*dSinPos*dCosPos/dBase;
		double dPosY = m_tHS_WeavePara.dAmplitude*dCosPos/dBase;

		switch(m_tHS_WeavePara.eWeavePlane)	
		{
		case WEAVE_X:			
		case WEAVE_Y:
			dVWeave[0] = dPosX;
			dVWeave[1] = dPosY;
			break;
		case WEAVE_Z:
			dVWeave[0] = dPosX;
			dVWeave[2] = dPosY;
			break;
		default:
			break;
		}
	}

	////////////////////////////////////
	double dAddPos[3] = {0};


	//绕X轴旋转【Pitch】
	double dRotPitch[3][3] = {0};
	double dSinPicth = sin(angle2Rad(m_tHS_WeavePara.dPitch));
	double dCosPicth = cos(angle2Rad(m_tHS_WeavePara.dPitch));
	dRotPitch[0][0] = 1;
	dRotPitch[1][1] = dCosPicth;
	dRotPitch[2][1] = dSinPicth;
	dRotPitch[1][2] = -dSinPicth;
	dRotPitch[2][2] = dCosPicth;

	//Ori:当前运动是X轴方向，则不处理；Y方向，则绕Z轴旋转，Z方向，则绕Y轴旋转，正负反向，为正时向行进方向旋转
	double dRotOri[3][3] = {0};
	HS_Math::Matrix_Eye(3,&dRotOri[0][0]);
	double dSinOri = sin(angle2Rad(-m_tHS_WeavePara.dOri));
	double dCosOri = cos(angle2Rad(-m_tHS_WeavePara.dOri));
	switch(m_tHS_WeavePara.eWeavePlane)	
	{
	case WEAVE_X:
		break;
	case WEAVE_Y:
		dRotOri[0][0] = dCosOri;
		dRotOri[0][1] = -dSinOri;
		dRotOri[1][0] = dSinOri;
		dRotOri[1][1] = dCosOri;
		break;
	case WEAVE_Z:
		dRotOri[0][0] = dCosOri;
		dRotOri[2][0] = -dSinOri;
		dRotOri[0][2] = dSinOri;
		dRotOri[2][2] = dCosOri;
		break;
	default:
		break;
	}

	double dMTemp[3][3] = {0};
	double dMComp[3][3] = {0};
	
	//坐标系使用
	if(bCoordOKFlag)
	{
		//对于类似往复运动场景，摆动坐标系需摆焊方向向量的未发生变化，则进行复位重置，不进行滤波处理，防止摆焊坐标系的偏移
		//场景一般出现在主运动的位移量较小的场景
		double dMainXLengthLimit = 0.05;
		if(dVXLength < dMainXLengthLimit)
		{
			int iDir = 0;
			switch(m_tHS_WeavePara.eWeavePlane)	
			{
			case WEAVE_X:
				iDir = 0;
				break;
			case WEAVE_Y:
				iDir = 1;
				break;
			case WEAVE_Z:
				iDir = 2;
				break;
			default:
				break;
			}

			double dVCur[3] = {0};
			double dVPre[3] = {0};

			for(int i = 0;i < 3;i++)
			{
				dVCur[i] = dMCoord[i][iDir];
				dVPre[i] = m_dMCoord[i][iDir];
			}

			//夹角
			double dAngleC = dVCur[0]*dVPre[0] + dVCur[1]*dVPre[1] + dVCur[2]*dVPre[2];

			if(fabs(dAngleC) > 0.95)
			{
				//m_HS_Printer->outDebugInfo("Motion_P","Move","WeavePosCalc",0,AllDeb,"dAngleC = %.6lf;Length = %.6lf",dAngleC,dVXLength);
				
				if(m_tHS_WeavePara.eWeaveType == WEAVE_CIRCLE)
					m_bWeaveFilterFlag = false;
			}

		}

		//m_bWeaveFilterFlag = false;
		//WeaveCoordHandle(dMCoord);
		WeaveCoordHandle_Q(dMCoord);
		memcpy(m_dMCoord,dMCoord,sizeof(dMCoord));
	}
	else
		memcpy(dMCoord,m_dMCoord,sizeof(m_dMCoord));

	//m_HS_MotionParaLib->HS_SetWeaveCoord(m_dMCoord);

	HS_Math::Matrix_Multi(3,3,3,&dMCoord[0][0],&dRotPitch[0][0],&dMTemp[0][0]);
	HS_Math::Matrix_Multi(3,3,3,&dMTemp[0][0],&dRotOri[0][0],&dMComp[0][0]);
	HS_Math::Matrix_Multi(3,3,1,&dMComp[0][0],dVWeave,dAddPos);

	double dWeaveAllPos[MaxAxisNum] = {0}; //摆动时的合成位置

	memcpy(dWeaveAllPos,dCPos,sizeof(double)*MaxAxisNum);
	//叠加转换后的坐标
	dWeaveAllPos[0] += dAddPos[0];
	dWeaveAllPos[1] += dAddPos[1];
	dWeaveAllPos[2] += dAddPos[2];		

	double dRMPos[4][4] = {0};

	m_HS_Kinematic->HS_CPosToMPos(dWeaveAllPos,dRMPos);
	memcpy(dPosOut,dRMPos,sizeof(dRMPos));

	//缓存位置值
	memcpy(&m_dMainSavePos[1],&m_dMainSavePos[0],sizeof(double)*MaxAxisNum);
	memcpy(&m_dMainSavePos[0],dCPos,sizeof(double)*MaxAxisNum);
	
	return 0;
}
/************************************************
函数功能：焊接坐标系计算，焊枪Z轴为Z轴，前进方向为XZ平面
参    数：		
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeldCoordCalc()
{
	//获取当前插补位置值
	double dCPos[MaxAxisNum] = {0};
	m_HS_Kinematic->HS_MPosToCPos(m_dRMPos,dCPos);

	//m_HS_Printer->outDebugInfo("Motion_P","Move","WeaveMoveHandle",0,AllDeb,"dWeavePos = %.3lf!",dWeavePos);

	//叠加方向求解，工具Z轴向与当前运行方向的垂线方向为Z轴方向
	//当前运行方向认为是X轴
	double dVX[3] = {0};

	dVX[0] = dCPos[0] - m_dMainSavePos[0][0];
	dVX[1] = dCPos[1] - m_dMainSavePos[0][1];
	dVX[2] = dCPos[2] - m_dMainSavePos[0][2];

	double dVXLength = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);

	//VX方向位移量过小使得坐标系的求解值无法使用
	bool bCoordOKFlag = true;
	if(dVXLength < 1e-4)
		bCoordOKFlag = false;

	//m_HS_Printer->outDebugInfo("Motion_P","Move","WeavePosCalc",0,AllDeb,"dVX = %.6lf,%.6lf,%.6lf;Length = %.6lf",dVX[0],dVX[1],dVX[2],dVXLength);

	//归一化
	HS_Math::Normalize(3, dVX);

	//工具坐标系的Z轴认为是XZ平面向量
	double dVXZ[3] = {0};

	//摆焊坐标系修改【Z轴沿工具坐标系的Z轴方向，20241025永华需求修改】
	dVXZ[0] = m_dRMPos[0][2];dVXZ[1] = m_dRMPos[1][2];dVXZ[2] = m_dRMPos[2][2];//点位坐标系的Z轴矢量
	//dVXZ[0] = -m_dRMPos[0][2];dVXZ[1] = -m_dRMPos[1][2];dVXZ[2] = -m_dRMPos[2][2];//点位坐标系的Z轴矢量

	//求解得到Y轴和Z轴向量
	double dVY[3] = {0};
	double dVZ[3] = {0};

	HS_Math::Matrix_VecCross(dVXZ,dVX,dVY);
	HS_Math::Normalize(3, dVY);
	HS_Math::Matrix_VecCross(dVX,dVY,dVZ);
	HS_Math::Normalize(3, dVZ);

	double dMCoord[3][3] = {0};
	dMCoord[0][0] = dVX[0];dMCoord[0][1] = dVY[0];dMCoord[0][2] = dVZ[0];
	dMCoord[1][0] = dVX[1];dMCoord[1][1] = dVY[1];dMCoord[1][2] = dVZ[1];
	dMCoord[2][0] = dVX[2];dMCoord[2][1] = dVY[2];dMCoord[2][2] = dVZ[2];


	//坐标系使用
	if(bCoordOKFlag)
	{
		//WeaveCoordHandle(dMCoord);
		memcpy(m_dMCoord,dMCoord,sizeof(dMCoord));
	}
	else
		memcpy(dMCoord,m_dMCoord,sizeof(m_dMCoord));

	//m_HS_MotionParaLib->HS_SetWeaveCoord(m_dMCoord);

	//缓存位置值
	memcpy(&m_dMainSavePos[1],&m_dMainSavePos[0],sizeof(double)*MaxAxisNum);
	memcpy(&m_dMainSavePos[0],dCPos,sizeof(double)*MaxAxisNum);
	return 0;
}
/************************************************
函数功能：摆焊坐标系的过滤平滑处理【欧拉角方式】
参    数：		
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveCoordHandle(double dWCoord[3][3])
{
	int iErrorId = 0;

	double dEuler[3] = {0};

	dEuler[1] = atan2(-dWCoord[2][0],sqrt(dWCoord[2][1]*dWCoord[2][1] + dWCoord[2][2]*dWCoord[2][2]));	
	dEuler[0] = atan2(dWCoord[1][0],dWCoord[0][0]);	
	dEuler[2] = atan2(dWCoord[2][1],dWCoord[2][2]);

	//dEuler[0] = atan2(dWCoord[2][2],dWCoord[1][2]);
	//dEuler[1] = atan2(sqrt(dWCoord[1][2]*dWCoord[1][2]+dWCoord[2][2]*dWCoord[2][2]),dWCoord[3][2]);
	//dEuler[2] = atan2(dWCoord[3][1],-dWCoord[3][0]);

	if(fabs(dWCoord[2][0]-1.0) < 1e-8||fabs(dWCoord[2][0]+1.0) < 1e-8)
	{
		m_bWeaveFilterFlag = false;
	}

	//m_HS_Printer->outDebugInfo("Motion_P","Move","WeaveCoordHandle",0,AllDeb,"dEuler = %.6lf,%.6lf,%.6lf;WeaveFilterFlag = %d",dEuler[0],dEuler[1],dEuler[2],m_bWeaveFilterFlag);
	//一阶滞后滤波处理
	if(!m_bWeaveFilterFlag)
	{
		memcpy(m_dWeaveEuler,dEuler,sizeof(double)*3);
	}
	else
	{
		//欧拉角防止角度超限制
		for(int i = 0;i < 3;i++)
		{
			if(dEuler[i] - m_dWeaveEuler[i] > PI)
				dEuler[i] -= 2*PI;
			else if(dEuler[i] - m_dWeaveEuler[i] < -PI)
				dEuler[i] += 2*PI;
		}

		//滤波限制幅度
		double dLimit = 0.01;  //0.006

		double dMaxEulerLimit = 0;
		//double dLimit = 0.1;

		if(m_bWeavePErrorFlag)
		{
			LOG_ALGO("dEuler = %.6lf,%.6lf,%.6lf;m_dWeaveEuler = = %.6lf,%.6lf,%.6lf",
				Rad2angle(dEuler[0]),Rad2angle(dEuler[1]),Rad2angle(dEuler[2]),Rad2angle(m_dWeaveEuler[0]),Rad2angle(m_dWeaveEuler[1]),Rad2angle(m_dWeaveEuler[2]));
			m_bWeavePErrorFlag = false;
		}

		for(int i = 0;i < 3;i++)
		{
			dMaxEulerLimit += fabs(dEuler[i] - m_dWeaveEuler[i]);

			if(dEuler[i] - m_dWeaveEuler[i] > dLimit)
				dEuler[i] = m_dWeaveEuler[i] + dLimit;
			else if(dEuler[i] - m_dWeaveEuler[i] < -dLimit)
				dEuler[i] = m_dWeaveEuler[i] - dLimit;			
		}

		double dK = 0.4;
		for(int i = 0;i < 3;i++)
		{
			m_dWeaveEuler[i] = dK*dEuler[i] + (1 - dK)*m_dWeaveEuler[i]; 
		}

		double dCosA = cos(m_dWeaveEuler[0]);
		double dSinA = sin(m_dWeaveEuler[0]);
		double dCosB = cos(m_dWeaveEuler[1]);
		double dSinB = sin(m_dWeaveEuler[1]);
		double dCosC = cos(m_dWeaveEuler[2]);
		double dSinC = sin(m_dWeaveEuler[2]);

		dWCoord[0][0] = dCosA*dCosB;
		dWCoord[1][0] = dSinA*dCosB;
		dWCoord[2][0] = -dSinB;

		dWCoord[0][1] = dCosA*dSinB*dSinC - dSinA*dCosC;
		dWCoord[1][1] = dSinA*dSinB*dSinC + dCosA*dCosC;
		dWCoord[2][1] = dCosB*dSinC;

		dWCoord[0][2] = dCosA*dSinB*dCosC + dSinA*dSinC;
		dWCoord[1][2] = dSinA*dSinB*dCosC - dCosA*dSinC;
		dWCoord[2][2] = dCosB*dCosC;
	}
	m_bWeaveFilterFlag = true;
	return iErrorId;
}
//四元数方式
int HS_Int_Move::WeaveCoordHandle_Q(double dWCoord[3][3])
{
	int iErrorId = 0;

	//一阶滞后滤波处理
	if(!m_bWeaveFilterFlag)
	{
		memcpy(m_dMCoord,dWCoord,sizeof(m_dMCoord));
	}
	else
	{
		double dMCoordR[3][3] = {0};
		HS_Math::Matrix_Inverse(3,&m_dMCoord[0][0],&dMCoordR[0][0]);

		double dMatrixQ[3][3] = {0};
		HS_Math::Matrix_Multi(3,3,3,&dMCoordR[0][0],&dWCoord[0][0],&dMatrixQ[0][0]);

		double dQ[4] = {0};
		HS_Math::Matrix_MToA(&dMatrixQ[0][0],dQ);

		if(m_bWeavePErrorFlag)
		{
			double dAngleChange = Rad2angle(dQ[0]);
			LOG_ALGO("dQ Angle = %.6lf",dAngleChange);
			if(dAngleChange > 170&&m_tHS_WeavePara.eWeaveType == WEAVE_SINE)
			{							
				//memcpy(dWCoord,m_dMCoord,sizeof(m_dMCoord));
				//m_bWeaveChangeYDir = true;
				//return 0;
				//如果是Z型摆动，则允许摆焊坐标系的方向X轴进行180度旋转【反向】，不影响实际摆焊处理	
				double dVX[3] = {0};
				double dVY[3] = {0};
				double dVZ[3] = {0};
				dVX[0] = -dWCoord[0][0];dVY[0] = dWCoord[0][1];dVZ[0] = dWCoord[0][2];
				dVX[1] = -dWCoord[1][0];dVY[1] = dWCoord[1][1];dVZ[1] = dWCoord[1][2];
				dVX[2] = -dWCoord[2][0];dVY[2] = dWCoord[2][1];dVZ[2] = dWCoord[2][2];

				HS_Math::Matrix_VecCross(dVZ,dVX,dVY);

				dWCoord[0][0] = dVX[0];dWCoord[0][1] = dVY[0];dWCoord[0][2] = dVZ[0];
				dWCoord[1][0] = dVX[1];dWCoord[1][1] = dVY[1];dWCoord[1][2] = dVZ[1];
				dWCoord[2][0] = dVX[2];dWCoord[2][1] = dVY[2];dWCoord[2][2] = dVZ[2];
				
				HS_Math::Matrix_Multi(3,3,3,&dMCoordR[0][0],&dWCoord[0][0],&dMatrixQ[0][0]);
				
				HS_Math::Matrix_MToA(&dMatrixQ[0][0],dQ);
				dAngleChange = Rad2angle(dQ[0]);
				LOG_ALGO("ReCalc dQ Angle = %.6lf",
					dAngleChange);

				m_bWeaveChangeYDir = true;
			}
			m_bWeavePErrorFlag = false;
		}

		//限制姿态变化
		double dK = 0.4;
		double dAngle = Rad2angle(dQ[0])*dK;

		if(dAngle > 0.05)
			dAngle = 0.05;

		dQ[0] = angle2Rad(dAngle);

		double dQMPos[4][4] = {0};		
		HS_Math::Matrix_AToM(dQ,dQMPos);

		double dMPosPre[3][3] = {0};
		double dMPosChange[3][3] = {0};

		memcpy(dMPosPre,m_dMCoord,sizeof(dMPosPre));
		for(int i = 0;i < 3;i++)
			for(int j = 0;j < 3;j++)
				dMPosChange[i][j] = dQMPos[i][j];

		HS_Math::Matrix_Multi(3,3,3,&dMPosPre[0][0],&dMPosChange[0][0],&dWCoord[0][0]);

	}
	m_bWeaveFilterFlag = true;
	return iErrorId;
}

/************************************************
函数功能：摆焊状态机切换以及获取点位输出
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMoveHandle(double &dWeavePos,bool bStopFlag)
{
	if(bStopFlag)
	{
		if(m_dWeaveTCur < m_dWeaveTStop)
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + 
				m_dWeaveTCur*(m_dWeaveKc*m_dWeaveTCur + m_dWeaveStopSVel);
		else
		{
			m_dWeaveTCur = m_dWeaveTStop;
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + 
				m_dWeaveTCur*(m_dWeaveKc*m_dWeaveTCur + m_dWeaveStopSVel);

			if(!m_bWeaveStopDoneFlag)
				LOG_ALGO("Weave Stop Done!");
			m_bWeaveStopDoneFlag = true;						
			m_bStopSineNextFlag = false;
			m_bStopSineWaitTFlag = false;
		}
	}
	else
	{
		switch(m_tHS_WeavePara.eWeaveType)
		{
		case WEAVE_SINE:
		case WEAVE_L:
		case WEAVE_MOONB:
		case WEAVE_MOONF:
			WeaveMove_SineAndL(dWeavePos);
			break;
		case WEAVE_CIRCLE:
		case WEAVE_8:
			WeaveMove_Circle(dWeavePos);
			break;
		default:
			break;
		}
	}
	
	return 0;
}
/************************************************
函数功能：摆焊状态机切换以及获取点位输出【Circle】
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMove_Circle(double &dWeavePos)
{
	switch(m_eWeaveState)
	{
	case WEAVE_CIRCLESTART:
		if(m_dWeaveTCur < m_dWeaveCicleTAcc)
		{
			dWeavePos = m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		else
		{
			dWeavePos = m_dWeaveCiclePosAcc + m_dWeaveCicleVel*(m_dWeaveTCur - m_dWeaveCicleTAcc);
			m_eWeaveState = WEAVE_CIRCLEMOVE;
		}
		break;
	case WEAVE_CIRCLEMOVE:
		{
			dWeavePos = m_dWeaveCiclePosAcc + m_dWeaveCicleVel*(m_dWeaveTCur - m_dWeaveCicleTAcc);

			//计算当前位置运动至结束的时间+一个循环，如果剩余时间不足，则规划减速停止
			int iCnt = (int)(dWeavePos/(2*PI));
			double dNPos = dWeavePos - iCnt*2*PI;
			double dLPos = 2*PI - dNPos;
			double dLTime = m_dWeaveCicleTAcc + (dLPos - m_dWeaveCiclePosAcc)/m_dWeaveCicleVel + m_dWeavePeriod;
			if(dLTime > m_dAheadTAll)
			{
				//停止在360度位置处
				m_dWeaveTCur = 0;				
				m_dWeaveSPos = dNPos;
				m_eWeaveState = WEAVE_CIRCLEEND;
				LOG_ALGO("Weave OFF,LTime = %.3lf,AheadTAll = %.3lf!",dLTime,m_dAheadTAll);

				if(dLTime - m_dAheadTAll < m_dWeavePeriod/3)
				{
					//主运动等待
					m_bStopWeaveMainWaitFlag = true;
					m_dWeaveCicleTSCon = (dLPos - m_dWeaveCiclePosAcc)/m_dWeaveCicleVel + m_dWeavePeriod;
					LOG_ALGO("Weave OFF,TSCon = %.3lf,MainMoveWait!",m_dWeaveCicleTSCon);
				}
				else
				{
					//修改减速停止时间，使得运动停止与主运动一致
					double dTAll = m_dAheadTAll;
					if(dTAll > 2*m_dCycle)
						dTAll -= 2*m_dCycle;
					double dTTemp = dLPos/m_dWeaveCicleVel;
					double dTAcc = (dTAll - dTTemp)*2;
					m_dWeaveCicleTSCon = dTAll - dTAcc;

					if(m_dWeaveCicleTSCon < Eps)
					{
						m_dWeaveCicleTSCon = 0;
						dTAcc = dLPos/m_dWeaveCicleVel*2;
					}
					m_dWeaveKa = - m_dWeaveCicleVel/(2*dTAcc*dTAcc*dTAcc);
					m_dWeaveKb = m_dWeaveCicleVel/(dTAcc*dTAcc);	
					m_dWeaveCicleTAcc = dTAcc;
					m_dWeaveCiclePosAcc = m_dWeaveCicleVel*dTAcc/2;
					LOG_ALGO("Weave OFF,TSCon = %.3lf,TAcc = %.3lf!",m_dWeaveCicleTSCon,dTAcc);
				}
			}
		}		
		break;
	case WEAVE_CIRCLEEND:
		if(m_dWeaveTCur < m_dWeaveCicleTSCon)
		{
			dWeavePos = m_dWeaveSPos + m_dWeaveCicleVel*m_dWeaveTCur;
		}
		else if(m_dWeaveTCur - (m_dWeaveCicleTSCon + m_dWeaveCicleTAcc) < -Eps)
		{
			dWeavePos = m_dWeaveSPos + m_dWeaveCicleVel*m_dWeaveCicleTSCon;

			double dTTemp = m_dWeaveTCur - m_dWeaveCicleTSCon;
			double dTempB = dTTemp*dTTemp*dTTemp;
			dWeavePos += m_dWeaveCicleVel*dTTemp - (m_dWeaveKa*dTTemp + m_dWeaveKb)*dTempB;	
		}
		else
		{
			dWeavePos = 0;
			m_eWeaveState = WEAVE_DONE;
		}
		break;
	case WEAVE_DONE:
		dWeavePos = 0;
		m_bWeaveFlag = false;
		m_bStopWeaveMainWaitFlag = false;
		LOG_ALGO("Weave OFF");
		break;
	default:
		break;
	}

	return 0;
}
/************************************************
函数功能：摆焊状态机切换以及获取点位输出【sine】
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMove_SineAndL(double &dWeavePos)
{
	static double dWeavePosSave = 0;
	static bool bWeaveNumCntFlag = false;
	if(m_bWeaveWaitFlag)
	{
		if(m_dWeaveTCur < m_dWeaveWaitTime)
		{
			//Wait
			dWeavePos = dWeavePosSave;
		}
		else
		{
			if(m_tHS_WeavePara.eWeaveType == WEAVE_SINE)
			{
				if(m_bDStopMode)
					WeaveMove_StopSine(dWeavePos);
				else
					WeaveMove_NormalSine(dWeavePos);
			}
			else if(m_tHS_WeavePara.eWeaveType == WEAVE_MOONB||m_tHS_WeavePara.eWeaveType == WEAVE_MOONF)
			{
				WeaveMove_StopSine(dWeavePos);
			}
			else if(m_tHS_WeavePara.eWeaveType == WEAVE_L)
			{
				WeaveMove_L(dWeavePos);
			}
		}
	}
	else
	{
		if(m_dWeaveTCur < m_dWeaveTAll)
		{
			if(!m_bSinglePlan)
			{
				if(m_dWeaveTCur < m_dWeaveTAll/2)
				{
					dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
				}
				else if(m_dWeaveTCur < m_dWeaveTAll)
				{
					double dT = m_dWeaveTAll - m_dWeaveTCur;
					dWeavePos = dT*dT*dT*(m_dWeaveKa*dT + m_dWeaveKb);	
					dWeavePos = m_dWeaveSPos + m_dWeavePAll - dWeavePos;

					if(m_eWeaveState == WEAVE_DOWN&&m_dWeaveSPos + m_dWeaveAmp < Eps&&!bWeaveNumCntFlag)
					{
						//当前处于由下往上过程中
						m_iWeaveNum++;
						//m_HS_MotionParaLib->HS_SetWeaveNum(m_iWeaveNum);
						bWeaveNumCntFlag = true;
					}
				}
			}
			else
			{
				dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + m_dWeaveSVel*m_dWeaveTCur;
			}				
		}			
		else
		{			
			bWeaveNumCntFlag = false;
			if(m_tHS_WeavePara.eWeaveType == WEAVE_SINE)
			{
				if(m_bDStopMode)
					WeaveMove_StopSine(dWeavePos);
				else
				{
					if(m_eWeaveState == WEAVE_HALFUP)
					{
						//焊缝下半部分完成
						m_iWeaveNum++;
						//m_HS_MotionParaLib->HS_SetWeaveNum(m_iWeaveNum);
					}
					WeaveMove_NormalSine(dWeavePos);
				}
			}
			else if(m_tHS_WeavePara.eWeaveType == WEAVE_MOONB||m_tHS_WeavePara.eWeaveType == WEAVE_MOONF)
			{
				WeaveMove_StopSine(dWeavePos);
			}
			else if(m_tHS_WeavePara.eWeaveType == WEAVE_L)
			{
				WeaveMove_L(dWeavePos);
			}
		}
	}
	dWeavePosSave = dWeavePos;
	return 0;
}
/************************************************
函数功能：摆焊状态机切换以及获取点位输出【L型】
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMove_L(double &dWeavePos)
{
	double dTMove = m_dWeavePeriod/4;
	if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
	{
		if(dTMove > m_dTLast&&!m_bWeaveStopFlag)
		{	
			m_bWeaveStopFlag = true;
			m_dWeaveStopAmp = 0;
			LOG_ALGO("Weave OFF,TMove = %.3lf,TLast = %.3lf!",dTMove,m_dTLast);
			dTMove = m_dTLast;
			if(dTMove < m_dWeavePeriod/4)
				dTMove = m_dWeavePeriod/4;
		}
	}
	else if(m_tHS_WeavePara.eWeaveDType == WEAVE_DMOVE)
	{
		double dTAdd = m_dWeavePeriod/4;
		double dTWait = 0;

		if(m_eWeaveState == WEAVE_DOWN)
		{
			if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dLDT > m_dCycle)
				dTWait = m_tHS_WeavePara.dLDT;
		}
		else if(m_eWeaveState == WEAVE_UP)
		{
			if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dRDT > m_dCycle)
				dTWait = m_tHS_WeavePara.dRDT;
		}
		else if(m_eWeaveState == WEAVE_MIDUP)
		{
			dTWait = m_tHS_WeavePara.dLDT;
		}
		else if(m_eWeaveState == WEAVE_MIDDOWN)
		{
			dTWait = m_tHS_WeavePara.dRDT;
		}

		if((dTMove + dTAdd + dTWait > m_dAheadTAll)&&!m_bWeaveStopFlag)
		{	
			m_bWeaveStopFlag = true;
			m_dWeaveStopAmp = 0;
			LOG_ALGO("Weave OFF,State:%d,TMove = %.3lf,dTAdd = %.3lf,dTWait = %.3lf,AheadTAll = %.3lf!",
				m_eWeaveState,dTMove,dTAdd,dTWait,m_dAheadTAll);

			if(m_dAheadTAll > dTWait)
				dTMove = m_dAheadTAll - dTWait;
			else
				dTMove = m_dWeavePeriod/4;
		}
	}

	switch(m_eWeaveState)
	{
	case WEAVE_DOWN:
		if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dLDT > m_dCycle)
		{
			m_bWeaveWaitFlag = true;
			m_dWeaveWaitTime = m_tHS_WeavePara.dLDT;
			dWeavePos = m_dWeaveSPos + m_dWeavePAll;
			m_dWeaveTCur = 0;
			m_bWeaveStopFlag = false;
		}
		else
		{
			m_bWeaveWaitFlag = false;
			m_dWeaveSPos = m_dWeaveAmp;
			if(m_bWeaveStopFlag)
			{
				m_dWeavePAll = -(m_dWeaveAmp + m_dWeaveStopAmp);					
				m_eWeaveState = WEAVE_DONE;
				m_bWeaveStopDir = true;
			}
			else
			{
				m_dWeavePAll = -m_dWeaveAmp;
				m_eWeaveState = WEAVE_MIDDOWN;
			}
			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_MIDDOWN:
		if(m_bWeaveStopFlag)
		{
			m_eWeaveState = WEAVE_DONE;
			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
		}
		else
		{

			m_dWeaveSPos = 0;
			m_dWeavePAll = -m_dWeaveAmp;
			m_eWeaveState = WEAVE_UP;

			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_UP:
		if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dRDT > m_dCycle)
		{
			m_bWeaveWaitFlag = true;
			m_dWeaveWaitTime = m_tHS_WeavePara.dRDT;
			dWeavePos = m_dWeaveSPos + m_dWeavePAll;
			m_dWeaveTCur = 0;
			m_bWeaveStopFlag = false;
		}
		else
		{
			m_bWeaveWaitFlag = false;
			m_dWeaveSPos = -m_dWeaveAmp; 
			if(m_bWeaveStopFlag)
			{
				m_dWeavePAll = (m_dWeaveAmp + m_dWeaveStopAmp);					
				m_eWeaveState = WEAVE_DONE;
				m_bWeaveStopDir = false;
			}
			else
			{
				m_dWeavePAll = m_dWeaveAmp;
				m_eWeaveState = WEAVE_MIDUP;
			}

			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_MIDUP:
		if(m_bWeaveStopFlag)
		{
			m_eWeaveState = WEAVE_DONE;
			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
		}
		else
		{
			m_dWeaveSPos = 0; 
			m_dWeavePAll = m_dWeaveAmp;
			m_eWeaveState = WEAVE_DOWN;
		
			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_DONE:
		m_bWeaveFlag = false;
		m_dWeavePAll = 0;
		LOG_ALGO("Weave OFF,StopAmp = %.3lf,Dir = %d!",m_dWeaveStopAmp,m_bWeaveStopDir);
		break;
	default:
		break;
	}	
	return 0;
}
/************************************************
函数功能：摆焊状态机切换以及获取点位输出【不停止的sine】
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMove_StopSine(double &dWeavePos)
{	
	double dTMove = m_dWeavePeriod/2;
	if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
	{
		//检测是否有下一段点位数据
		int iBuffNext = (m_iIndex + 1)%MaxBuffSize;

		HS_WeavePara tWeavePara = m_tGTrajData[iBuffNext].tMotionData.tBaseMoveData[m_iGroupNum].tWeavePara;

		if(m_eWeaveState != WEAVE_DONE)
		{
			bool bNextStopModeFlag = false;
			if((tWeavePara.dLDT > m_dCycle||tWeavePara.dRDT > m_dCycle)&&tWeavePara.eWeaveDType == WEAVE_DSTOP&&
				tWeavePara.eWeaveType == WEAVE_SINE||tWeavePara.eWeaveType == WEAVE_MOONB||tWeavePara.eWeaveType == WEAVE_MOONF)
			{
				bNextStopModeFlag = true;
			}

			if(m_tGTrajData[iBuffNext].tMotionData.iLineNum != -1&&m_tGTrajData[iBuffNext].tMotionData.tBaseMoveData[m_iGroupNum].eTrajType != MP_Joint&&tWeavePara.bWeaveEn && fabs(tWeavePara.dAmplitude) > Eps&&tWeavePara.dFrequency > Eps&&
				bNextStopModeFlag&&!m_bStopSineNextFlag)	
			{
				m_bStopSineNextFlag = true;
				LOG_ALGO("Stop Sine Have Next Smooth!");
			}
		}

		if(dTMove > m_dTLast&&!m_bWeaveStopFlag)
		{	
			m_bWeaveStopFlag = true;
			m_dWeaveStopAmp = 0;			
			LOG_ALGO("Weave OFF,TMove = %.3lf,TLast = %.3lf!",dTMove,m_dTLast);
			dTMove = m_dTLast;
			if(dTMove < m_dWeavePeriod/3)
				dTMove = m_dWeavePeriod/3;

			if(m_bStopSineNextFlag)
				dTMove = m_dWeavePeriod/2;
		}
	}
	else if(m_tHS_WeavePara.eWeaveDType == WEAVE_DMOVE)
	{
		double dTAdd = m_dWeavePeriod/2;
		double dTWait = 0;
		if(m_eWeaveState == WEAVE_DOWN)
		{
			if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dLDT > m_dCycle)
				dTWait = m_tHS_WeavePara.dLDT;
		}
		else if(m_eWeaveState == WEAVE_UP)
		{
			if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dRDT > m_dCycle)
				dTWait = m_tHS_WeavePara.dRDT;
		}

		if((dTMove + dTAdd + dTWait > m_dAheadTAll)&&!m_bWeaveStopFlag)
		{	
			m_bWeaveStopFlag = true;
			m_dWeaveStopAmp = 0;
			LOG_ALGO("Weave OFF,TMove = %.3lf,AheadTAll = %.3lf!",dTMove,m_dAheadTAll);
			dTMove = m_dAheadTAll;
		}
	}

	switch(m_eWeaveState)
	{
	case WEAVE_DOWN:
		if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dLDT > m_dCycle)
		{
			m_bWeaveWaitFlag = true;
			m_dWeaveWaitTime = m_tHS_WeavePara.dLDT;
			dWeavePos = m_dWeaveSPos + m_dWeavePAll;
			m_dWeaveTCur = 0;
			m_bWeaveStopFlag = false;
		}
		else
		{
			m_bWeaveWaitFlag = false;
			m_dWeaveSPos = m_dWeaveAmp;
			if(m_bWeaveStopFlag)
			{
				if(m_bStopSineNextFlag)
				{
					m_dWeavePAll = -m_dWeaveAmp*2;
					m_bStopSineWaitTFlag = true;
				}
				else
					m_dWeavePAll = -(m_dWeaveAmp + m_dWeaveStopAmp);					
				m_eWeaveState = WEAVE_DONE;
				m_bWeaveStopDir = true;
			}
			else
			{
				m_dWeavePAll = -m_dWeaveAmp*2;
				m_eWeaveState = WEAVE_UP;
			}
			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_UP:
		if(!m_bWeaveWaitFlag&&m_tHS_WeavePara.dRDT > m_dCycle)
		{
			m_bWeaveWaitFlag = true;
			m_dWeaveWaitTime = m_tHS_WeavePara.dRDT;
			dWeavePos = m_dWeaveSPos + m_dWeavePAll;
			m_dWeaveTCur = 0;
			m_bWeaveStopFlag = false;
		}
		else
		{
			m_bWeaveWaitFlag = false;
			m_dWeaveSPos = -m_dWeaveAmp; 
			if(m_bWeaveStopFlag)
			{
				if(m_bStopSineNextFlag)
				{
					m_dWeavePAll = m_dWeaveAmp*2;
					m_bStopSineWaitTFlag = true;
				}
				else
					m_dWeavePAll = (m_dWeaveAmp + m_dWeaveStopAmp);					
				m_eWeaveState = WEAVE_DONE;
				m_bWeaveStopDir = false;
			}
			else
			{
				m_dWeavePAll = m_dWeaveAmp*2;
				m_eWeaveState = WEAVE_DOWN;
			}

			if(m_tHS_WeavePara.eWeaveDType == WEAVE_DSTOP)
			{
				WeavePlan(m_dWeavePAll,dTMove);
				WeaveMainPlan(dTMove,m_bWeaveStopFlag);
			}
			else
			{
				WeavePlan(m_dWeavePAll,dTMove);
			}
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_STOP:
		if(m_dWeaveSPos < 0)
		{
			m_dWeaveSPos = m_dWeaveStopAmp;
			m_dWeavePAll = -m_dWeaveStopAmp;
		}
		else
		{
			m_dWeaveSPos = -m_dWeaveStopAmp;
			m_dWeavePAll = m_dWeaveStopAmp;
		}
		m_eWeaveState = WEAVE_DONE;
		if(m_dWeaveStopAmp < 0.02)
		{
			m_bWeaveFlag = false;
			m_dWeavePAll = 0;
			LOG_ALGO("Weave OFF,StopAmp = %.3lf!",m_dWeaveStopAmp);
		}
		else
		{
			WeavePlan(m_dWeavePAll);
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_DONE:
		
		if(!m_bStopSineNextFlag)
		{
			m_bWeaveFlag = false;
			m_dWeavePAll = 0;
			LOG_ALGO("Weave OFF Done,Dir = %d!",m_bWeaveStopDir);
		}
		else
		{
			dWeavePos = -m_dWeaveSPos;

			if(!m_bWeaveWaitFlag)
			{
				if(m_bWeaveStopDir)							
					m_dWeaveWaitTime = m_tHS_WeavePara.dRDT;				
				else 
					m_dWeaveWaitTime = m_tHS_WeavePara.dLDT;
				m_bWeaveWaitFlag = true;
				m_dWeaveTCur = 0;
				m_bWeaveStopFlag = false;
				LOG_ALGO("Weave OFF Last Wait!");
			}
			else
			{
				m_bWeaveFlag = false;
				m_bStopSineWaitTFlag = false;
				LOG_ALGO("Weave OFF Done,Dir = %d!",m_bWeaveStopDir);
			}
		}		
		break;
	default:
		break;
	}	
	return 0;
}

/************************************************
函数功能：摆焊状态机切换以及获取点位输出【不停止的sine】
参    数：
		dWeavePos----摆焊坐标点位
返 回 值：错误码
*************************************************/
int HS_Int_Move::WeaveMove_NormalSine(double &dWeavePos)
{
	//如果当前段平滑为0，需要根据剩余时间修改最后的波峰幅值
	double TError=0,TStop=0;
	static double dStopTime = m_dWeavePeriod/2;

	//摆动剩余时间，如果按照幅度为m_dWeaveAmp计算时间：T(0->A) + T(A->0) + T(0->0)
	double dTRemain = 2*m_dWeaveAmp/m_dWeaveVel + sqrt(3*m_dWeaveAmp/2/(m_dWeaveAccMAX*2/3))*2 + m_dWeavePeriod/2;
	//如果当前为启动A，增加一个（A->0)
	if(m_eWeaveState == WEAVE_STARTDOWN)
		dTRemain += 2*m_dWeaveAmp/m_dWeaveVel;

	if(dTRemain > m_dAheadTAll&&!m_bWeaveStopFlag)
	{	
		m_bWeaveStopFlag = true;
		m_dWeaveStopAmp = 0;
		//停止时间：总时间-T（0->A)
		dStopTime = m_dAheadTAll - 2*m_dWeaveAmp/m_dWeaveVel;
		//停止时间：总时间-T（0->A)*2
		if(m_eWeaveState == WEAVE_STARTDOWN)
			dStopTime -= 2*m_dWeaveAmp/m_dWeaveVel;

		LOG_ALGO("Stop:AheadTime = %.3lf,StopTime = %.3lf!",m_dAheadTAll,dStopTime);
	}
	//
	switch(m_eWeaveState)
	{
	case WEAVE_DOWN:
		m_dWeaveSPos = m_dWeaveAmp;
		if(m_bWeaveStopFlag)
		{
			m_dWeavePAll = -(m_dWeaveAmp + m_dWeaveStopAmp);					
			m_eWeaveState = WEAVE_DONE;
			WeavePlan(m_dWeavePAll,dStopTime);
		}
		else
		{
			m_dWeavePAll = -m_dWeaveAmp*2;
			m_eWeaveState = WEAVE_UP;
			WeavePlan(m_dWeavePAll);
		}
		//WeavePlan(m_dWeavePAll);
		dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		break;
	case WEAVE_UP:
		m_dWeaveSPos = -m_dWeaveAmp; 
		if(m_bWeaveStopFlag)
		{
			m_dWeavePAll = (m_dWeaveAmp + m_dWeaveStopAmp);					
			m_eWeaveState = WEAVE_DONE;
			WeavePlan(m_dWeavePAll,dStopTime);
		}
		else
		{
			m_dWeavePAll = m_dWeaveAmp*2;
			m_eWeaveState = WEAVE_DOWN;
			WeavePlan(m_dWeavePAll);
		}
		//WeavePlan(m_dWeavePAll);
		dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		break;
	case WEAVE_STARTDOWN:
		m_dWeaveSPos = m_dWeaveAmp;
		m_eWeaveState = WEAVE_HALFDOWN;
		WeavePlan(0,-m_dWeaveVel,2*m_dWeaveAmp/m_dWeaveVel);
		dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + m_dWeaveSVel*m_dWeaveTCur;
		break;
	case WEAVE_HALFDOWN:
		m_dWeaveSPos = 0;
		/**************************************************
		当前处于A+到0的0位置，计算当前剩余时间TE 和 需要完成当前波峰停止的时间TStop 进行对比，
		如果TE < TStop，则进行迭代重新修正波峰的处理
		**************************************************/
		TError = m_dAheadTAll;
		TStop = 2*m_dWeaveAmp/m_dWeaveVel + sqrt(3*m_dWeaveAmp/2/(m_dWeaveAccMAX*2/3))*2;
		if(TError < TStop)
		{
			int icnt=0;
			double WeaveAmpMin = 0,WeaveAmpMax = m_dWeaveAmp,WeaveAmpTemp = 0;
			while(1)
			{
				WeaveAmpTemp = (WeaveAmpMin+WeaveAmpMax)/2;
				double TS = 2*WeaveAmpTemp/m_dWeaveVel + sqrt(3*WeaveAmpTemp/2/(m_dWeaveAccMAX*2/3))*2;
				if(TS > TError)
					WeaveAmpMax=WeaveAmpTemp;
				else
					WeaveAmpMin=WeaveAmpTemp;
				if(icnt>10 || fabs(TS-TError)<Eps)
				{
					m_dWeaveAmp = WeaveAmpTemp;
					break;
				}
				icnt++;
			}
		}
		//////////////////////////////////////////////////////
		if(m_bWeaveStopFlag)
		{
			m_eWeaveState = WEAVE_UP;
			WeavePlan(-m_dWeaveVel,0,2*m_dWeaveAmp/m_dWeaveVel);
		}
		else
		{
			m_eWeaveState = WEAVE_HALFUP;
			WeavePlan(-m_dWeaveVel,m_dWeaveVel,m_dWeavePeriod/2);
		}	
		dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + m_dWeaveSVel*m_dWeaveTCur;
		break;
	case WEAVE_HALFUP:
		/**************************************************
		当前处于A+到0的0位置，计算当前剩余时间TE 和 需要完成当前波峰停止的时间TStop 进行对比，
		如果TE < TStop，则进行迭代重新修正波峰的处理
		**************************************************/
		TError = m_dAheadTAll;
		TStop = 2*m_dWeaveAmp/m_dWeaveVel + sqrt(3*m_dWeaveAmp/2/(m_dWeaveAccMAX*2/3))*2;
		if(TError < TStop)
		{
			int icnt=0;
			double WeaveAmpMin = 0,WeaveAmpMax = m_dWeaveAmp,WeaveAmpTemp = 0;
			while(1)
			{
				WeaveAmpTemp = (WeaveAmpMin+WeaveAmpMax)/2;
				double TS = 2*WeaveAmpTemp/m_dWeaveVel + sqrt(3*WeaveAmpTemp/2/(m_dWeaveAccMAX*2/3))*2;
				if(TS > TError)
					WeaveAmpMax=WeaveAmpTemp;
				else
					WeaveAmpMin=WeaveAmpTemp;
				if(icnt>10 || fabs(TS-TError)<Eps)
				{
					m_dWeaveAmp = WeaveAmpTemp;
					break;
				}
				icnt++;
			}
		}
		//////////////////////////////////////////////////////
		m_dWeaveSPos = 0;
		if(m_bWeaveStopFlag)
		{
			m_eWeaveState = WEAVE_DOWN;
			WeavePlan(m_dWeaveVel,0,2*m_dWeaveAmp/m_dWeaveVel);
		}
		else
		{
			m_eWeaveState = WEAVE_HALFDOWN;
			WeavePlan(m_dWeaveVel,-m_dWeaveVel,m_dWeavePeriod/2);
		}
		dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb) + m_dWeaveSVel*m_dWeaveTCur;
		break;
	case WEAVE_STOP:
		if(m_dWeaveSPos < 0)
		{
			m_dWeaveSPos = m_dWeaveStopAmp;
			m_dWeavePAll = -m_dWeaveStopAmp;
		}
		else
		{
			m_dWeaveSPos = -m_dWeaveStopAmp;
			m_dWeavePAll = m_dWeaveStopAmp;
		}
		m_eWeaveState = WEAVE_DONE;
		if(m_dWeaveStopAmp < 0.02)
		{
			m_bWeaveFlag = false;
			m_dWeavePAll = 0;
			LOG_ALGO("Weave OFF,StopAmp = %.3lf!",m_dWeaveStopAmp);
		}
		else
		{
			//(无效)
			/**************************************************
			当前处于A+到0的A+位置，计算当前剩余时间TE 和 需要完成当前波峰停止的时间TStop 进行对比，
			如果TE < TStop，则进行迭代重新修正加速度值的处理
			**************************************************/
			//TError = m_dAheadTAll;
			//TStop = sqrt(3*fabs(m_dWeavePAll)/2/(m_dWeaveAccMAX*2/3))*2;
			//if(TError < TStop)
			//{
			//	int icnt=0;
			//	double WeaveAccMin = 0,WeaveAccMax = 4*m_dWeaveAccMAX,WeaveAccTemp = 0;
			//	while(1)
			//	{
			//		WeaveAccTemp = (WeaveAccMin+WeaveAccMax)/2;
			//		double TS = sqrt(3*fabs(m_dWeavePAll)/2/(WeaveAccTemp*2/3))*2;
			//		if(TS > TError)
			//			WeaveAccMin=WeaveAccTemp;
			//		else
			//			WeaveAccMax=WeaveAccTemp;
			//		if(icnt>10 || fabs(TS-TError)<Eps)
			//		{
			//			m_dWeaveAccMAX = WeaveAccTemp;
			//			break;
			//		}
			//		icnt++;
			//	}
			//}
			////////////////////////////////////////////////////////
			WeavePlan(m_dWeavePAll);
			dWeavePos = m_dWeaveSPos + m_dWeaveTCur*m_dWeaveTCur*m_dWeaveTCur*(m_dWeaveKa*m_dWeaveTCur + m_dWeaveKb);
		}
		break;
	case WEAVE_DONE:
		m_bWeaveFlag = false;
		m_dWeavePAll = 0;
		LOG_ALGO("Weave OFF,StopAmp = %.3lf!",m_dWeaveStopAmp);
		break;
	default:
		break;
	}	
	return 0;
}

/************************************************
函数功能：获取摆焊停止的主运动关节位置
参    数：
		dMainJPos----主运动停止的关节位置
返 回 值：
		是否为摆焊停止
*************************************************/
bool HS_Int_Move::GetWeaveStopMainPos(double dMainJPos[MaxAxisNum])
{
	memcpy(dMainJPos,m_dWeaveStopMainJPos,sizeof(double)*MaxAxisNum);
	
	if(m_bWeaveFlag)
		return m_bStopDoneFlag;
	
	return false;
}