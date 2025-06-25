/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Joint.h
* 摘    要：关节运动

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_INT_JOINT_H
#define _HS_INT_JOINT_H

#include "HS_Int_Move.h"
#include "HS_Kinematics.h"

class HS_Int_Joint:public HS_Int_Move
{
public:
	HS_Int_Joint(HS_Kinematics *pKinematics);
	HS_Int_Joint(HS_GroupKin *pGroupKin);
	~HS_Int_Joint();
	int ResetData();
	int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum); 
	int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
    int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll);
	int Stop();
	int setRatio(double dRatio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
	HS_MStatus Move(int &iErrorId,bool bLastCycle = false);
private:
	int GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint,bool bStartMoveFlag);
	int GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint);
    int NearestAngleMoveHandle(Para_PreHandle &tPH_Joint,PosInfo sEndPos);
    int TurnAlarmHandle(HS_Revolve tRevolve,double dSetJPos[6],double dEndJPos[6]);
	int GetMoveDis(Para_PreHandle &tPH_Joint);
	int CheckEndPosLimit(Para_PreHandle &tPH_Joint);
	int AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Joint);
	int GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio);
	int GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Joint,double dRatio);
	int TrajectoryPlan(void);
	int VelPlan(void);
	int StartPosCheckAndReplan(void);
private:
    unsigned char m_eStartState;                			//起点的形态，可能会与结束点不同
};

#endif