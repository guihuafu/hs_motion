/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Line.h
* 摘    要:直线运动

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_INT_LINE_H
#define _HS_INT_LINE_H

#include "HS_Int_Move.h"
#include "HS_Kinematics.h"

#define MAXCNT 4

#define BRWristQYLimit	30.0+Eps

class HS_Int_Line:public HS_Int_Move
{
public:
	HS_Int_Line(HS_Kinematics *pKinematics);
	HS_Int_Line(HS_GroupKin *pGroupKin);
	~HS_Int_Line();
	int ResetData();
	int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum); 
	int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
    int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll);
	int Stop();
	int setRatio(double dRatio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
	HS_MStatus Move(int &iErrorId,bool bLastCycle = false);
private:	
	int GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int GetMoveDis(Para_PreHandle &tPH_Line);
	int PUMAWristQYCheck(Para_PreHandle &tPH_Line);
	int CheckPosLimit(Para_PreHandle &tPH_Line);
	int AutoBasePara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int AutoAdjustJAcc(Para_PreHandle &tPH_Line);
	int GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio);
	int GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Line,double dRatio);
	int TrajectoryPlan(void);
	int VelPlan(void);
	int StartPosCheckAndReplan(void);
	int CalcMovePara(Para_PreHandle &tPH_Line);
private:
	double m_dRestartPos[SpaceAxisNum];

    unsigned char m_eStartState;                			//起点的形态，可能会与结束点不同	
};

#endif