/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Circle.h
* 摘    要: 圆弧运动

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_INT_CIRCLE_H
#define _HS_INT_CIRCLE_H

#include "HS_Int_Move.h"
#include "HS_Kinematics.h"

#define MAXCNT 4

class HS_Int_Circle:public HS_Int_Move
{
public:
	HS_Int_Circle(HS_Kinematics *pKinematics);
	HS_Int_Circle(HS_GroupKin *pGroupKin);
	~HS_Int_Circle();
	int ResetData();
	int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum); 
	int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
    int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll);
	int Stop();
	int setRatio(double dRatio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
	HS_MStatus Move(int &iErrorId,bool bLastCycle = false);
private:	
	void GetToolWorkNum(BaseMoveData tMoveData);
	int GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle);
	int GetMidPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle);
	int GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle);
	int GetMoveDis(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle);
	int CheckPosLimit(Para_PreHandle &tPH_Circle);
	int AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle);
	int AutoAdjustJAcc(Para_PreHandle &tPH_Circle);
	int GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio);
	int GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Circle,double dRatio);
	int TrajectoryPlan(void);
	int VelPlan(void);
    bool CalCenter(double *p1,double *p2,double *p3,double *center);
	int StartPosCheckAndReplan(void);
private:
	double m_dRestartPos[SpaceAxisNum];

    unsigned char m_eStartState;                			//起点的形态，可能会与结束点不同
    HS_Math m_HS_Math;
    double m_dCircleR;							//半径
    double m_TCircle[4][4];						//圆变换矩阵

};

#endif