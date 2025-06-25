/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_SLine.h
* 摘    要:直线运动【小线段方式】

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_INT_SLINE_H
#define _HS_INT_SLINE_H

#include "HS_Int_Move.h"
#include "HS_Kinematics.h"
#include "BezierCurve.h"
#include <vector>

#define MAXCNT 4


class HS_Int_SLine:public HS_Int_Move
{
public:
	HS_Int_SLine(HS_Kinematics *pKinematics);
	HS_Int_SLine(HS_GroupKin *pGroupKin);
	~HS_Int_SLine();
	int ResetData();
	int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum); 
	int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
    int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll);
	int Stop();
	int setRatio(double dRatio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
private:	
	HS_MStatus Move(int &iErrorId,bool bLastCycle = false);
	HS_MStatus StopMoveHandle(double dMovePos[MaxAxisNum]);
	void GetToolWorkNum(BaseMoveData tMoveData);
	int GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int GetMoveDis(Para_PreHandle &tPH_Line);
    int HandleParaWithPre(Para_PreHandle &tPH_Line);
    int MoveAdjust(Para_PreHandle &tPH_Line);
	int JPosAutoHandle(Para_PreHandle &tPH_Line,double dEJPos[6],int iAxis);
	int JPosPrediction(Para_PreHandle &tPH_Line,double dEJPos[6]);
    int JPosPrediction_Scara(Para_PreHandle &tPH_Line,double dEJPos[6]);
	int CheckPosLimit(Para_PreHandle &tPH_Line);
	int AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Line);
	int AutoAdjustJAcc(Para_PreHandle &tPH_Line);
	int GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio);
	int GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Line,double dRatio);
    int GetDynData(GroupTrajData *trajout,int index,Para_PreHandle &tPH_Line,double dRatio);
	int TrajectoryPlan(void);
    int DynPlan();
	int MoveDynPlan();
	int DynLookAheadHandle(int iLookAhead);
	int VelPlan(void);
    bool CheckSmallLimeUpMode(SyncPara tSyncCur,SyncPara tSyncNex);
    int SmoothHandle_Forward(int iLookAhead,bool bStart = false);
    int SmoothHandle_Backward(int iLookAhead);
    double GetConVTime(SyncPara &tSyncCur);
    double MaxKAccCalc(SyncPara &tSyncCur,SyncPara &tSyncNex);
    double S2STSmoothCalc_Up(SyncPara &tSyncCur,SyncPara &tSyncNex,double dTMin,double dTMax,double dKLimit,bool bBack = false,bool bStart = false);
    double S2STSmoothCalc_Down(SyncPara &tSyncCur,SyncPara &tSyncNex,double dTMax,double dKLimit);
	int GetPosByLength(Para_PreHandle &tPH_Line,double dLength,double dCPos[MaxAxisNum]);
	int GetPosByMixPos(Para_PreHandle &tPH_Line,Para_PreHandle &tPH_LinePre,double dCPos[MaxAxisNum]);
    int SLAutoPara(Para_PreHandle &tPH_Line);
    void BezierSmoothHandle(int iAheadIndex,bool &bBezierHandle);
	int LengthAheadHandle();
	int SLMaxJAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJAcc);
	int SLCAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dCAcc[2]);
	void DivLengthHandle();
	double GetQYDegree(double *dJPos);
private:
	double m_dRMPos[5][4];									//当前齐次坐标位置
	double m_dRestartPos[SpaceAxisNum];
    double m_dMoveVelPre[MaxAxisNum];
	double m_dMoveAccPre[MaxAxisNum];
    SyncPara m_tSyncPre;                                    //上一段规划

    HS_VelPlan_Para **m_HS_VelPlan_ParaPre;		            //速度规划指针

    unsigned char m_eStartState;                			//起点的形态，可能会与结束点不同
	bool m_bSLLookAheadPlanFlag;

    /********************************************************************************/
	SyncPara m_tSyncBase;                                   //当前规划基础规划【未进行动态规划】
    static double m_dDynLOffset;                            
    static double m_dDynPreEPos[MaxAxisNum];
    static double m_dDynPlanLength;                         //动态规划长度
    static double m_dDivFoundJPos[MaxAxisNum];
    static DynPlanState m_eDynPlanState;    
    static Para_PreHandle m_tBasePH_Line;
	static Para_PreHandle m_tBasePH_LinePre;
    static double m_dDynDivLength;
	static double m_dDynDivLengthMax;						//动态调整规划端的长度，精准控制需要减小该值，提高最大速度需要放大该值
    static BezierCurve m_tBezierCurve;
    static int m_iLookAhead_SL;                             //小线段前瞻段数
    static bool m_bBezierHandle;
	static bool m_bSLStopFlag;
    /********************************************************************************/
};

#endif