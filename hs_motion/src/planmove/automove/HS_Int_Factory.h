/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Factory.h
* 摘    要：插补运动工厂类

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/

#ifndef _HS_INT_FACTORY_H
#define _HS_INT_FACTORY_H

#include "HS_Int_Move.h"
#include "HS_Int_Joint.h"
#include "HS_Int_Line.h"
#include "HS_Int_Circle.h"
#include "HS_Int_SLine.h"

class HS_Int_Factory
{
public:
	HS_Int_Factory(HS_MOVETYPE eMoveType,HS_GroupKin *pGroupKin);
	~HS_Int_Factory();
	int ResetData();
	int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum);
	int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
    int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll);
	SyncPara GetSyncPara();
	int SetSyncParaPlan(SyncPara tSyncPara);
	int Stop();
	int setRatio(double ratio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);	
    int MixSmoothMove(IntData &intdataPre,IntData &intdata);
    bool GetSmoothNextFlag();
    bool GetSLPlanFlag();
	bool GetHalfSmoothFlag();
	bool GetSmoothAheadFlag();
	int GetMasterCPos(double dMasterCPos[10][MaxAxisNum]);
	int SetMasterCPos(double dMasterCPos[10][MaxAxisNum]);
	double CalcStopTime();
	int StopPlanByTime(double dTStop);
	int GetRatioPara(double dRatio,double dTime[TIMECNT]);
	int SetRatioPara(double dRatio,double dTime[TIMECNT],bool bRatioOKFlag);
	bool GetWeaveStopMainPos(double dMainJPos[MaxAxisNum]);
private:
	HS_Int_Move *m_HS_Int_Move;	
public:
    int m_iLineNum;
    HS_MOVETYPE m_Hs_MoveType;	
};
#endif