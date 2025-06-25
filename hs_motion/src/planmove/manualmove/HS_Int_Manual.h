/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Manual.h
* 摘    要：手动运动插补类

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-12-08
*			
*/

#ifndef _HS_INT_MANUAL_H
#define _HS_INT_MANUAL_H

#include "manualmoveparadef.h"
#include "AutoControl.h"
#include "HS_BasicPara.h"
#include "HS_VelPlan_Para.h"
#include "HS_Kinematics.h"
#include "HS_SetPosCheck.h"
#include "HS_FilterHandle.h"
#include "HS_GroupKin.h"

#define AutoQYVelKLimit       10.0				//奇异速度自适应倍率限制
#define WRISTSTARTPROTECT      2.0				//未开启腕部奇异保护时，点动空间时的奇异保护值

using namespace hsc3::algo;

class HS_Int_Manual
{
public:
	HS_Int_Manual();
	~HS_Int_Manual();
	int Plan(HS_GroupJPos &tHS_GroupJPos, ManualPara tManualPara);
	HS_MStatus Move(int &iErrorId,HS_GroupJPos &tHS_GroupJPos);
	int StopPlan(double dStopDis = 0,bool bVelPlanStop = true);   
private:
    int AheadLimitCheck(double dCurJPos[6],double dAheadJPos[MaxAxisNum]);
    int GetAheadJPos(double dAheadJPos[MaxAxisNum]);
    int StartPosWristQYCheck(double dCurJPos[6]);
    int ManualFilterHandle(double *dRJPos,HS_MStatus &eMStatus);

	HS_GroupKin *m_HS_GroupKin;
    HS_BasicPara *m_HS_BasicPara;
	HS_Kinematics *m_HS_Kinematics;
    HS_FilterHandle *m_HS_FilterHandle;
	bool m_bSyncMoveFlag;

	double *m_dJVelPara;
	double *m_dJAccPara;
	double m_dCycle;
    LimitPara *m_tLimitPara;
    HS_SetPosCheck *m_HS_SetPosCheck;
    bool m_bWristQYFlag;

    double m_dJogVfac;                                          //点动速度倍率系数

	ManualPara m_tManualPara;
	int m_iAxisNum;												//运行轴号
	bool m_bHandCoord;										    //关节【空间规划】
	double m_dHandPos[MaxAxisNum];							    //手动模式下更新位置【关节、空间】
	double m_dInitHandCPos[MaxAxisNum];						    //空间手动初始位置
	double m_dInitHandJPos[MaxAxisNum];						    //关节手动初始位置
	double m_dRJPos[MaxAxisNum];								//实时关节位置
    bool m_bPlanFlag;                                           //是否进行了运动规划
    double m_dKCVel;
    double m_dAheadJPosDis;
    bool m_bAheadJErrorFlag;
	int m_iToolNum;

	bool m_bCoorperFlag;										// 变位机协同
	double m_dBaseTCMPos[5][4];

	VelPlanPara m_tVelPlanPara;	
	HS_VelPlan_Para *m_HS_VelPlan_Para;	
    FilterControl *m_tFilterControl;

	double m_dMovePos;
	double m_dMoveVel;
	double m_dMoveAcc;

	//停止相关
	double m_dStopDec;
    double m_dStopJerk;
	double m_dTStop;
	double m_dStopKa; 
	double m_dStopKb; 
	double m_dStopKc; 
	double m_dStopDis;
	double m_dStopSPos;
	bool m_bStopFlag;
	double m_dTCur;
    bool m_bMoveErrorFlag;                                      //运动过程中发生了报错

};
#endif