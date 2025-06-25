/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_VelPlan_Para.h
* 摘    要：速度规划抛物线插补

* 当前版本：3.0
* 作    者：cyh
* 完成日期：
*			
*/
#ifndef _HS_VELPLAN_PARA_H
#define _HS_VELPLAN_PARA_H

#include "AutoControl.h"
#include "HS_FilterHandle.h"

using namespace hsc3::algo;

class HS_VelPlan_Para
{
public:
	HS_VelPlan_Para(double dCycle);	
	~HS_VelPlan_Para();
	int Plan(VelPlanPara tPara);	
    int Plan(VelPlanPara tPara,FilterPara tFilterPara);
    int RePlan(VelPlanPara tPara);
	int Ratio(VelPlanPara tPara);
	HS_MStatus Move(double &dPos);	
	int GetVel(double &dVel);
	int GetAcc(double &dAcc);
    int GetOrignPos(double &dPos);
    int GetFilterVel(double &dVel);
    int GetFilterAcc(double &dAcc);
	int SyncStop(void);	
    int StopPlan(double dTStop);
	double RePlanByTime(double &dTRePlan,bool bLastPlanFlag);
private:	
	void Reset(void);

    HS_FilterHandle m_HS_FilterHandle;
    bool m_bFilterOpenFlag;

	VelPlanPara m_tPlan;			//规划参数	
	double m_dTCur;				//当前运行时间

	double m_dKaAcc;			//加速系数a
	double m_dKbAcc;			//加速系数b
	double m_dKcAcc;			//加速系数c
	double m_dPosAcc;			//加速完成位置
	double m_dPosAccA;

	double m_dKaDec;			//减速系数
	double m_dKbDec;			//减速系数

	double m_dKaAccA;			
	double m_dKbAccA;			

	double m_dPos;
	double m_dVel;
	double m_dAcc;
	double m_dSAcc;				//调速启动加速度
	double m_dSVel;				//调速启动速度
	double m_dSPos;				//调速启动位置
	double m_dCycle;

    double m_dFilterPos;
    double m_dFilterVel;
    double m_dFilterAcc;

	bool m_bRePlanFlag;			//基于时间的重新规划标识
	double m_dTLast;			//剩余运动时间
	double m_dTAcc;
};

#endif