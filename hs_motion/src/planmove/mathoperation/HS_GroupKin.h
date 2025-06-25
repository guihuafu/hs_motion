/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_GroupKin.h
* 摘    要：算法的多轴组管理

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2024-10-30
*			
*/
#ifndef _HS_GROUPKIN_H
#define _HS_GROUPKIN_H

#include "motionpara.h"
#include "MathOperation.h"
#include "HS_Kinematics.h"

using namespace hsc3::algo;


class HS_GroupKin:public HS_Math
{
public:
	HS_GroupKin();
	~HS_GroupKin();

	HS_Kinematics* GetKinematicsByNum(int iGroupNum);
	bool SetGroupNum(int iGroupNum,HS_GroupRel tHS_GroupRel);
	int HandSyncMove(HS_GroupJPos &tHS_GroupJPos);
	int HandSyncInit(HS_GroupJPos &tHS_GroupJPos,int iToolNum[MAXGROUPNUM]);

public:
	HS_Kinematics *m_HS_Kinematics[MAXGROUPNUM];

private:
	int m_iGroupNum;				//当前运动组号
	int m_iSlaveNum;				//从机组号
	HS_GroupRel m_tHS_GroupRel;
	double m_dHandMSMPosInit[4][4];
	int m_iGroupToolNum[MAXGROUPNUM];

};


#endif