#pragma once

#include "automove.h"
#include "motionpara.h"

#ifndef DLL_EXPORT
#ifndef _LINUX_
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif
#endif

class HS_AutoMove;

namespace hsc3
{
namespace algo
{
// 规划算法接口--对外接口
class DLL_EXPORT BaseAutoMove : public AutoMove
{
public:
    DLL_EXPORT BaseAutoMove(MotionPara *para,double dCycle,int iInterMultCnt);
    DLL_EXPORT ~BaseAutoMove();
    DLL_EXPORT virtual int execPrintKeyInfo();
    DLL_EXPORT virtual void execReset();
	DLL_EXPORT virtual int execPrehandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex);
	DLL_EXPORT virtual int execPlanMove(GroupTrajData *tTrajData,int iIndex,double dRatio,HS_GroupJPos &tHS_GroupJPos);
	DLL_EXPORT virtual int execStopPlan();
	DLL_EXPORT virtual int execStopRestartPlan(double dRatio,HS_GroupJPos &tRealJPos);
    DLL_EXPORT virtual int setRatio(double ratio);
	DLL_EXPORT virtual HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
    DLL_EXPORT virtual int execGetJoint(int *joint);
	DLL_EXPORT virtual bool GetSmoothAheadFlag(); 
	DLL_EXPORT virtual int execGetCurMoveId();
	DLL_EXPORT virtual bool HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB);

public:
	HS_AutoMove *m_HS_AutoMove;
};
}
}