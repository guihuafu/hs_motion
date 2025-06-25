#pragma once

#include "manualmove.h"
#include "motionpara.h"

#ifndef DLL_EXPORT
#ifndef _LINUX_
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT  __attribute__((visibility("default")))
#endif
#endif

class HS_Int_Manual;
class HS_Kinematics;

namespace hsc3
{
namespace algo
{

// 点动算法接口--对外接口
class DLL_EXPORT BaseManualMove : public ManualMove
{
public:
    DLL_EXPORT BaseManualMove(MotionPara *para,double dCycle);
    DLL_EXPORT ~BaseManualMove();
    DLL_EXPORT virtual int execPrintKeyInfo();
	DLL_EXPORT virtual int Plan(HS_GroupJPos &tHS_GroupJPos, ManualPara tManualPara);
	DLL_EXPORT virtual HS_MStatus Move(int &iErrorId,HS_GroupJPos &tHS_GroupJPos);
	DLL_EXPORT virtual int StopPlan();   

public:
    MotionPara *mMotionPara;
	HS_Int_Manual *m_HS_Int_Manual;
	HS_Kinematics *m_HS_Kinematics;
};
}
}