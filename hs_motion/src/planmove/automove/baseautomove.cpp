#include "HS_AutoMove.h"
#include <iostream>
#include "baseautomove.h"


namespace hsc3
{
namespace algo
{
    BaseAutoMove::BaseAutoMove(MotionPara *para,double dCycle,int iInterMultCnt) : AutoMove(para,dCycle,iInterMultCnt)
    {
        m_HS_AutoMove = new HS_AutoMove(para,dCycle,iInterMultCnt);
    }

    BaseAutoMove::~BaseAutoMove()
    {
        delete m_HS_AutoMove;
    }

    int BaseAutoMove::execPrintKeyInfo()
    {
        return m_HS_AutoMove->execPrintKeyInfo();
    }

    void BaseAutoMove::execReset()
    {
        m_HS_AutoMove->execReset();
    }
	
	int BaseAutoMove::execPrehandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex)
	{
		return m_HS_AutoMove->execPrehandle(tGroupMotionData,tTrajData,iIndex);
	}  

	int BaseAutoMove::execPlanMove(GroupTrajData *tTrajData,int iIndex,double dRatio,HS_GroupJPos &tHS_GroupJPos)
	{
		return m_HS_AutoMove->execPlanMove(tTrajData,iIndex,dRatio,tHS_GroupJPos);
	}

	int BaseAutoMove::execStopPlan()
    {
        return m_HS_AutoMove->execStopPlan();
    }

	int BaseAutoMove::execStopRestartPlan(double dRatio,HS_GroupJPos &tRealJPos)
    {
        return m_HS_AutoMove->execStopRestartPlan(dRatio,tRealJPos);
    }

    int BaseAutoMove::setRatio(double ratio)
    {
        return m_HS_AutoMove->setRatio(ratio);
    }

	HS_MStatus BaseAutoMove::execIntMove(IntData &intdata,int &iErrorId)
    {
        return m_HS_AutoMove->execIntMove(intdata,iErrorId);
    }

    int BaseAutoMove::execGetJoint(int *joint) 
    {
        return m_HS_AutoMove->execGetJoint(joint);
    }

	bool BaseAutoMove::GetSmoothAheadFlag() 
	{
		return m_HS_AutoMove->execGetSmoothAheadFlag();
	}

	int BaseAutoMove::execGetCurMoveId() 
	{
		return m_HS_AutoMove->execGetCurMoveId();
	}

	bool BaseAutoMove::HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB)
	{
		return m_HS_AutoMove->HS_RepeatPosCheck(tPosA,tPosB);
	}
}
}