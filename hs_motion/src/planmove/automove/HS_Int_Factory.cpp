/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Factory.cpp
* 摘    要：插补运动工厂类

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*			
*/
#include "HS_Int_Factory.h"

HS_Int_Factory::HS_Int_Factory(HS_MOVETYPE eMoveType,HS_GroupKin *pGroupKin)
{
	m_Hs_MoveType = eMoveType;
	m_HS_Int_Move = NULL;
	switch(m_Hs_MoveType)
	{
	case MP_Joint:
		m_HS_Int_Move = new HS_Int_Joint(pGroupKin);
		break;
	case MP_Line:
		m_HS_Int_Move = new HS_Int_Line(pGroupKin);
		break;
	case MP_Arc:
		m_HS_Int_Move = new HS_Int_Circle(pGroupKin);
		break;
	case MP_BLine:
		m_HS_Int_Move = new HS_Int_SLine(pGroupKin);
		break;
	default:
		m_HS_Int_Move = NULL;
		break;
	}
}

HS_Int_Factory::~HS_Int_Factory()
{
	  delete m_HS_Int_Move;
	  m_HS_Int_Move = NULL;
}

int HS_Int_Factory::ResetData()
{
	if(m_HS_Int_Move == NULL)
	{
		LOG_ALGO("Error Move Type!");
		return E_MOVE_MOVETYPE;
	}
	return (m_HS_Int_Move->ResetData());
}

int HS_Int_Factory::PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum)
{
	LOG_ALGO("NewPoint = %d,LineNum = %d,MoveType = %d,CntType = %d,CoorperMove = %d,GroupNum = %d",\
		iIndex,tGroupMotionData.iLineNum,tGroupMotionData.tBaseMoveData[iGroupNum].eTrajType,
		tGroupMotionData.tBaseMoveData[iGroupNum].iCntType,(int)tGroupMotionData.tBaseMoveData[iGroupNum].bCoorperMove,iGroupNum);
	if(m_HS_Int_Move == NULL)
	{
		LOG_ALGO("Error Move Type!");
		return E_MOVE_MOVETYPE;
	}
	int iErrorId = m_HS_Int_Move->PreHandle(tGroupMotionData,tTrajData,iIndex,iGroupNum);

	return iErrorId;
}

int HS_Int_Factory::Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	if(m_HS_Int_Move == NULL)
	{
		LOG_ALGO("Error Move Type!");
		return E_MOVE_MOVETYPE;
	}
	m_iLineNum = tTrajData[iIndex].tMotionData.iLineNum;

	int iErrorId = m_HS_Int_Move->Plan(tTrajData,iIndex,iGroupNum,dRatio,tHS_GroupJPos);

	return iErrorId;
}

int HS_Int_Factory::RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll)
{
    if(m_HS_Int_Move == NULL)
    {
        LOG_ALGO("Error Move Type!");
        return E_MOVE_MOVETYPE;
    }
    return (m_HS_Int_Move->RestartPlan(dRatio,tRealJPos,eRstState,dTAll));
}

int HS_Int_Factory::Stop()
{
    if(m_HS_Int_Move == NULL)
    {
        LOG_ALGO("Error Move Type!");
        return E_MOVE_MOVETYPE;
    }
	return (m_HS_Int_Move->Stop());
}

int HS_Int_Factory::setRatio(double ratio)
{
    if(m_HS_Int_Move == NULL)
    {
        LOG_ALGO("Error Move Type!");
        return E_MOVE_MOVETYPE;
    }
	return (m_HS_Int_Move->setRatio(ratio));
}

HS_MStatus HS_Int_Factory::execIntMove(IntData &intdata,int &iErrorId)
{
    if(m_HS_Int_Move == NULL)
        return M_Error;
    iErrorId = 0;
	return (m_HS_Int_Move->execIntMove(intdata,iErrorId));
}

int HS_Int_Factory::MixSmoothMove(IntData &intdataPre,IntData &intdata)
{
    if(m_HS_Int_Move == NULL)
        return 0;
    return (m_HS_Int_Move->MixSmoothMove(intdataPre,intdata));
}

bool HS_Int_Factory::GetSmoothNextFlag()
{
    if(m_HS_Int_Move == NULL)
        return false;
    return (m_HS_Int_Move->GetSmoothNextFlag());
}

bool HS_Int_Factory::GetHalfSmoothFlag()
{
	if(m_HS_Int_Move == NULL)
		return false;
	return (m_HS_Int_Move->GetHalfSmoothFlag());
}

bool HS_Int_Factory::GetSmoothAheadFlag()
{
	if(m_HS_Int_Move == NULL)
		return false;
	return (m_HS_Int_Move->GetSmoothAheadFlag());
}

bool HS_Int_Factory::GetSLPlanFlag()
{
    if(m_HS_Int_Move == NULL)
        return false;
    return (m_HS_Int_Move->GetSLPlanFlag());
}

SyncPara HS_Int_Factory::GetSyncPara()
{
	return (m_HS_Int_Move->GetSyncPara());
}

int HS_Int_Factory::SetSyncParaPlan(SyncPara tSyncPara)
{
	if(m_HS_Int_Move == NULL)
		return false;
	return (m_HS_Int_Move->SetSyncParaPlan(tSyncPara));
}

int HS_Int_Factory::GetMasterCPos(double dMasterCPos[10][MaxAxisNum])
{
	return (m_HS_Int_Move->GetMasterCPos(dMasterCPos));
}

int HS_Int_Factory::SetMasterCPos(double dMasterCPos[10][MaxAxisNum])
{
	return (m_HS_Int_Move->SetMasterCPos(dMasterCPos));
}

double HS_Int_Factory::CalcStopTime()
{
	return m_HS_Int_Move->CalcStopTime();
}

int HS_Int_Factory::StopPlanByTime(double dTStop)
{
	return m_HS_Int_Move->StopPlanByTime(dTStop);
}

int HS_Int_Factory::GetRatioPara(double dRatio,double dTime[TIMECNT])
{
	return m_HS_Int_Move->GetRatioPara(dRatio,dTime);
}

int HS_Int_Factory::SetRatioPara(double dRatio,double dTime[TIMECNT],bool bRatioOKFlag)
{
	return m_HS_Int_Move->SetRatioPara(dRatio,dTime,bRatioOKFlag);
}

bool HS_Int_Factory::GetWeaveStopMainPos(double dMainJPos[MaxAxisNum])
{
	return m_HS_Int_Move->GetWeaveStopMainPos(dMainJPos);
}