#include "basemanualmove.h"
#include <iostream>
#include "HS_BasicPara.h"
#include "HS_Int_Manual.h"

namespace hsc3
{
namespace algo
{
    BaseManualMove::BaseManualMove(MotionPara *para,double dCycle) : ManualMove(para,dCycle)
    {
        this->mMotionPara = para;
        HS_BasicPara  * pHS_BasicPara = HS_BasicPara::GetInstance();
        pHS_BasicPara->SetPara(para);
        pHS_BasicPara->SetCycle(dCycle,1);

        m_HS_Int_Manual = new HS_Int_Manual();
    }

    BaseManualMove::~BaseManualMove()
    {
        delete m_HS_Int_Manual;
    }

    int BaseManualMove::execPrintKeyInfo()
    {
        return 0;
    }

    int BaseManualMove::Plan(HS_GroupJPos &tHS_GroupJPos, ManualPara tManualPara)
    {
        return m_HS_Int_Manual->Plan(tHS_GroupJPos,tManualPara);
    }

	HS_MStatus BaseManualMove::Move(int &iErrorId,HS_GroupJPos &tHS_GroupJPos) 
    {
        return m_HS_Int_Manual->Move(iErrorId,tHS_GroupJPos);
    }

	int BaseManualMove::StopPlan() 
    {
        return m_HS_Int_Manual->StopPlan(0,false);
    }

}
}