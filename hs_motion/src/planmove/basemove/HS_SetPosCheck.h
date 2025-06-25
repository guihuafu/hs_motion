

#ifndef _HS_SETPOSCHECK_H
#define _HS_SETPOSCHECK_H

#include "HS_BasicPara.h"
#include "HS_VelPlan_Para.h"
#include "HS_Kinematics.h"

//HS_SetPosCheck 类
class HS_SetPosCheck
{
public:
    HS_SetPosCheck(int iGroupNum);
    ~HS_SetPosCheck();
    int SetJPos(double dSetJPos[MaxAxisNum],HS_MStatus &eStaus,bool bHandCoordFlag = false);
    int QuickStop();
    void ResetCheck();
private:
    int ErrorStop(double *dRealJPos);
    HS_MStatus StopUpdatePos(double *dRealJPos);

    double m_dRealJPos[MaxAxisNum];
    double m_dRealJVel[MaxAxisNum];
    double m_dRealJAcc[MaxAxisNum];
    double m_dStopSPos[MaxAxisNum];
    HS_BasicPara *m_HS_BasicPara;
    HS_Kinematics *m_HS_Kinematics;
    double *m_dJVelPara;
    double *m_dJAccPara;
    double m_dCycle;
    int m_iParaState;                   //点位参数状态
    bool m_bErrorStopFlag;
    bool m_bQuickStopFlag;
    HS_RobotType *m_eRobotType;

	HS_VelPlan_Para **m_HS_VelPlan_Para;				//速度规划指针
};

#endif