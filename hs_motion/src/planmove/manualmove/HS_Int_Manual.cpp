
#include "HS_Int_Manual.h"

HS_Int_Manual::HS_Int_Manual()
{
    m_HS_BasicPara = HS_BasicPara::GetInstance();
	m_HS_GroupKin = new HS_GroupKin();
    m_dCycle = m_HS_BasicPara->m_dCycle;
    
    m_HS_FilterHandle = new HS_FilterHandle(m_dCycle);
    m_tFilterControl = &m_HS_BasicPara->m_tSysFilterPara.tHandControl;
    m_HS_SetPosCheck = new HS_SetPosCheck(0);

    memset(&m_tVelPlanPara,0,sizeof(m_tVelPlanPara));
    m_HS_VelPlan_Para = new HS_VelPlan_Para(m_dCycle);
    m_dStopDec = 0;;
	m_dTStop = 0;
	m_dStopKa = 0; 
	m_dStopKb = 0; 
	m_dStopKc = 0; 
	m_dStopDis = 0;
	m_dStopSPos = 0;
	m_bStopFlag = 0;
	m_dTCur = 0;
    m_bHandCoord = false;
    m_bPlanFlag = false;
    m_bWristQYFlag = false;
	m_bSyncMoveFlag = false;
    m_dJogVfac = 1.0;					//由0.3倍修改为1.0，外部控制倍率
}

HS_Int_Manual::~HS_Int_Manual()
{
    delete m_HS_SetPosCheck;
    m_HS_SetPosCheck = NULL;
    delete m_HS_VelPlan_Para;
    m_HS_VelPlan_Para = NULL;
    delete m_HS_FilterHandle;
    m_HS_FilterHandle = NULL;
	delete m_HS_GroupKin;
	m_HS_GroupKin = NULL;
}
/************************************************
函数功能：点动规划接口，进行点动的运动规划
参    数：tHS_GroupJPos--当前实际位置【关节】
         tManualPara---点动运动规划参数
返 回 值：错误码
*************************************************/
int HS_Int_Manual::Plan(HS_GroupJPos &tHS_GroupJPos, ManualPara tManualPara)
{
    int iErrorId = 0;
    double dAcc = 0;
    double dDec = 0;

    //参数获取以及检测
	m_tManualPara = tManualPara;
    m_iAxisNum = tManualPara.iAxisNum;
    m_bWristQYFlag = tManualPara.bWristQYOpen;

	m_bSyncMoveFlag = m_HS_GroupKin->SetGroupNum(tManualPara.iGroupNum,tManualPara.tHS_GroupRel);
	m_HS_Kinematics = m_HS_GroupKin->GetKinematicsByNum(tManualPara.iGroupNum);
    m_HS_Kinematics->InitPara();

	m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[tManualPara.iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[tManualPara.iGroupNum].tAxisVelocityPara.dAccelerate;
	m_tLimitPara = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[tManualPara.iGroupNum].tLimitPara;

	LOG_ALGO("---------------Start Manual Move!Group:%d---------------",tManualPara.iGroupNum);

    double dVelRatio = m_dJogVfac*tManualPara.dHandVelRatio;
    double dKVel = dVelRatio;
	double *dCurJPos = tHS_GroupJPos.dJPos[tManualPara.iGroupNum]; 

	if(dKVel < Eps)
	{
		LOG_ALGO("Error dKVel:%.6lf",dKVel);
		return E_H_PARKVEL;
	}

    memcpy(m_dRJPos,dCurJPos,sizeof(double)*MaxAxisNum);
	m_bCoorperFlag = false;

    if(tManualPara.hs_coordinate.iCoordinate == JOINT_COORD_SYSTEM)
    {
        //关节
        m_tVelPlanPara.dEVel = m_dJVelPara[m_iAxisNum]*dVelRatio;
        dAcc = m_dJAccPara[m_iAxisNum]*dVelRatio;        
		if(dAcc < Eps)
		{
			LOG_ALGO("Error Acc Para:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
				m_dJAccPara[0],m_dJAccPara[1],m_dJAccPara[2],m_dJAccPara[3],m_dJAccPara[4],m_dJAccPara[5],m_dJAccPara[6],m_dJAccPara[7],m_dJAccPara[8]);
			return E_H_PARAACC;
		}
		m_tVelPlanPara.dTAcc = fabs(3*m_tVelPlanPara.dEVel/2/dAcc);
        m_bHandCoord = false;

        memcpy(m_dInitHandJPos,dCurJPos,sizeof(double)*MaxAxisNum);

		if(m_iAxisNum >= 6&&m_iAxisNum <= 9&&tManualPara.bCoorperMove)
		{
			m_bCoorperFlag = true;
			LOG_ALGO("Open Coorper Move!");
			m_HS_Kinematics->HS_JPosToTCMPos(m_dRJPos,tManualPara.hs_coordinate.iToolNum,m_dBaseTCMPos);
			m_iToolNum = tManualPara.hs_coordinate.iToolNum;
		}
    }
    else
    {
		if(m_HS_Kinematics->GetRobotType() == HSROB_COORPER)
		{
			LOG_ALGO("Robot Coorper Type No Hand Move!");
			return E_H_COORPERHAND;
		}

        //空间
        m_bHandCoord = true;  
        //坐标系设置
        switch(tManualPara.hs_coordinate.iCoordinate)
        {
            case BASE_COORD_SYSTEM:
                m_HS_Kinematics->HS_SetCoordinate(tManualPara.hs_coordinate.iToolNum,-2);
                break;
            case USER_COORD_SYSTEM:
                m_HS_Kinematics->HS_SetCoordinate(tManualPara.hs_coordinate.iToolNum,tManualPara.hs_coordinate.iWorkNum);
                break;
            case WORLD_COORD_SYSTEM:
                m_HS_Kinematics->HS_SetCoordinate(tManualPara.hs_coordinate.iToolNum,-1);
                break;
            case TOOL_COORD_SYSTEM:
                m_HS_Kinematics->HS_SetTCoordinate(tManualPara.hs_coordinate.iToolNum,m_dRJPos);
                break;
            default:
                break;
        }	

        m_HS_Kinematics->HS_SetManualWristQY(tManualPara.bWristQYOpen,m_dRJPos);
        m_HS_Kinematics->HS_ResetQYHandle();

        memcpy(m_dInitHandJPos,dCurJPos,sizeof(double)*MaxAxisNum);

        m_HS_Kinematics->HS_JPosToCPos(dCurJPos,CP_ToolWork,m_dHandPos);	      

        if(tManualPara.hs_coordinate.bExtCoorper)
        {
            //地轨协同
        }

        memcpy(m_dInitHandCPos,m_dHandPos,sizeof(m_dInitHandCPos));	   

        //规划
        double dCVel[2] = {0};
        double dCAcc[2] = {0};
		dCVel[0] = m_tManualPara.dVtran;
		dCVel[1] = m_tManualPara.dVrot;

        m_HS_Kinematics->AutoHandleCVel(dCVel,dCAcc);
        if(m_iAxisNum < 3)
        {
            m_tVelPlanPara.dEVel = dCVel[0]*dVelRatio;
            dAcc = dCAcc[0]*dVelRatio;        
        }
        else if(m_iAxisNum < 6)
        {
            m_tVelPlanPara.dEVel = dCVel[1]*dVelRatio;
            dAcc = dCAcc[1]*dVelRatio;        
        }
        else
        {
            m_tVelPlanPara.dEVel = m_dJVelPara[m_iAxisNum]*dVelRatio;
            dAcc = m_dJAccPara[m_iAxisNum]*dVelRatio;        
        }
        m_tVelPlanPara.dTAcc = fabs(3*m_tVelPlanPara.dEVel/2/dAcc);
        
        iErrorId = StartPosWristQYCheck(dCurJPos);
        if(iErrorId != 0)
        {
            LOG_ALGO("ErrorNum = %d",iErrorId);
        }
    }

	if(m_bSyncMoveFlag)
	{
		m_HS_GroupKin->HandSyncInit(tHS_GroupJPos,tManualPara.iToolNum);
		LOG_ALGO("Group Sync Move!");
	}
    
    //范围检测【柔顺等级约束】
    double dJPos[6] = {0};
    double dTFreProtect = m_HS_BasicPara->HS_GetTFre(dJPos,tManualPara.iSmooth,tManualPara.iGroupNum);
    double dTFreProtectNew = m_HS_BasicPara->CalcTProtectByVel(dTFreProtect,dKVel,tManualPara.iGroupNum);
    LOG_ALGO("KVel:%.3lf; dTFreProtect:%.3lf;OrgiT:%.3lf",dKVel,dTFreProtect,dTFreProtectNew);
    if(m_tVelPlanPara.dTAcc < dTFreProtectNew)
    {
        m_tVelPlanPara.dTAcc = dTFreProtectNew;
        dAcc = fabs(3*m_tVelPlanPara.dEVel/m_tVelPlanPara.dTAcc/2);        
    } 

    m_tVelPlanPara.dTDec = m_tVelPlanPara.dTAcc;
    dDec = dAcc;
    m_dStopDec = dDec;
    m_dStopJerk = fabs(dAcc/m_tVelPlanPara.dTAcc)*1.0;
    //点动寸动处理
    if(tManualPara.dIncLen > Eps)
    {
        //寸动
        m_tVelPlanPara.dDis = tManualPara.dIncLen;
        
        //最大速度调整
        double dSAcc = m_tVelPlanPara.dEVel*m_tVelPlanPara.dTAcc/2;					
        double dSDec = m_tVelPlanPara.dEVel*m_tVelPlanPara.dTDec/2;				

        if(fabs(dSAcc + dSDec) > fabs(m_tVelPlanPara.dDis))
        {
            m_tVelPlanPara.dEVel = 2*m_tVelPlanPara.dDis/(m_tVelPlanPara.dTAcc + m_tVelPlanPara.dTDec);						
        }
        if(fabs(m_tVelPlanPara.dEVel) > Eps)
            m_tVelPlanPara.dTCon = fabs(m_tVelPlanPara.dDis/m_tVelPlanPara.dEVel) - m_tVelPlanPara.dTAcc/2 - m_tVelPlanPara.dTDec/2;
        m_tVelPlanPara.dTAll = m_tVelPlanPara.dTAcc + m_tVelPlanPara.dTDec + m_tVelPlanPara.dTCon;

        //反向运动
        if(!tManualPara.bDir)
        {
            m_tVelPlanPara.dEVel = -m_tVelPlanPara.dEVel;
            m_tVelPlanPara.dDis = -m_tVelPlanPara.dDis;
        }
    }
    else
    {
        //点动
        m_tVelPlanPara.dTCon = 100000;          //给足匀速段时间，则无停止
        m_tVelPlanPara.dTAll = m_tVelPlanPara.dTAcc + m_tVelPlanPara.dTDec + m_tVelPlanPara.dTCon;
        //反向运动
        if(!tManualPara.bDir)
        {
            m_tVelPlanPara.dEVel = -m_tVelPlanPara.dEVel;
        }
    }
    LOG_ALGO("RealJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);
    LOG_ALGO("Start Inch:%.3lf Axis %d;WristQYFlag = %d",m_tVelPlanPara.dDis,m_iAxisNum,(int)tManualPara.bWristQYOpen);
    LOG_ALGO("EVel:%.3lf,TAcc:%.3lf,TDec:%.3lf;Smooth = %d,TSmooth = %.3lf",
		m_tVelPlanPara.dEVel,m_tVelPlanPara.dTAcc,m_tVelPlanPara.dTDec,tManualPara.iSmooth,dTFreProtect);
    LOG_ALGO("Acc:%.3lf,Dec:%.3lf;SetVel:%.3lf,%.3lf",dAcc,dDec,m_tManualPara.dVtran,m_tManualPara.dVrot);
    LOG_ALGO("coord:%d Ratio:%.3lf",tManualPara.hs_coordinate.iCoordinate,tManualPara.dHandVelRatio);	

    if(m_tFilterControl->bFilterOpenFlag)
    {
        m_HS_VelPlan_Para->Plan(m_tVelPlanPara,m_tFilterControl->tFilterPara);
    }
    else
        m_HS_VelPlan_Para->Plan(m_tVelPlanPara);

    m_bStopFlag = false;
    m_dMovePos = 0;
    m_dKCVel = 1.0;
    m_bMoveErrorFlag = false;
    m_bAheadJErrorFlag = false;
    m_dAheadJPosDis = 0;
    m_HS_SetPosCheck->ResetCheck();
    if(iErrorId == 0)
      m_bPlanFlag = true;

    return iErrorId;
}

HS_MStatus HS_Int_Manual::Move(int &iErrorId,HS_GroupJPos &tHS_GroupJPos) 
{
    if(!m_bPlanFlag)
        return M_UnInit;

	double *dCurJPos = tHS_GroupJPos.dJPos[m_tManualPara.iGroupNum]; 

    iErrorId = 0;
    double dMovePos = 0;
    HS_MStatus Status = M_Busy;
    
    if(m_bStopFlag)
    {
        if(m_dTCur < m_dTStop)
        {
            dMovePos = m_dTCur*m_dTCur*m_dTCur*(m_dStopKa*m_dTCur + m_dStopKb) + 
                m_dTCur*(m_dStopKc*m_dTCur + m_dMoveVel);         
        }
        else
        {            
            dMovePos = m_dStopDis;       
            Status = M_Done;
        }

        dMovePos =  m_dStopSPos + dMovePos;
        m_dTCur += m_dCycle;
    }
    else
    {
        Status = m_HS_VelPlan_Para->Move(dMovePos);
        m_HS_VelPlan_Para->GetFilterVel(m_dMoveVel);
        m_HS_VelPlan_Para->GetFilterAcc(m_dMoveAcc);
    }

    if(m_bHandCoord)
    {   
        //增量添加，HandPos会约束改变，指令端的速度未变化
        m_dHandPos[m_iAxisNum] = m_dHandPos[m_iAxisNum] + (dMovePos - m_dMovePos);

        iErrorId = m_HS_Kinematics->HS_CPosToJPos_Hand(m_dHandPos,m_dInitHandCPos,m_dRJPos,m_dRJPos,m_dKCVel,m_iAxisNum);

        //奇异速度自适应保护检测
        if(m_dKCVel > AutoQYVelKLimit&&!m_bMoveErrorFlag)
        {
            //识别当前位置，给与不同的报警码信息
            iErrorId = m_HS_Kinematics->HS_QYErrorCheck(m_dRJPos);
            if(iErrorId == 0)
                iErrorId = ERROR_Hand_OverVel;
            m_HS_SetPosCheck->QuickStop();
            m_bMoveErrorFlag = true;
            LOG_ALGO("Error AutoQYVel,KCVEL = %.3lf,Limit = %.3lf",m_dKCVel,AutoQYVelKLimit);
        }
    }
    else
    {
        m_dRJPos[m_iAxisNum] = m_dInitHandJPos[m_iAxisNum] + dMovePos;
		if(m_bCoorperFlag)
		{
			//变位机点位转换
			m_HS_Kinematics->HS_TCMPosToJPos(m_dBaseTCMPos,m_iToolNum,m_dRJPos);
		}
    }

    m_dMovePos = dMovePos;

    //下发点位检测
    HS_MStatus eStausCheck = M_Busy;
    if(!m_bMoveErrorFlag)
    {
        iErrorId = m_HS_SetPosCheck->SetJPos(m_dRJPos,eStausCheck,m_bHandCoord);
        if(iErrorId != 0)
        {
            LOG_ALGO("SetJPos Check Error!");
            m_bMoveErrorFlag = true;
            LOG_ALGO("ErrorNum = %d",iErrorId);
        }
        if(m_bMoveErrorFlag)
        {
            if(eStausCheck == M_Error)
            {
                Status = M_Error;
            }
        }
    }
    else
    {
        m_HS_SetPosCheck->SetJPos(m_dRJPos,eStausCheck,m_bHandCoord);
    }

    //输出位置
    memcpy(dCurJPos,m_dRJPos,sizeof(double)*MaxAxisNum);

	if(m_bSyncMoveFlag)
	{
		m_HS_GroupKin->HandSyncMove(tHS_GroupJPos);
	}

    //LOG_ALGO("InterJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
			m_dRJPos[0],m_dRJPos[1],m_dRJPos[2],m_dRJPos[3],m_dRJPos[4],m_dRJPos[5],\
			m_dRJPos[6],m_dRJPos[7],m_dRJPos[8]);

    //前瞻检测处理
    double dAheadJPos[MaxAxisNum] = {0};
    if(!m_bMoveErrorFlag)
    {
        iErrorId = GetAheadJPos(dAheadJPos);
        if(iErrorId != 0)
        {
            //识别当前位置，给与不同的报警码信息
            iErrorId = m_HS_Kinematics->HS_QYErrorCheck(m_dRJPos);
            if(iErrorId == 0)
                iErrorId = ERROR_Hand_OverVel;
            LOG_ALGO("ErrorNum = %d",iErrorId);
            m_HS_SetPosCheck->QuickStop();
            m_bMoveErrorFlag = true;
            LOG_ALGO("Error Ahead JPos");
        }
    }

    if(!m_bMoveErrorFlag)
    {
        int iError = AheadLimitCheck(m_dRJPos,dAheadJPos);
        if(iError != 0)
        {
            m_bMoveErrorFlag = true;
            iErrorId = iError;
        }
    }

    //if(m_tFilterControl->bFilterOpenFlag)
    //{
    //    ManualFilterHandle(dhandJPos,Status);
    //}


    if(Status == M_Done)
    {
        m_bPlanFlag = false;
        LOG_ALGO("Stop Jog Done!");
    }

    return Status;
}

/************************************************
函数功能：整形滤波处理【输出端】
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Manual::ManualFilterHandle(double *dRJPos,HS_MStatus &eMStatus)
{
    int iErrorId = 0;
    bool bFilterDoneFlag = false;

    bFilterDoneFlag = m_HS_FilterHandle->FilterHandle(dRJPos,dRJPos);
   
    if(eMStatus == M_Done)
    {
        if(!bFilterDoneFlag)
            eMStatus = M_Busy;
    }
    return iErrorId;
}

/************************************************
函数功能：执行减速停止规划处理
参    数：
         dStopDis---限定停止距离【关节限位严格到点】
         bVelPlanStop---基于滤波后的数据快速停止【前置滤波使用】
返 回 值：错误码
*************************************************/
int HS_Int_Manual::StopPlan(double dStopDis,bool bVelPlanStop)
{
    if(!m_bPlanFlag)
        return 0;

    int iErrorId = 0;
    if(m_dKCVel > 1.0+Eps)
    {
        double dMoveVel = m_dMoveVel/m_dKCVel;
        m_dMoveVel = dMoveVel;
        double dDec = sqrt(fabs(3*dMoveVel*m_dStopJerk/2));
        m_dTStop = fabs(3*dMoveVel/dDec/2);
        m_dStopKa = 2*dMoveVel/(4*m_dTStop*m_dTStop*m_dTStop);
        m_dStopKb = -dMoveVel/(m_dTStop*m_dTStop);
        m_dStopKc = 0;		
        m_dStopDis = dMoveVel*m_dTStop/2;	
        LOG_ALGO("Stop Jog:T = %.3lf,KCVel = %.3lf",m_dTStop,m_dKCVel);
        bVelPlanStop = false;
    }
    else
    {
        if(fabs(dStopDis) > Eps)
        {
            if(fabs(m_dMoveVel) > Eps)
            {
                m_dTStop = fabs(2*dStopDis/m_dMoveVel);
                m_dStopKa = 2*m_dMoveVel/(4*m_dTStop*m_dTStop*m_dTStop);
                m_dStopKb = (-m_dMoveVel)/(m_dTStop*m_dTStop);
                m_dStopKc = 0;
            }
            m_dStopDis = dStopDis;	
            bVelPlanStop = false;
        }
        else
        {
            double dDec = sqrt(fabs(3*m_dMoveVel*m_dStopJerk/2));
            m_dTStop = fabs(3*m_dMoveVel/dDec/2);
            m_dStopKa = (m_dMoveAcc*m_dTStop + 2*m_dMoveVel)/(4*m_dTStop*m_dTStop*m_dTStop);
            m_dStopKb = (-3*m_dMoveVel - 2*m_dMoveAcc*m_dTStop)/(3*m_dTStop*m_dTStop);
            m_dStopKc = m_dMoveAcc/2;		
            m_dStopDis = m_dMoveAcc*m_dTStop*m_dTStop/12 + m_dMoveVel*m_dTStop/2;	

            if(bVelPlanStop)
            {
                m_HS_VelPlan_Para->StopPlan(m_dTStop);
            }
        }

        LOG_ALGO("Stop Jog:T = %.3lf",m_dTStop);
    }

    m_dStopSPos = m_dMovePos;
    if(!bVelPlanStop)
        m_bStopFlag = true;
    m_dTCur = m_dCycle;
    return iErrorId;
}

/************************************************
函数功能：获取前瞻停止的关节位置
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Manual::GetAheadJPos(double dAheadJPos[MaxAxisNum])
{
    int iErrorId = 0;
    memcpy(dAheadJPos,m_dRJPos,sizeof(double)*MaxAxisNum);

    if(m_bHandCoord)
    {  
        //前瞻空间位置
        double dAheadCPos[MaxAxisNum] = {0};
        double dMoveVel = m_dMoveVel/m_dKCVel;
        double dDec = sqrt(fabs(3*dMoveVel*m_dStopJerk/2));
        double dTStop = fabs(3*dMoveVel/dDec/2);	
        double dStopDis = dMoveVel*dTStop/2;	

        memcpy(dAheadCPos,m_dHandPos,sizeof(double)*MaxAxisNum);

        dAheadCPos[m_iAxisNum] += dStopDis;

        iErrorId = m_HS_Kinematics->HS_CPosToJPos_HandAhead(dAheadCPos,m_dInitHandCPos,m_dRJPos,dAheadJPos);
        if(iErrorId != 0)
        {
            if(m_HS_Kinematics->m_bWristQYHandleFlag&&m_HS_Kinematics->GetRobotType() == HSROB_PUMA&&fabs(m_dRJPos[5]) < 30)
            {
                iErrorId = 0;
            }
        }
    }
    else
    {
        double dDec = sqrt(fabs(3*m_dMoveVel*m_dStopJerk/2));
        double dTStop = fabs(3*m_dMoveVel/dDec/2);	
        double dStopDis = m_dMoveVel*dTStop/2;	

        dAheadJPos[m_iAxisNum] = dAheadJPos[m_iAxisNum] + dStopDis;
    }
    return iErrorId;
}

/************************************************
函数功能：限位的前瞻检测停止
参    数：
        dCurJPos-----当前关节位置
        dAheadJPos---前瞻关节位置
返 回 值：错误码
*************************************************/
int HS_Int_Manual::AheadLimitCheck(double dCurJPos[6],double dAheadJPos[MaxAxisNum])
{
    int iErrorId = 0;
    //空间运动添加一定的限位保护
    double dAddProect = 0;
    if(m_bHandCoord)
        dAddProect = 0.5;

	double dCJPos[MaxAxisNum] = {0};
	double dAJPos[MaxAxisNum] = {0};

	memcpy(dCJPos,dCurJPos,sizeof(double)*MaxAxisNum);
	memcpy(dAJPos,dAheadJPos,sizeof(double)*MaxAxisNum);

    for(int i = 0;i < MaxAxisNum;i++)
    {
        if(m_tLimitPara->bOpen[i])
        {
            if(dAJPos[i] > m_tLimitPara->dPmax[i] - dAddProect&&dAJPos[i] > dCJPos[i]+Eps)
            {
                LOG_ALGO("Error PMax Limit:Axis:%d;Cur = %.3lf,Ahead = %.3lf;PMax = %.3lf",i+1,dCJPos[i],dAJPos[i],m_tLimitPara->dPmax[i]);
                iErrorId = ERROR_PMAX;
                LOG_ALGO("ErrorNum = %d",iErrorId);
                if(m_bHandCoord)
                    StopPlan();
                else
                    StopPlan(m_tLimitPara->dPmax[i] - m_dRJPos[i]);
                break;
            }
            else if(dAJPos[i] < m_tLimitPara->dPmin[i] + dAddProect&&dAJPos[i] < dCJPos[i]-Eps)
            {
                LOG_ALGO("Error PMin Limit:Axis:%d;Cur = %.3lf,Ahead = %.3lf;;PMin = %.3lf",i+1,dCJPos[i],dAJPos[i],m_tLimitPara->dPmin[i]);
                iErrorId = ERROR_PMIN;
                LOG_ALGO("ErrorNum = %d",iErrorId);
                if(m_bHandCoord)
                    StopPlan();
                else
                    StopPlan(m_tLimitPara->dPmin[i] - m_dRJPos[i]);
                break;
            }
        }
    }

    //空间运动
    if(iErrorId == 0&&m_bHandCoord&&!m_bAheadJErrorFlag)
    {
        //增加判断处理，存在前瞻计算结果异常或者失效的情况，比如奇异处理，或者求解错误等
        bool bAheadCheckFlag = true;
        double dJPosDis = 0;
        for(int i = 0;i < 6;i++)
        {
            dJPosDis += fabs(dAheadJPos[i] - dCurJPos[i]);
        }
        if(fabs(dJPosDis - m_dAheadJPosDis) > 10)
        {
            bAheadCheckFlag = false;
            m_bAheadJErrorFlag = true;
            LOG_ALGO("AheadJError:Cur = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,",
                dCurJPos[0],dCurJPos[1],dCurJPos[2],dCurJPos[3],dCurJPos[4],dCurJPos[5]);
            LOG_ALGO("AheadJError:Ahead = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,",
                dAheadJPos[0],dAheadJPos[1],dAheadJPos[2],dAheadJPos[3],dAheadJPos[4],dAheadJPos[5]);
        }
        else
            m_dAheadJPosDis = dJPosDis;

        if(bAheadCheckFlag)
        {
            iErrorId = m_HS_Kinematics->HS_QYDynCheck(dCurJPos,dAheadJPos);
            if(iErrorId != 0)
			{
				//过腕部奇异功能不进行该检测处理
				if(iErrorId == ERROR_QY_WRIST&&m_HS_Kinematics->m_bWristQYHandleFlag&&m_HS_Kinematics->GetRobotType() == HSROB_PUMA)
					iErrorId = 0;
				else
				{
					LOG_ALGO("QyError = %d",iErrorId);
					StopPlan(0,false);
				}
			}
        }
    }
    return iErrorId;
}

/************************************************
函数功能：空间点动启动点关节位置的腕部奇异保护
参    数：
        dCurJPos-----当前关节位置
返 回 值：错误码
*************************************************/
int HS_Int_Manual::StartPosWristQYCheck(double dCurJPos[6])
{
    int iErrorId = 0;

    if(m_HS_Kinematics->GetRobotType() == HSROB_PUMA&&!m_bWristQYFlag)
    {
        //if(fabs(dCurJPos[4]) < WRISTSTARTPROTECT)
        //{
        //    iErrorId = ERROR_QY_WRIST;
        //}
    }
    else if(m_HS_Kinematics->GetRobotType() == HSROB_SCARA)
    {
        if(fabs(dCurJPos[1]) < WRISTSTARTPROTECT)
        {
            iErrorId = ERROR_QY_BORDER;
        }
    }
    return iErrorId;
}

