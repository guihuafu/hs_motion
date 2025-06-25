/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_SetPosCheck.cpp
* 摘    要：点位下发检测，对下发的点位进行一定的参数检测
           
* 当前版本：1.0
* 作    者：cyh
* 完成日期：2024-2-18
*			
*/
#include "HS_SetPosCheck.h"

/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_SetPosCheck::HS_SetPosCheck(int iGroupNum)
{
    m_HS_BasicPara = HS_BasicPara::GetInstance();
    m_HS_Kinematics = new HS_Kinematics(iGroupNum);  
    m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
    m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate; 
    m_dCycle = m_HS_BasicPara->m_dCycle;
    //m_eRobotType = &m_HS_BasicPara->mMotionPara->m_tGroupModelPara.eRobtype;

    m_HS_VelPlan_Para = new HS_VelPlan_Para *[MaxAxisNum];
    for(int i = 0;i < MaxAxisNum;i++)	
        m_HS_VelPlan_Para[i] = new HS_VelPlan_Para(m_dCycle);
    ResetCheck();
}
HS_SetPosCheck::~HS_SetPosCheck(void)
{
    for(int i = 0;i < MaxAxisNum;i++)
        delete m_HS_VelPlan_Para[i];
    delete[] m_HS_VelPlan_Para; 
    m_HS_VelPlan_Para = NULL;
}
/************************************************
函数功能：对输入的点位进行缓存以及检测
参    数：dSetJPos-----下发检测的点位数据
返 回 值：错误码
*************************************************/
void HS_SetPosCheck::ResetCheck()
{
    memset(m_dRealJPos,0,sizeof(double)*MaxAxisNum);
    memset(m_dRealJVel,0,sizeof(double)*MaxAxisNum);
    memset(m_dRealJAcc,0,sizeof(double)*MaxAxisNum);
    m_iParaState = 0;
    m_bErrorStopFlag = false;
    m_bQuickStopFlag = false;
}
/************************************************
函数功能：对输入的点位进行缓存以及检测
参    数：dSetJPos-----下发检测的点位数据
         eStaus--------运行状态【停止返回M_Error】
         bHandCoordFlag--空间点动，需要对速度加速度进行特定的检测，避免运动过快
返 回 值：错误码
*************************************************/
int HS_SetPosCheck::SetJPos(double dSetJPos[MaxAxisNum],HS_MStatus &eStaus,bool bHandCoordFlag)
{
    int iErrorId = 0;

    eStaus = M_Busy;
    if(m_bErrorStopFlag)	//如果处于错误停止的状态
    {
        HS_MStatus eStausStop = StopUpdatePos(dSetJPos);      
        if(eStausStop == M_Done)
            eStaus = M_Error;
        return 0;
    }

    double dRealJVel[MaxAxisNum] = {0};
    double dRealJAcc[MaxAxisNum] = {0};
    double dRealJerk[MaxAxisNum] = {0};

    for(int i = 0;i < MaxAxisNum;i++)
    {
        if(m_iParaState > 0)
            dRealJVel[i] = (dSetJPos[i] - m_dRealJPos[i])/m_dCycle;
        if(m_iParaState > 1)
            dRealJAcc[i] = (dRealJVel[i] - m_dRealJVel[i])/m_dCycle;	
        if(m_iParaState > 2)
            dRealJerk[i] = (dRealJAcc[i] - m_dRealJAcc[i])/m_dCycle;
    }

    if(m_bQuickStopFlag)
    {
        m_bQuickStopFlag = false;
        ErrorStop(dSetJPos);  
        return 0;
    }

    //
    //限制比例值，如果是非动力学，则倍数低，动力学则增加
    for(int i = 0;i < MaxAxisNum;i++)
    {
        //阈值放大，大多用来检测算法异常，对规划加速度过大的情况不进行判别，通过电流检测处理
        double dKAccLimt = 5.0;
        double dJerkLimit = m_dJAccPara[i]*dKAccLimt/0.004;

        //算法异常
        if((fabs(dRealJerk[i]) > dJerkLimit||fabs(dRealJAcc[i]) > m_dJAccPara[i]*dKAccLimt))
		//if(false)
        {
            LOG_ALGO("Error Jerk:%.0lf,Acc:%.3lf,%.3lf,LimitAcc:%.3lf,Axis:%d",\
                dRealJerk[i],dRealJAcc[i],m_dRealJAcc[i],m_dJAccPara[i],i+1);

            LOG_ALGO("PreJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                m_dRealJPos[0],m_dRealJPos[1],m_dRealJPos[2],m_dRealJPos[3],m_dRealJPos[4],m_dRealJPos[5],m_dRealJPos[6],m_dRealJPos[7],m_dRealJPos[8]);
            LOG_ALGO("CurJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dSetJPos[0],dSetJPos[1],dSetJPos[2],dSetJPos[3],dSetJPos[4],dSetJPos[5],dSetJPos[6],dSetJPos[7],dSetJPos[8]);
            LOG_ALGO("RealJVel:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dRealJVel[0],dRealJVel[1],dRealJVel[2],dRealJVel[3],dRealJVel[4],dRealJVel[5],dRealJVel[6],dRealJVel[7],dRealJVel[8]);
            LOG_ALGO("RealJAcc:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dRealJAcc[0],dRealJAcc[1],dRealJAcc[2],dRealJAcc[3],dRealJAcc[4],dRealJAcc[5],dRealJAcc[6],dRealJAcc[7],dRealJAcc[8]);

            iErrorId = m_HS_Kinematics->HS_QYErrorCheck(dSetJPos);
            if(iErrorId == 0)
                iErrorId = E_SETPOS_OVERACC;	

            //各轴单独减速停止，防止当前的规划无法正常停止
            ErrorStop(dSetJPos);  
            return iErrorId;
        }

        //超速报警
        const double dKVelLimit = 1.2;//2.0;//1.2;
        if(fabs(dRealJVel[i]) > m_dJVelPara[i]*dKVelLimit&&m_dJVelPara[i] > Eps)
        {
            LOG_ALGO("Axis:%d,Vel:%.0lf,LimitVel:%.0lf,BaseVel:%.0lf",i,dRealJVel[i],m_dJVelPara[i]*dKVelLimit,m_dJVelPara[i]);
            LOG_ALGO("CurJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dSetJPos[0],dSetJPos[1],dSetJPos[2],dSetJPos[3],dSetJPos[4],dSetJPos[5],dSetJPos[6],dSetJPos[7],dSetJPos[8]);
            LOG_ALGO("RealJVel:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dRealJVel[0],dRealJVel[1],dRealJVel[2],dRealJVel[3],dRealJVel[4],dRealJVel[5],dRealJVel[6],dRealJVel[7],dRealJVel[8]);
            LOG_ALGO("RealJAcc:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                dRealJAcc[0],dRealJAcc[1],dRealJAcc[2],dRealJAcc[3],dRealJAcc[4],dRealJAcc[5],dRealJAcc[6],dRealJAcc[7],dRealJAcc[8]);


			iErrorId = m_HS_Kinematics->HS_QYErrorCheck(dSetJPos);
            if(iErrorId == 0)
                //iErrorId = ERROR_LOW(i+1,ERROR_VMAX);
                iErrorId = E_SETPOS_OVERVEL;

            ErrorStop(dSetJPos);  
            return iErrorId;
        }	

        if(bHandCoordFlag)
        {
            const double dKVelLimit = 0.6;
            const double dKAccLimit = 2.0;
            if((fabs(dRealJVel[i]) > m_dJVelPara[i]*dKVelLimit&&m_dJVelPara[i] > Eps)||
                (fabs(dRealJAcc[i]) > m_dJAccPara[i]*dKAccLimit&&m_dJAccPara[i] > Eps))
            {
                LOG_ALGO("Axis:%d,Vel:%.0lf,KLimitVel:%.3lf;KAccLimit:%.3lf",i,dRealJVel[i],dKVelLimit,dKAccLimit);
                LOG_ALGO("CurJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                    dSetJPos[0],dSetJPos[1],dSetJPos[2],dSetJPos[3],dSetJPos[4],dSetJPos[5],dSetJPos[6],dSetJPos[7],dSetJPos[8]);
                LOG_ALGO("RealJVel:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                    dRealJVel[0],dRealJVel[1],dRealJVel[2],dRealJVel[3],dRealJVel[4],dRealJVel[5],dRealJVel[6],dRealJVel[7],dRealJVel[8]);
                LOG_ALGO("RealJAcc:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
                    dRealJAcc[0],dRealJAcc[1],dRealJAcc[2],dRealJAcc[3],dRealJAcc[4],dRealJAcc[5],dRealJAcc[6],dRealJAcc[7],dRealJAcc[8]);
                iErrorId = ERROR_Hand_OverVel;
                ErrorStop(dSetJPos);  
                return iErrorId;
            }
        }
    }

    //数据缓存
    if(m_iParaState < 3)
        m_iParaState++;
    memcpy(m_dRealJPos,dSetJPos,sizeof(double)*MaxAxisNum);
    memcpy(m_dRealJVel,dRealJVel,sizeof(double)*MaxAxisNum);
    memcpy(m_dRealJAcc,dRealJAcc,sizeof(double)*MaxAxisNum);
    return iErrorId;
}

/************************************************
函数功能：报错处理，减速停止
参   数：
返 回 值：无
*************************************************/
int HS_SetPosCheck::ErrorStop(double *dRealJPos)
{
    //以当前的速度，直接规划减速停止
    //执行规划输出、减速停止
    VelPlanPara tPlan[MaxAxisNum] = {0};
    double dMaxTime = 0;	//最大减速停止时间
    double dTSDec = 0;
    double dKStopAcc = 1.5;
    for(int j = 0;j < MaxAxisNum;j++)
    {
        if(m_dJAccPara[j] > Eps)
            dTSDec = fabs(3*m_dRealJVel[j]/2/(m_dJAccPara[j]*dKStopAcc));
        dMaxTime = Max(dMaxTime,dTSDec);
    }
    //限制停止最小时间
    if(dMaxTime < 0.02)
        dMaxTime = 0.02;
    for(int j = 0;j < MaxAxisNum;j++)
    {
        m_dStopSPos[j] = m_dRealJPos[j];
        tPlan[j].dSVel = m_dRealJVel[j];
        tPlan[j].dSAcc = 0;
        tPlan[j].eTypeVel = TYPEVP_Stop;
        tPlan[j].dEVel = 0;
        tPlan[j].dTAcc = dMaxTime;
        tPlan[j].dTAll = dMaxTime;
        m_HS_VelPlan_Para[j]->Plan(tPlan[j]);
    }

    StopUpdatePos(dRealJPos);
    m_bErrorStopFlag = true;
    return 0;
}
/************************************************
函数功能：算法错误异常停止处理
参   数：
返 回 值：无
*************************************************/
HS_MStatus HS_SetPosCheck::StopUpdatePos(double *dRealJPos)
{
    int iNum_Done = 0;
    for(int i = 0;i < MaxAxisNum;i++)
    {
        HS_MStatus Status = m_HS_VelPlan_Para[i]->Move(dRealJPos[i]);

        dRealJPos[i] = m_dStopSPos[i] + dRealJPos[i];

        if(Status == M_Done||Status == M_UnInit)
            iNum_Done++;

        if(Status == M_Error)
            return M_Error;
    }

    if(iNum_Done == MaxAxisNum)
        return M_Done;

    return M_Busy;
}
/************************************************
函数功能：算法错误异常停止处理
参   数：
返 回 值：无
*************************************************/
int HS_SetPosCheck::QuickStop()
{
    m_bQuickStopFlag = true;
    return 0;
}