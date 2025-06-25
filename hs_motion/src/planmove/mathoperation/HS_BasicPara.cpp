
#include "HS_BasicPara.h"

HS_BasicPara* HS_BasicPara::GetInstance()
{
    /**
     * 局部静态特性的方式实现单实例。
     * 静态局部变量只在当前函数内有效，其他函数无法访问。
     * 静态局部变量只在第一次被调用的时候初始化，也存储在静态存储区，生命周期从第一次被初始化起至程序结束止。
     */
    static HS_BasicPara single;
    return &single;
}

void HS_BasicPara::SetPara(MotionPara *para)
{
    mMotionPara = para;

    memset(&m_tSysFilterPara,0,sizeof(m_tSysFilterPara));

    m_tSysFilterPara.tHandControl.bFilterOpenFlag = false;
    m_tSysFilterPara.tHandControl.tFilterPara.eFilterType = TYPE_FIR;
    m_tSysFilterPara.tHandControl.tFilterPara.dFre = 10;
    m_tSysFilterPara.tHandControl.tFilterPara.iAxisNum = 9;
    m_tSysFilterPara.tHandControl.tFilterPara.iGrade = 9;

    m_tSysFilterPara.tAutoControl.bFilterOpenFlag = false;
    m_tSysFilterPara.tAutoControl.tFilterPara.eFilterType = TYPE_FIR;
    m_tSysFilterPara.tAutoControl.tFilterPara.dFre = 10;
    m_tSysFilterPara.tAutoControl.tFilterPara.iAxisNum = 9;
    m_tSysFilterPara.tAutoControl.tFilterPara.iGrade = 9;
    m_tSysFilterPara.iAutoFilterType = 1;
    
}

void HS_BasicPara::SetCycle(double dCycle,int iInterMultCnt)
{
    m_dCycle = dCycle;
    m_iInterMultCnt = iInterMultCnt;
    if(m_dCycle < 0.001)
        m_dCycle = 0.001;
    if(m_iInterMultCnt < 1)
        m_iInterMultCnt = 1;
}

HS_BasicPara::HS_BasicPara()
{
}

HS_BasicPara::~HS_BasicPara()
{
}

/************************************************
函数功能：动力学约束的最小约束时间获取，用来获取振动时间约束量
参    数：
返 回 值：无
*************************************************/
double HS_BasicPara::HS_GetTFre(double dJPos[6],int iSmooth,int iGroupNum)
{
    double m_dTFreProtect = 0;

	m_dJVelPara = mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate;

	double dTFreMin = mMotionPara->m_tGroupStaticPara[iGroupNum].tGroupVelocityPara.dTFreMin;
	double dTFreMax = mMotionPara->m_tGroupStaticPara[iGroupNum].tGroupVelocityPara.dTFreMax;

    if(dTFreMin < 0.01) 
        dTFreMin = 0.01;
     if(dTFreMax < 0.2) 
        dTFreMax = 0.2;   

    if(iSmooth < 1)
        m_dTFreProtect = dTFreMin;
    else if(iSmooth > 9)
        m_dTFreProtect = dTFreMax;
    else
        m_dTFreProtect = dTFreMin + (iSmooth - 1)*(dTFreMax - dTFreMin)/8;
    return m_dTFreProtect;
}

/************************************************
函数功能：计算不同速度条件下的时间保护优化值，以满足节拍需求等
参    数：dTFreProtect---原时间保护值
		 dKJVel---------当前的速度比例
返 回 值：修正后的时间保护值
*************************************************/
double HS_BasicPara::CalcTProtectByVel(double dTFreProtect,double dKJVel,int iGroupNum)
{

	m_dJVelPara = mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate;

    //计算基础的加速时间最大值【6轴】
    double dTAccBase = 0;
    for(int i = 0;i < 6;i++)
    {
        double dTTemp = 0;
        if(m_dJAccPara[i] > Eps)
            dTTemp = fabs(m_dJVelPara[i]/m_dJAccPara[i]);
        dTAccBase = Max(dTAccBase,dTTemp);
    }

    double dTProtect = dTFreProtect;
    bool bHeavyLoad = false;

    if(bHeavyLoad)
    {
        //重载机器人的处理不同，直接约束时间，不做约束处理
    }
    else
    {
        //速度限制下的保护，小于该速度值，则进行时间的约束保护
        double dLimitVel = 0.4;
        //进行不同速度等级的时间约束处理
        if(dKJVel < dLimitVel)
        {
            double dTAccLimit = sqrt(dKJVel)*dTAccBase;

            double dKT = dKJVel/0.4;
            //最大缩放时间保护
            const double dKTLIMIT = 0.5;
            if(dKT < dKTLIMIT)
                dKT = dKTLIMIT;
            dTProtect *= dKT;
            if(dTProtect < 0.008)
                dTProtect = 0.008;

            //基于速度的约束小于基于捷度的约束时，按照捷度约束处理，最大为保护约束
            if(dTProtect < dTAccLimit)
            {
                if(dTAccLimit < dTFreProtect)
                    dTProtect = dTAccLimit;
                else
                    dTProtect = dTFreProtect;
            }
        } 
    }
    return dTProtect;
}

