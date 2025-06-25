/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_VelPlan_Para.cpp
* 摘    要：速度规划抛物线插补

* 当前版本：3.0
* 作    者：cyh
* 完成日期：
*		1、修改为考虑平滑的规划方式，外部添加平滑系数；
*		2、纯净版抛物线规划方式
*			
*/
#include "HS_VelPlan_Para.h"

/************************************************
函数功能：构造函数
参    数：伺服周期
返 回 值：无
*************************************************/
HS_VelPlan_Para::HS_VelPlan_Para(double dCycle)
{
	m_dCycle = dCycle;
    m_HS_FilterHandle.SetCycle(m_dCycle);
	Reset();
}
HS_VelPlan_Para::~HS_VelPlan_Para()
{

}

/************************************************
函数功能：对参数进行复位
参    数：
返 回 值：无
*************************************************/
void HS_VelPlan_Para::Reset(void)
{
	m_dPos = 0;	
	m_dTCur = 0;
	m_dKaAcc = 0;
	m_dKbAcc = 0;	
	m_dKaDec = 0;
	m_dKbDec = 0;
	m_dPosAcc = 0;
	m_dSPos = 0;
	m_dKaAccA = 0;
	m_dKbAccA = 0;
    m_dVel = 0;
    m_dAcc = 0;
    m_dFilterPos = 0;
    m_dFilterVel = 0;
    m_dFilterAcc = 0;
    m_bFilterOpenFlag = false;
}
/************************************************
函数功能：规划曲线
参    数：
返 回 值：错误ID
*************************************************/
int HS_VelPlan_Para::Plan(VelPlanPara tPara)
{
	Reset();	
	m_tPlan = tPara;

    if(tPara.eTypeVel == TYPEVP_Stop)
    {
        //减速停止模式
        m_dKaAcc = (m_tPlan.dSAcc*m_tPlan.dTAcc + 2*m_tPlan.dSVel)/(4*m_tPlan.dTAcc*m_tPlan.dTAcc*m_tPlan.dTAcc);
        m_dKbAcc = (-3*m_tPlan.dSVel - 2*m_tPlan.dSAcc*m_tPlan.dTAcc)/(3*m_tPlan.dTAcc*m_tPlan.dTAcc);
        m_dKcAcc = m_tPlan.dSAcc/2;
        m_dPosAcc = m_tPlan.dSAcc*m_tPlan.dTAcc*m_tPlan.dTAcc/12 + m_tPlan.dSVel*m_tPlan.dTAcc/2;
        m_dSVel = m_tPlan.dSVel;
        m_dSAcc = m_tPlan.dSAcc;
        m_dTCur = m_dCycle;
        return 0;
    }

	//1、加速度段
	if(m_tPlan.dTAcc > Eps)
	{		
        //根据类型不同，进行不同的规划
        switch(tPara.eTypeAcc)
        {
        case APType_Para:
        case APType_ParaSin:
            m_dKaAcc = - m_tPlan.dEVel/(2*m_tPlan.dTAcc*m_tPlan.dTAcc*m_tPlan.dTAcc);
            m_dKbAcc = m_tPlan.dEVel/(m_tPlan.dTAcc*m_tPlan.dTAcc);	
            break;
        case APType_Sin:
            m_dKaAcc = m_tPlan.dEVel*m_tPlan.dTAcc/PI/PI/4;
            m_dKbAcc = m_tPlan.dEVel/m_tPlan.dTAcc/2;	
            break;
        default:
            break;
        }

		m_dKcAcc = 0;
		m_dSAcc = 0;
		m_dSVel = 0;
		m_dPosAcc = m_tPlan.dEVel*m_tPlan.dTAcc/2;
	}		
	
	if(m_tPlan.bAddModeFlag)
	{
		if(m_tPlan.bBackFlag)
		{
			double dSVel = m_tPlan.dEVel*m_tPlan.dAddKVel;
			double dEVel = m_tPlan.dEVel;
			double dTAcc = m_tPlan.dTAcc;

			m_dKaAcc = - dSVel/(2*dTAcc*dTAcc*dTAcc);
			m_dKbAcc = dSVel/(dTAcc*dTAcc);	
			m_dPosAcc = dSVel*dTAcc/2;

			if(m_tPlan.dAddTAcc)
			{
				m_dKaAccA = -(dEVel - dSVel)/(2*m_tPlan.dAddTAcc*m_tPlan.dAddTAcc*m_tPlan.dAddTAcc);
				m_dKbAccA = (dEVel - dSVel)/(m_tPlan.dAddTAcc*m_tPlan.dAddTAcc);
			}
			m_dPosAccA = (dSVel + dEVel)*m_tPlan.dAddTAcc/2;
			if(m_tPlan.dTDec > Eps)
			{
				m_dKaDec = dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
				m_dKbDec = - dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);		
			}
		}
		else
		{
			//添加加速段
			double dSVel = m_tPlan.dEVel;
			double dEVel = m_tPlan.dEVel*m_tPlan.dAddKVel;
			if(m_tPlan.dAddTAcc)
			{
				m_dKaAccA = -(dEVel - dSVel)/(2*m_tPlan.dAddTAcc*m_tPlan.dAddTAcc*m_tPlan.dAddTAcc);
				m_dKbAccA = (dEVel - dSVel)/(m_tPlan.dAddTAcc*m_tPlan.dAddTAcc);
			}
			m_dPosAccA = (dSVel + dEVel)*m_tPlan.dAddTAcc/2;
			if(m_tPlan.dTDec > Eps)
			{
				m_dKaDec = dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
				m_dKbDec = - dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);		
			}
		}	
	}
	else
	{
		//2、减速段，时间为dTDec,减速段的启动加速度为0，所以不包含Kc项	
		if(m_tPlan.dTDec > Eps)
		{	
            //根据类型不同，进行不同的规划
            switch(tPara.eTypeAcc)
            {
            case APType_Para:
                m_dKaDec = m_tPlan.dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
                m_dKbDec = - m_tPlan.dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);
                break;
            case APType_Sin:
            case APType_ParaSin:
                m_dKaDec = -m_tPlan.dEVel*m_tPlan.dTDec/PI/PI/4;
                m_dKbDec = -m_tPlan.dEVel/m_tPlan.dTDec/2;	
                break;
            default:
                break;
            }
		}
		m_tPlan.dAddKVel = 1.0;
		m_tPlan.dAddTAcc = 0;
		m_dPosAccA = 0;
	}

	m_dTCur = m_dCycle + m_tPlan.dTSmoothOff;	
    m_bFilterOpenFlag = false;
	m_bRePlanFlag = false;
	m_dTLast = m_tPlan.dTAll - m_tPlan.dTAcc;
	m_dTAcc = m_tPlan.dTAcc;
	return 0;
}
/************************************************
函数功能：速度规划，叠加滤波器处理
参    数：
        tPara---------速度规划
        tFilterPara---整形滤波器
返 回 值：错误ID
*************************************************/
int HS_VelPlan_Para::Plan(VelPlanPara tPara,FilterPara tFilterPara)
{
    Plan(tPara);
    tFilterPara.iAxisNum = 1;
    m_HS_FilterHandle.Filer_SetPara(tFilterPara);
    double dInitPos = m_dSPos;
    m_HS_FilterHandle.FilterHandle(&dInitPos,&dInitPos);
    m_bFilterOpenFlag = true;
    return 0;
}
/************************************************
函数功能：重新规划曲线，在现有的运动基础上更改运行条件
参    数：
返 回 值：错误ID
*************************************************/
int HS_VelPlan_Para::RePlan(VelPlanPara tPara)
{
    m_tPlan.dTAll = tPara.dTAll;
    m_tPlan.dTCon = tPara.dTCon;
    m_tPlan.dTDec = tPara.dTDec;
   
    //减速段，时间为dTDec,减速段的启动加速度为0，所以不包含Kc项	
    if(m_tPlan.dTDec > Eps)
    {	
        //根据类型不同，进行不同的规划
        switch(tPara.eTypeAcc)
        {
        case APType_Para:
            m_dKaDec = m_tPlan.dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
            m_dKbDec = - m_tPlan.dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);
            break;
        case APType_Sin:
        case APType_ParaSin:
            m_dKaDec = -m_tPlan.dEVel*m_tPlan.dTDec/PI/PI/4;
            m_dKbDec = -m_tPlan.dEVel/m_tPlan.dTDec/2;	
            break;
        default:
            break;
        }
    }

    return 0;
}
/************************************************
函数功能：计算速度，生成曲线
参    数：
返 回 值：轨迹生成状态
*************************************************/
HS_MStatus HS_VelPlan_Para::Move(double &dPos)
{
	double dTAcc = m_tPlan.dTAcc;
    HS_MStatus eStatus = M_Busy;
	
    if(m_tPlan.eTypeVel == TYPEVP_Stop)
    {
        if(m_dTCur - m_tPlan.dTAll > Eps)
        {
            m_dPos = m_dPosAcc;
            dPos = m_dPos + m_dSPos;
            if(m_bFilterOpenFlag)
            {
                bool bFilterDoneFlag = false;
                bFilterDoneFlag = m_HS_FilterHandle.FilterHandle(&dPos,&dPos);              
                if(!bFilterDoneFlag)
                    eStatus = M_Busy;
                else 
                    eStatus = M_Done;                
            }
            else 
                eStatus = M_Done;
            return eStatus;
        }
    }
    
	//加速段
	if(m_dTCur - dTAcc <= Eps)
	{	
        switch(m_tPlan.eTypeAcc)
        {
        case APType_Para:
        case APType_ParaSin:
            m_dPos = m_dTCur*m_dTCur*m_dTCur*(m_dKaAcc*m_dTCur + m_dKbAcc) + m_dTCur*(m_dKcAcc*m_dTCur + m_dSVel);	
            m_dVel = 4*m_dTCur*m_dTCur*m_dTCur*m_dKaAcc + 3*m_dTCur*m_dTCur*m_dKbAcc + 2*m_dKcAcc*m_dTCur + m_dSVel;
            m_dAcc = 12*m_dTCur*m_dTCur*m_dKaAcc + 6*m_dTCur*m_dKbAcc + 2*m_dKcAcc; 
            break;
        case APType_Sin:
            m_dPos = m_dKaAcc*cos(2*PI/dTAcc*m_dTCur) + m_dKbAcc*m_dTCur*m_dTCur - m_dKaAcc;	
            m_dVel = -2*m_dKaAcc*PI/dTAcc*sin(2*PI/dTAcc*m_dTCur) + 2*m_dKbAcc*m_dTCur;
            m_dAcc = -2*m_dKbAcc*cos(2*PI/dTAcc*m_dTCur) + 2*m_dKbAcc;
            break;
        default:
            break;
        }
        
	}
	else if(m_dTCur - (dTAcc+ m_tPlan.dAddTAcc) <= Eps) 
	{
		double dTTemp = m_dTCur - dTAcc;
		double dTempB = dTTemp*dTTemp*dTTemp;	
		if(m_tPlan.bBackFlag)
		{
			m_dPos = m_dPosAcc + (m_dKaAccA*dTTemp + m_dKbAccA)*dTempB + m_tPlan.dEVel*dTTemp*m_tPlan.dAddKVel;	
			m_dVel = m_tPlan.dEVel*m_tPlan.dAddKVel + 4*dTTemp*dTTemp*dTTemp*m_dKaAccA + 3*dTTemp*dTTemp*m_dKbAccA;;
			m_dAcc = 12*dTTemp*dTTemp*m_dKaAccA + 6*dTTemp*m_dKbAccA;
		}
		else
		{
			m_dPos = m_dPosAcc + (m_dKaAccA*dTTemp + m_dKbAccA)*dTempB + m_tPlan.dEVel*dTTemp;	
			m_dVel = m_tPlan.dEVel + 4*dTTemp*dTTemp*dTTemp*m_dKaAccA + 3*dTTemp*dTTemp*m_dKbAccA;;
			m_dAcc = 12*dTTemp*dTTemp*m_dKaAccA + 6*dTTemp*m_dKbAccA;
		}

	}
	//匀速段
	else if(m_dTCur - (dTAcc + m_tPlan.dAddTAcc + m_tPlan.dTCon) <= Eps)
	{
		if(m_tPlan.bBackFlag)
		{
			m_dPos = m_dPosAcc + m_dPosAccA + m_tPlan.dEVel*(m_dTCur - dTAcc - m_tPlan.dAddTAcc);	
			m_dVel = m_tPlan.dEVel;
			m_dAcc = 0;
		}
		else
		{
			m_dPos = m_dPosAcc + m_dPosAccA + m_tPlan.dEVel*m_tPlan.dAddKVel*(m_dTCur - dTAcc - m_tPlan.dAddTAcc);	
			m_dVel = m_tPlan.dEVel*m_tPlan.dAddKVel;
			m_dAcc = 0;
		}		
	}
	//减速段
	else if(m_dTCur - m_tPlan.dTAll <= Eps)
	{		
		double dTTemp = m_dTCur - (dTAcc + m_tPlan.dAddTAcc + m_tPlan.dTCon);
		double dTempB = dTTemp*dTTemp*dTTemp;
		if(m_tPlan.bBackFlag)
		{
			m_dPos = m_dPosAcc	+ m_dPosAccA + m_tPlan.dEVel*m_tPlan.dTCon + (m_dKaDec*dTTemp + m_dKbDec)*dTempB + m_tPlan.dEVel*dTTemp;		
			m_dVel = m_tPlan.dEVel + 4*dTTemp*dTTemp*dTTemp*m_dKaDec + 3*dTTemp*dTTemp*m_dKbDec;
			m_dAcc = 12*dTTemp*dTTemp*m_dKaDec + 6*dTTemp*m_dKbDec; 
		}
		else
		{
			m_dPos = m_dPosAcc	+ m_dPosAccA + m_tPlan.dEVel*m_tPlan.dAddKVel*m_tPlan.dTCon + m_tPlan.dEVel*m_tPlan.dAddKVel*dTTemp;		
			
            switch(m_tPlan.eTypeAcc)
            {
            case APType_Para:
                m_dPos += (m_dKaDec*dTTemp + m_dKbDec)*dTempB;	
                m_dVel = m_tPlan.dEVel*m_tPlan.dAddKVel + 4*dTTemp*dTTemp*dTTemp*m_dKaDec + 3*dTTemp*dTTemp*m_dKbDec;
                m_dAcc = 12*dTTemp*dTTemp*m_dKaDec + 6*dTTemp*m_dKbDec; 
                break;
            case APType_Sin:
            case APType_ParaSin:
                m_dPos += m_dKaDec*cos(2*PI/m_tPlan.dTDec*dTTemp) + m_dKbDec*dTTemp*dTTemp - m_dKaDec;	
                m_dVel = -2*m_dKaDec*PI/m_tPlan.dTDec*sin(2*PI/m_tPlan.dTDec*dTTemp) + 2*m_dKbDec*dTTemp + m_tPlan.dEVel*m_tPlan.dAddKVel;
                m_dAcc = -2*m_dKbDec*cos(2*PI/m_tPlan.dTDec*dTTemp) + 2*m_dKbDec;
                break;
            default:
                break;
            }
		}		
	}
	else	//执行停止
	{	
		if(!m_bRePlanFlag)
		{
			if(fabs(m_dPos + m_dSPos - m_tPlan.dDis) > fabs(m_tPlan.dEVel*m_dCycle)&&fabs(m_dPos + m_dSPos - m_tPlan.dDis) > 0.001)
			{
				LOG_ALGO("Error CalcPos:Pos = %.6lf,SetDis = %.6lf",m_dPos + m_dSPos,m_tPlan.dDis);
				dPos = m_dPos + m_dSPos;
				return M_Error;
			}
			else
			{
				m_dSPos = m_tPlan.dDis - m_dPos;
				dPos = m_dPos + m_dSPos;
				eStatus = M_Done;
			}	
		}	
		else
		{
			m_dAcc = 0;
			m_dVel = 0;
		}
	}

	m_dTCur += m_dCycle;
	dPos = m_dPos + m_dSPos;

    if(m_bFilterOpenFlag)
    {
        bool bFilterDoneFlag = false;
        bFilterDoneFlag = m_HS_FilterHandle.FilterHandle(&dPos,&dPos);
        if(eStatus == M_Done)
        {
            if(!bFilterDoneFlag)
                eStatus = M_Busy;
        }
        double dFilterVel = 0;
        dFilterVel = (dPos - m_dFilterPos)/m_dCycle;
        m_dFilterAcc = (dFilterVel - m_dFilterVel)/m_dCycle;
        dFilterVel += m_dFilterAcc*m_dCycle/2;

        m_dFilterPos = dPos;
        m_dFilterVel = dFilterVel;
        m_dFilterAcc = m_dFilterAcc;
    }
    else
    {
        m_dFilterPos = dPos;
        m_dFilterVel = m_dVel;
        m_dFilterAcc = m_dAcc;
    }

	return 	eStatus;
}
/************************************************
函数功能：执行停止运动规划
参    数：无
返 回 值：无
*************************************************/
int HS_VelPlan_Para::StopPlan(double dTStop)
{
    m_tPlan.dTAcc = dTStop;
    m_tPlan.dTAll = dTStop;
    m_tPlan.eTypeVel = TYPEVP_Stop;
    m_dKaAcc = (m_dAcc*dTStop + 2*m_dVel)/(4*dTStop*dTStop*dTStop);
    m_dKbAcc = (-3*m_dVel - 2*m_dAcc*dTStop)/(3*dTStop*dTStop);
    m_dKcAcc = m_dAcc/2;
    m_dPosAcc = m_dAcc*dTStop*dTStop/12 + m_dVel*dTStop/2;
    m_dSVel = m_dVel;
    m_dSAcc = m_dAcc;
    m_dSPos = m_dPos + m_dSPos;
    m_dTCur = m_dCycle;
    return 0;
}
/************************************************
函数功能：修改运动模式，使得停止目标点确定
参    数：无
返 回 值：无
*************************************************/
int HS_VelPlan_Para::SyncStop(void)
{
	m_dTCur += m_dCycle;
	return 0;
}
/************************************************
函数功能：获取实时速度
参    数：dVel---实时速度
返 回 值：无
*************************************************/
int HS_VelPlan_Para::GetVel(double &dVel)
{
	dVel = m_dVel;
	return 0;
}
/************************************************
函数功能：获取实时加速度
参    数：dAcc---实时加速度
返 回 值：无
*************************************************/
int HS_VelPlan_Para::GetAcc(double &dAcc)
{
	dAcc = m_dAcc;
	return 0;
}
/************************************************
函数功能：获取滤波前位置
参    数：dPos---滤波前位置
返 回 值：无
*************************************************/
int HS_VelPlan_Para::GetOrignPos(double &dPos)
{
    dPos = m_dPos + m_dSPos;
    return 0;
}
/************************************************
函数功能：获取滤波后速度
参    数：dVel---滤波后速度
返 回 值：无
*************************************************/
int HS_VelPlan_Para::GetFilterVel(double &dVel)
{
    dVel = m_dFilterVel;
    return 0;
}
/************************************************
函数功能：获取滤波后加速度
参    数：dVel---滤波加速度
返 回 值：无
*************************************************/
int HS_VelPlan_Para::GetFilterAcc(double &dAcc)
{
    dAcc = m_dFilterAcc;
    return 0;
}
/************************************************
函数功能：速度调节
参    数：无
返 回 值：无
*************************************************/
int HS_VelPlan_Para::Ratio(VelPlanPara tPara)
{	
	m_tPlan = tPara;

    //距离优化
    //m_tPlan.dDis -= (m_dPos+m_dSPos);

	//1、加速度段
	if(m_tPlan.dTAcc > Eps)
	{		
		m_dKcAcc = m_dAcc/2;
		m_dSAcc = m_dAcc;
		m_dSVel = m_dVel;

		m_dKaAcc = (m_dSAcc*m_tPlan.dTAcc + 2*(m_dSVel - m_tPlan.dEVel))/(4*m_tPlan.dTAcc*m_tPlan.dTAcc*m_tPlan.dTAcc);
		m_dKbAcc = (3*(m_tPlan.dEVel - m_dSVel) - 2*m_dSAcc*m_tPlan.dTAcc)/(3*m_tPlan.dTAcc*m_tPlan.dTAcc);			
		m_dPosAcc = m_dAcc*m_tPlan.dTAcc*m_tPlan.dTAcc/12 + (m_dSVel+m_tPlan.dEVel)*m_tPlan.dTAcc/2;
	}		

	//2、减速段，时间为dTDec,减速段的启动加速度为0，所以不包含Kc项
	if(m_tPlan.dTDec > Eps)
	{
		m_dKaDec = m_tPlan.dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
		m_dKbDec = - m_tPlan.dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);		
	}

	m_dTCur = m_dCycle;
	m_dSPos = m_dPos+m_dSPos;
	return 0;
}

/************************************************
函数功能：调节当前规划时间，提前停止运动
参    数：
		dTRePlan------修改调节的时间
		bLastPlanFlag-最后一段规划
返 回 值：
*************************************************/
double HS_VelPlan_Para::RePlanByTime(double &dTRePlan,bool bLastPlanFlag)
{	
	m_dSPos += m_dPos;

	if(!bLastPlanFlag)
	{
		if(dTRePlan > m_dTAcc*2)
		{
			m_tPlan.dTAcc = m_dTAcc;
			m_tPlan.dTCon = dTRePlan - m_dTAcc*2;
			m_tPlan.dTDec = m_dTAcc;
		}
		else
		{
			m_tPlan.dTCon = 0;
			m_tPlan.dTAcc = dTRePlan/2;
			m_tPlan.dTDec = dTRePlan/2;
		}

		m_dKaAcc = - m_tPlan.dEVel/(2*m_tPlan.dTAcc*m_tPlan.dTAcc*m_tPlan.dTAcc);
		m_dKbAcc = m_tPlan.dEVel/(m_tPlan.dTAcc*m_tPlan.dTAcc);		

		m_dPosAcc = m_tPlan.dEVel*m_tPlan.dTAcc/2;

		m_dKaDec = m_tPlan.dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
		m_dKbDec = - m_tPlan.dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);

		m_tPlan.dTAll = m_tPlan.dTAcc + m_tPlan.dTCon + m_tPlan.dTDec;

		m_dTLast = m_dTLast - (m_tPlan.dTAcc + m_tPlan.dTCon);

		//运动完成后不返回Done	
		m_bRePlanFlag = true;
	}
	else
	{
		//按照给定的时间进行规划
		if(dTRePlan > m_dTAcc*2)
		{
			m_tPlan.dTAcc = m_dTAcc;
			m_tPlan.dTCon = dTRePlan - m_dTAcc*2;
			m_tPlan.dTDec = m_dTAcc;
		}
		else
		{
			m_tPlan.dTCon = 0;
			m_tPlan.dTAcc = dTRePlan/2;
			m_tPlan.dTDec = dTRePlan/2;
		}

		//基于位移量，重新计算速度
		double dDis = m_tPlan.dDis - m_dSPos;
		m_tPlan.dEVel = dDis/(m_tPlan.dTAcc + m_tPlan.dTCon);

		m_dKaAcc = - m_tPlan.dEVel/(2*m_tPlan.dTAcc*m_tPlan.dTAcc*m_tPlan.dTAcc);
		m_dKbAcc = m_tPlan.dEVel/(m_tPlan.dTAcc*m_tPlan.dTAcc);		

		m_dPosAcc = m_tPlan.dEVel*m_tPlan.dTAcc/2;

		m_dKaDec = m_tPlan.dEVel/(2*m_tPlan.dTDec*m_tPlan.dTDec*m_tPlan.dTDec);
		m_dKbDec = - m_tPlan.dEVel/(m_tPlan.dTDec*m_tPlan.dTDec);

		m_tPlan.dTAll = m_tPlan.dTAcc + m_tPlan.dTCon + m_tPlan.dTDec;

		m_bRePlanFlag = false;
	}

	m_dTCur = m_dCycle;

	return m_dTLast;
}