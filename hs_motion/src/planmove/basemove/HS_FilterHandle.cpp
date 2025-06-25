/*
    * Copyright (c) 2017
    * All rights reserved.

    * �ļ����ƣ�HS_MotionPara.cpp
    * ժ    Ҫ�������˲�����ӿڣ�ͨ����ͬ���˲��������Բ岹�滮�����Ż�����

    * ��ǰ�汾��
    * ��    �ߣ�
    * ������ڣ�
    *			
    */
#include "HS_FilterHandle.h"
#include <string.h>
#include <math.h>
/************************************************
�������ܣ����캯��
��    ����
�� �� ֵ����
*************************************************/
HS_FilterHandle::HS_FilterHandle(double dCycle)
{
    m_dCycle = dCycle;

    //Ĭ�ϲ���
    m_tFilterPara.eFilterType = TYPE_FIR;
    m_tFilterPara.dFre = 10.0;
    m_tFilterPara.iGrade = 10;

    ResetFilter();
}

HS_FilterHandle::HS_FilterHandle()
{
	m_dCycle = 0.004;

	//Ĭ�ϲ���
	m_tFilterPara.eFilterType = TYPE_FIR;
	m_tFilterPara.dFre = 10.0;
	m_tFilterPara.iGrade = 10;

	ResetFilter();
}

HS_FilterHandle::~HS_FilterHandle(void)
{

}
/************************************************
�������ܣ�������ʼ��
��    ����
�� �� ֵ����
*************************************************/
void HS_FilterHandle::SetCycle(double dCycle)
{
	m_dCycle = dCycle;
}
/************************************************
�������ܣ�������ʼ��
��    ����
�� �� ֵ����
*************************************************/
void HS_FilterHandle::ResetFilter()
{
    memset(m_dInputBuff,0,sizeof(m_dInputBuff));
    memset(m_dOutputBuff,0,sizeof(m_dOutputBuff));
    m_iIndex = -1;

}
/************************************************
�������ܣ������˲�����Ӧ�������Լ�������
��    ����
�� �� ֵ����
*************************************************/
void HS_FilterHandle::Filer_SetPara(FilterPara tFilterPara)
{
    m_tFilterPara = tFilterPara;

    ResetFilter();

    if(tFilterPara.dFre < 0.001)
        tFilterPara.dFre = 0.5;

    switch(m_tFilterPara.eFilterType)
    {
    case TYPE_FIR:
        m_iFirNum = (int)(1.0/(m_tFilterPara.dFre*m_dCycle));
        if(m_iFirNum < 1)
            m_iFirNum = 1;
        else if(m_iFirNum > FIRBUFFCNT)
            m_iFirNum = FIRBUFFCNT;
        //���δ�
        for(int i = 0;i < m_iFirNum;i++)
        {
            m_dKFilter[i] = 1.0/m_iFirNum;
        }
        break;
    case TYPE_IIR:
        //��һ���˲�
        m_dKFilter[0] = 0.033921923;
        m_dKFilter[1] = 0.033921923;
        m_dKFilter[2] = 0.932156153;
        break;
    case TYPE_SHAPE:
        {
            double dW = 2*PI*m_tFilterPara.dFre;
            double dZeta = 0.05;
            double dTd = PI/(dW*sqrt(1 - dZeta*dZeta));
            double dK = exp(-PI*dZeta/sqrt(1 -dZeta*dZeta));
            double dA1 = 1/(1 + dK);
            double dA2 = dK/(1 + dK);
            //��ZV������
            m_iShapeOff = (int)(dTd/m_dCycle)+1;
            m_dKFilter[0] = dA1;
            m_dKFilter[1] = dA2;
        }
        break;
    default:
        break;
    }

    //LOG_ALGO("Filer_SetPara Type:%d,Fre:%.3lf,Grade:%d",tFilterPara.eFilterType,tFilterPara.dFre,tFilterPara.iGrade);
}
/************************************************
�������ܣ�ʹ���˲��������ݽ��д���ִ��
��    ����
�� �� ֵ���˲���������ź�
*************************************************/
bool HS_FilterHandle::FilterHandle(double *dJPosIn,double *dJPosOut)
{
    double dJVel[MaxAxisNum] = {0};
    //��ʼ������
    if(m_iIndex == -1)
    {
        for(int i = 0;i < FIRBUFFCNT;i++)
        {
            for(int j = 0;j < m_tFilterPara.iAxisNum;j++)
            {
                m_dInputBuff[i][j] = dJPosIn[j];
                m_dOutputBuff[i][j] = dJPosIn[j];
            }
        }
        m_iIndex = 0;
    }

    for(int j = 0;j < m_tFilterPara.iAxisNum;j++)
    {
        m_dInputBuff[m_iIndex][j] = dJPosIn[j];
    }

    //�˲�������
    switch(m_tFilterPara.eFilterType)
    {
    case TYPE_FIR:
        for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
        {
            dJPosOut[i] = 0;

            for(int iCnt = 0;iCnt < m_iFirNum;iCnt++)
            {
                int iIndex = m_iIndex - (m_iFirNum - 1 - iCnt);
                if(iIndex < 0)
                    iIndex += FIRBUFFCNT;
                dJPosOut[i] += m_dInputBuff[iIndex][i]*m_dKFilter[iCnt];
            }
        }
        break;
    case TYPE_IIR:
        {
            int iIndexPre = m_iIndex - 1;
            if(iIndexPre < 0)
                iIndexPre += FIRBUFFCNT;
            for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
            {
                dJPosOut[i] = m_dKFilter[0]*m_dInputBuff[m_iIndex][i] + m_dKFilter[1]*m_dInputBuff[iIndexPre][i] + m_dKFilter[2]*m_dOutputBuff[iIndexPre][i];   
            }
        }
        break;
    case TYPE_SHAPE:
        {
            int iIndexPre = m_iIndex - m_iShapeOff;
            if(iIndexPre < 0)
                iIndexPre += FIRBUFFCNT;
            for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
            {
                dJPosOut[i] = m_dKFilter[0]*m_dInputBuff[iIndexPre][i] + m_dKFilter[1]*m_dInputBuff[m_iIndex][i];   
            }
        }
        break;
    default:
        for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
        {
            dJPosOut[i] = dJPosIn[i];
        }
        break;
    }

    //�ٶȽ�Ϊ0��ֵ�ж�
    for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
    {
        int iIndexPre = m_iIndex - 1;
        if(iIndexPre < 0)
            iIndexPre += FIRBUFFCNT;
        dJVel[i] = (dJPosOut[i] - m_dOutputBuff[iIndexPre][i])/m_dCycle;
    }

    for(int j = 0;j < m_tFilterPara.iAxisNum;j++)
    {
        m_dOutputBuff[m_iIndex][j] = dJPosOut[j];
    }

    m_iIndex = (m_iIndex + 1)%FIRBUFFCNT;

    const double dZeroVel = 1e-2;
    for(int i = 0;i < m_tFilterPara.iAxisNum;i++)
    {
        if(fabs(dJVel[i]) > dZeroVel)
        {
            return false;
        }
    }

    return true;
}

