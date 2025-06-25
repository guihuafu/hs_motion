/*
* Copyright (c) 2017
* All rights reserved.

* �ļ����ƣ�HS_FilterHandleLib.h
* ժ    Ҫ���˲�������ӿڣ���λ�ý����˲��Ż�
* ��ǰ�汾��
* ��    ��:
* ������ڣ�
*			
*/
#ifndef _HS_FILTERHANDLE_H
#define _HS_FILTERHANDLE_H


#include "algoglobalparadef.h"
#include "HS_BasicPara.h"

#define FIRBUFFCNT          1000
#define HANDLEFIRPOS        1           //�˲���λ�ã�1��ǰ�ã�2�Ǻ��ã�3��ǰ���� 

#define MaxAxisNum 9

#define PI			3.1415926535898

class HS_FilterHandle
{
public:
    HS_FilterHandle(double dCycle);	
	HS_FilterHandle();
    ~HS_FilterHandle(void);
    void ResetFilter();
    bool FilterHandle(double *dJPosIn,double *dJPosOut);
    void Filer_SetPara(FilterPara tFilterPara);
	void SetCycle(double dCycle);
private: 
    double m_dCycle;
    FilterPara m_tFilterPara;
    int m_iIndex;
    double m_dInputBuff[FIRBUFFCNT][MaxAxisNum];
    double m_dOutputBuff[FIRBUFFCNT][MaxAxisNum];

    double m_dKFilter[FIRBUFFCNT];
    int m_iFirNum;
    int m_iShapeOff;
};

#endif
