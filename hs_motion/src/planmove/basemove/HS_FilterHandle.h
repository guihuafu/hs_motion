/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_FilterHandleLib.h
* 摘    要：滤波器处理接口，对位置进行滤波优化
* 当前版本：
* 作    者:
* 完成日期：
*			
*/
#ifndef _HS_FILTERHANDLE_H
#define _HS_FILTERHANDLE_H


#include "algoglobalparadef.h"
#include "HS_BasicPara.h"

#define FIRBUFFCNT          1000
#define HANDLEFIRPOS        1           //滤波器位置，1是前置，2是后置，3是前后置 

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
