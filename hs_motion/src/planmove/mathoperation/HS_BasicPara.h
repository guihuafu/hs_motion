/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_BasicPara.h
* 摘    要：算法计算的公共参数管理【单例】

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_BASICPARA_H
#define _HS_BASICPARA_H

#include "motionpara.h"
#include "MathOperation.h"

using namespace hsc3::algo;

#ifndef _LINUX_
#define filename(x) strrchr(x,'\\')?strrchr(x,'\\')+1:x
#else
#define filename(x) strrchr(x,'/')?strrchr(x,'/')+1:x
#endif


//#define LOG_ALGO(format,...)	
//#define LOG_ALGO(format,...)	printf("[ALGO][%s][%s][%d]" format "\n",filename(__FILE__),__FUNCTION__,__LINE__,##__VA_ARGS__)
#ifdef _LINUX_
#define LOG_ALGO(format,...)	NANO_LOG("ALG","[%s:%d]" format ,filename(__FILE__),__LINE__,##__VA_ARGS__) //NANO_LOG("ALG","[%s:%d]" format ,filename(__FILE__),__LINE__,##__VA_ARGS__)	//NANO_LOG("ALG",format ,##__VA_ARGS__)
#else
#define LOG_ALGO(format,...)	printf("[ALGO][%s][%d]" format "\n",filename(__FILE__),__LINE__,##__VA_ARGS__)		//LOG_I("[ALGO]" format ,##__VA_ARGS__)
#endif


//系统整形滤波控制
struct SysFilterPara
{
    FilterControl tHandControl;
    FilterControl tAutoControl;
    int iAutoFilterType;                        //整形滤波类型，0--单段【1--完整】
};

/*
#ifdef _LINUX_
	struct timeval l_beginTime_Auto, l_endTime_Auto;
	gettimeofday(&l_beginTime_Auto, NULL);
#endif

#ifdef _LINUX_
	gettimeofday(&l_endTime_Auto, NULL);
	double de_us_Auto = 1000000*(l_endTime_Auto.tv_sec - l_beginTime_Auto.tv_sec) + (l_endTime_Auto.tv_usec - l_beginTime_Auto.tv_usec);
	
	LOG_ALGO("Time_Prehandle = %.3lf\n",de_us_Auto);
#endif

*/

class HS_BasicPara
{

public:
    // 获取单实例对象
    static HS_BasicPara* GetInstance();
	
	// 设置参数
    void SetPara(MotionPara *para);

    // 设置插补周期
    void SetCycle(double dCycle,int iInterMultCnt);

private:
    // 禁止外部构造
    HS_BasicPara();

    // 禁止外部析构
    ~HS_BasicPara();
public:
    double HS_GetTFre(double dJPos[6],int iSmooth,int iGroupNum);
	double CalcTProtectByVel(double dTFreProtect,double dKJVel,int iGroupNum);
public:
    MotionPara *mMotionPara;

    double m_dCycle;
    int m_iInterMultCnt;
    SysFilterPara m_tSysFilterPara;
private:
	double *m_dJVelPara;
	double *m_dJAccPara;
};


#endif