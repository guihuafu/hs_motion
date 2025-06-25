#pragma once

#include "algoglobalparadef.h"

#include <string>

using namespace std;

#ifdef _LINUX_
    #include <sys/time.h>      //获取系统时间
    #include <time.h>
#endif

#ifndef DLL_EXPORT
#ifndef _LINUX_
    #define DLL_EXPORT //__declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif
#endif

namespace hsc3
{
namespace algo
{

//算法接口参数--对外接口
class DLL_EXPORT MotionPara
{
public:
    DLL_EXPORT MotionPara();
    DLL_EXPORT ~MotionPara();

	/**
	* @brief  设置以及获取轴组的全部静态参数
	* 输入输出参数：多组轴组静态参数
	*/
	DLL_EXPORT void setGroupStaticPara(GroupStaticPara tGroupStaticPara[MAXGROUPNUM]);
	DLL_EXPORT void getGroupStaticPara(GroupStaticPara tGroupStaticPara[MAXGROUPNUM]);

	/**
	* @brief  获取算法版本号信息
	* 返回参数：算法版本号信息
	*/
    DLL_EXPORT string GetVision(void);

public:
    string m_strVision;                             //算法版本号

	GroupStaticPara m_tGroupStaticPara[MAXGROUPNUM];
};
}
}