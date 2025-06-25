#pragma once

#include "algoglobalparadef.h"
#include <string>
#include <vector>
#include "motionpara.h"

using namespace std;

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

typedef struct _DB_MotionPara
{
    int iId;
    MotionPara tMotionPara;
}DB_MotionPara;

class DLL_EXPORT MotionParaManager
{
public:
    DLL_EXPORT static MotionParaManager* GetInstance();
    /**
	* @brief  创建运动参数
	* 输入参数：id  id号
               motionP  输入参数
	*/
	DLL_EXPORT void creatMotionPara(int id,MotionPara &motionP);
    /**
	* @brief  删除运动参数
	* 输入参数：id  id号
	*/
	DLL_EXPORT void deleteMotionPara(int id);
    /**
	* @brief  获取运动参数
	* 输入参数：id  id号
	*/
	DLL_EXPORT MotionPara* getMotionPara(int id);

private:

    // 禁止外部析构
    ~MotionParaManager();

private:
    vector<DB_MotionPara> m_tDB_MotionPara;
};
}
}