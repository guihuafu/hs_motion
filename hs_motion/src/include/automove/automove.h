#pragma once

#include "motionpara.h"
#include "automoveparadef.h"

namespace hsc3
{
namespace algo
{
// 规划算法接口
class AutoMove
{
public:
	DLL_EXPORT AutoMove(MotionPara *para,double dCycle,int iInterMultCnt){}
	DLL_EXPORT virtual ~AutoMove(){}
	/**
	* @brief  打印关键参数信息
	* @return  错误码
	*/
    DLL_EXPORT virtual int execPrintKeyInfo() = 0;
    	/**
	* @brief  自动运行系统复位
	* @return 
	*/
    DLL_EXPORT virtual void execReset() = 0;
	/**
	* @brief  执行点位预处理，算法对点位进行预处理，非每个周期固定执行，当执行器下发点位时调用
	* @param  elemt  点位运动信息，执行器原始数据
	* @param  trajout  处理缓存基地址
	* @param  index  当前处理
	* @return  错误码
	*/
	DLL_EXPORT virtual int execPrehandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex) = 0;
	/**
	* @brief  普通轨迹规划
	* @param  trajout  规划信息基地址
	* @param  index    当前行索引号
	* @param  ratio    倍率
	* @param  realjpos  实际的关节角度值
	* @return  错误码
	*/
	DLL_EXPORT virtual int execPlanMove(GroupTrajData *tTrajData,int iIndex,double dRatio,HS_GroupJPos &tHS_GroupJPos) = 0;
	/**
	* @brief  停止轨迹规划
	* @return  是否成功规划
	*/
	DLL_EXPORT virtual int execStopPlan() = 0;

	/**
	* @brief  停止段重新规划
    * @param  ratio    倍率
    * @param  realjpos  实际的关节角度值
	* @return  是否成功规划
	*/
	DLL_EXPORT virtual int execStopRestartPlan(double dRatio,HS_GroupJPos &tRealJPos) = 0;
	
	/**
	* @brief  设置运行倍率
	* @param  ratio  倍率
	*/
	DLL_EXPORT virtual int setRatio(double ratio) = 0;
	
	/**
	* @brief  执行插补规划
	* @param  intdata  插补点位信息
	* @return  插补状态
	*/
	DLL_EXPORT virtual HS_MStatus execIntMove(IntData &intdata,int &iErrorId) = 0;

	/**
	* @brief  获取插补Joint行号信息
	* @param  joint数组[4]
	* @return  插补状态
	*/
    DLL_EXPORT virtual int execGetJoint(int *joint) = 0;

	/**
	* @brief  获取平滑拐入点的提前信号量
	* @param  
	* @return  true-----拐入信号量
	*/
	DLL_EXPORT virtual bool GetSmoothAheadFlag() = 0;

	/**
	* @brief  获取当前运行的行id号
	* @param  
	* @return  id号
	*/
	DLL_EXPORT virtual int execGetCurMoveId() = 0;

	/**
	* @brief  判断是否为重复点位
	* @param  
	* @return  true为重复点位
	*/
	DLL_EXPORT virtual bool HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB) = 0;
};

}
}







