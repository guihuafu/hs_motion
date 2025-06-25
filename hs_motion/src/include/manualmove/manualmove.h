#pragma once

#include "motionpara.h"
#include "manualmoveparadef.h"

namespace hsc3
{
namespace algo
{

// 手动算法基类
class ManualMove
{
public:
	DLL_EXPORT ManualMove(MotionPara *para,double dCycle){}
	DLL_EXPORT virtual ~ManualMove(){}

    /**
	* @brief  打印关键参数信息
	* @return  错误码
	*/
    DLL_EXPORT virtual int execPrintKeyInfo() = 0;

	/**
	 * @brief  运动规划
	 * 输入参数：mpara		手动运动上层接口参数
	 * 输入参数：curpos 		当前关节位置值
	 * @return 错误码
	 */

	DLL_EXPORT virtual int Plan(HS_GroupJPos &tHS_GroupJPos, ManualPara tManualPara) = 0;

	/**
	 * @brief  周期插补
	 * 输出参数：iErrorId----错误码
	 * 		    dhandJPos---关节位置，插补计算位移，关节角度值
	 * @return 插补规划状态
	 */
	DLL_EXPORT virtual HS_MStatus Move(int &iErrorId,HS_GroupJPos &tHS_GroupJPos)  = 0;

	/**
	 * @brief  停止运动	 
	 * @return 错误码
	 */
	DLL_EXPORT virtual int StopPlan() = 0;  
};

}
}