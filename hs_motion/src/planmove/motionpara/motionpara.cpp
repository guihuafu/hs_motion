#include "motionpara.h"
#include "HS_BasicPara.h"
#include <string.h> 

namespace hsc3
{
namespace algo
{

    MotionPara::MotionPara()
    {
		memset(m_tGroupStaticPara,0,sizeof(m_tGroupStaticPara));

        //ME2.0.0.2404245:增加版本号信息；添加自动暂停以及Joint获取接口；添加世界坐标系接口；修改新形态位定义等；
        //ME2.0.0.2404246:修改SetPos检测为指针；
        //ME2.0.0.240507:增加形态接口，完善直线圆弧规划速度倍率处理；
        //ME2.0.0.240617:完善功能
        //ME2.0.0.240624:结合测试仿真完善报警码等；
        //ME2.0.0.240625:接口完善；
		//ME2.0.0.240726:修复平滑Joint信号输出问题；
		//ME2.0.0.240913:算法版本同步，动态规划功能完善；
		//ME2.0.0.240926:点动倍率修改为全部由逻辑层控制；
		//ME2.0.0.240928:修改空间点动速度默认值【更改接口】；
		//ME2.0.0.241007:处理规划与插补周期不一致的平滑规划问题；
		//ME2.0.0.241012:内存浅拷贝导致的析构异常；
		//ME2.0.0.241029:进行多轴组方案的设计开发，架构调整；
		//ME2.0.0.241116:多轴组方案的完善补充；
		//ME2.0.0.250102:支持机器人协同运动；增加完善接口；
		//ME2.0.0.250226:增加CR属性支持以及距离输出；
		//ME2.0.0.250307:增加圆弧关节角度预测处理，避免角度预测异常；
		//ME2.0.0.250314:增加六轴协作机器人Cobot模型支持；
		//ME2.0.0.250315:协作机型形态修改;
		//ME2.0.0.250318:协作机型逆解bug处理;
		//ME2.0.0.250325:多轴组重复点位处理优化;增加变位机协同处理方案；
		//ME2.0.0.250328:增加Co系列过腕部奇异功能
		//ME2.0.0.250329:Co点动过腕部奇异增加起点5轴位置的判断，避免由于过奇异处理影响了姿态角度；
		//ME2.0.0.250401:协作机器人逆解算法优化；
		//ME2.0.0.250402:协作机器人带工具的奇异处理算法优化；重复点位处理优化；
		//ME2.0.0.250403:奇异+重复点位的方案优化处理；
		//ME2.0.0.250406:重新规划的位移量计算错误处理；奇异设置错误；
		//ME2.0.0.250409:BR机型形态设置错误导致逆解异常；增加运行行LineNum的判断；
		//ME2.0.0.250421:摆焊功能同步完善中，初步完成直线和圆弧端点停留摆焊方案的开发以及验证，端点不停留方案开发中；【接口变更】
		//ME2.0.0.250423:摆焊功能初步开发完成，暂停重启会原点功能待开发完善；
		//ME2.0.0.250429:完成摆焊暂停以及重启功能，直线回摆焊主运动点位；支持回点位过程中继续暂停；
		//ME2.0.0.250510:逆解算法完善处理，增加工具雅可比求解，满足欠自由度模型工具精度达标；
		//ME2.0.0.250514:增加4轴限定的过奇异处理方案，完善前瞻以及自适应处理，支持混合平滑；支持工具工件切换的混合平滑；
		//ME2.0.0.250522:修改完善动态规划算法处理，完善减速停止以及重启；耗时待上机进一步测试；
		//ME2.0.0.250526:摆焊算法优化同步；动态平滑处理下平滑标识设置异常导致起点点位检测失效异常；
		//ME2.0.0.250528:修复动态平滑信号异常导致的点位错误；多轴组LineNum清楚应等全部规划完进行；
		//ME2.0.0.250529:BR系列开启腕部过奇异功能同样需要识别起点5轴的关节角度值；
        m_strVision = "ME2.0.0.250529";
    }

    MotionPara::~MotionPara()
    {

    }

	void MotionPara::setGroupStaticPara(GroupStaticPara tGroupStaticPara[MAXGROUPNUM])
	{
		LOG_ALGO("Set Group Static Para!");
		memcpy(m_tGroupStaticPara,tGroupStaticPara,sizeof(m_tGroupStaticPara));
	}
	void MotionPara::getGroupStaticPara(GroupStaticPara tGroupStaticPara[MAXGROUPNUM])
	{
		memcpy(tGroupStaticPara,m_tGroupStaticPara,sizeof(m_tGroupStaticPara));
	}

    string MotionPara::GetVision(void)
    {
        return m_strVision;
    }
}
}