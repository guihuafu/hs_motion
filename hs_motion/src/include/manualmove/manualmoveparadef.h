#pragma once

#include "algoglobalparadef.h"

// 手动运行相关结构体
namespace hsc3
{
namespace algo
{

    typedef struct _ManualPara
    {	
        Hs_Coordinate hs_coordinate;        //运动坐标系 
        int iAxisNum;	                    //轴号
		int iGroupNum;						//组号
		int iToolNum[MAXGROUPNUM];			//多机器人轴组协同时从机的机器人组指定工具号
		HS_GroupRel tHS_GroupRel;			//轴组关系
        bool bDir;		                    //方向
        double dHandVelRatio;	            //运动速率
        double dIncLen;	                    //寸动距离。大于 0 代表寸动 ； 等于 0 代表连续
        int iSmooth;                        //点动柔顺等级
        bool bWristQYOpen;                  //是否开启腕部过奇异
		double dVtran;						//空间速度
		double dVrot;						//姿态速度
		bool bCoorperMove;					//变位机运动【附加轴变位机使用】

    }ManualPara;    
}
}