
#ifndef _AUTO_CONTROL_H
#define _AUTO_CONTROL_H

#include "algoglobalparadef.h"
#include "MathOperation.h"
#include "automoveparadef.h"

using namespace hsc3::algo;

#define TIMECNT			4
#define STOPTIMEMAX 	0.5                 //最大减速时间
#define SpaceAxisNum	5

enum TIMETYPE
{
	TACC = 0,
	TDEC,
	TCON,
	TALL,
};

enum DynPlanState
{
    DYNPLAN_None = 0,
    DYNPLAN_Start,
    DYNPLAN_Move,
    DYNPLAN_End,
    DYNPLAN_EndSmooth,
};


//定义预处理存储结构体-----统一使用
typedef struct _PreHandle
{
	/***********公共参数************/
    HS_MOVETYPE eMoveType;                      //运动类型
    bool bPreHandled;                           //是否经过预处理
    int iPreHandledError;                       //预处理报警码
	double dSPos[MaxAxisNum];
    double dMPos[MaxAxisNum];
	double dEPos[MaxAxisNum];
	double dDis[MaxAxisNum];					//XYZ和欧拉角位移
	double dSetDis[MaxAxisNum];					//实际执行位移量，暂停重启会使得位移量发生变化
	double dSetVel[MaxAxisNum];
	double dSetAcc[MaxAxisNum];
	double dSetDec[MaxAxisNum];	
	double dSetKVel[MaxAxisNum];				//速度缩放比例系数，约束最大速度
    double dRealKJVel;                  		//实际的关节速度比例值

    bool bTackPosRePlan;                		//传送带跟踪条件下的点位修正标识，只能处理一次
	bool bRepeatPos;							//是否是重复点位
	int iMaxAxis;								//有效轴，对比位移和姿态，一般取位移量，姿态变化大时为姿态
	double dSetMaxAcc;					        //设定的Acc最大比例系数值
	bool bWristQYOpenFlag;						//开启腕部过奇异
	bool bWristQyFlag;							//过腕部奇异处理

	/***********直线参数************/
	double dLength;
	double dKXYZ[3];
	double dQ[4];						        //姿态四元数变化矩阵

	double dRadius[4];					        

	double dOrigDis[SpaceAxisNum];		        //轨迹原始位移量
	double dDisSmooth[2];				        //平滑长度【前后】	

	double dSetKAcc[2];					        //加速度缩放比例系数，加减速段
    double dSmoothCAngle;                       //与前一段的夹角值
    double dOrigKJVel;                          //预处理的关节速度比例值
    double dPreHandleVel[SpaceAxisNum];         //预处理的空间速度值---对应关节速度比例值，用来进行Smooth计算

	bool bSmallLineFlag;				        //小线段离散化处理标识，是否还需要进行动态离散化

	double dSetJPos[2][MaxAxisNum];		        //对起、末点的关节坐标进行缓存，用来点位匹配以及一致性处理
	unsigned char eState;				        //当前段形态缓存，空间运动不改变形态【低3位】

    int bJVelDir[2][MaxAxisNum];                //当前空间运动对应匀速段的，启停段的关节轴速度方向 = 0代表 负向，1代表正向，提供给平滑运动使用

	/***********圆弧参数************/
    double dCircleR;
    double dMTrans[4][4];				        //坐标系转换矩阵
    bool bMidOutFlag;                           //中间点位是否在起末点之外

    /***********通用参数***************/
    //传送带跟踪参数
    double dTrackOrigXPos;

    //工具工件保存
    int iToolNum;
    int iWorkNum;

    double dKIBase;                     		//电流比例基准值

    bool  bSmoothCRMode;                     	//平滑参数定义含义，0----代表百分比定义；1---代表绝对位移量定义
    bool bLimit5Flag;

    /***********动态规划***************/
    DynPlanState eDynState;
    double dBzConPos[6][6];                     //贝塞尔平滑控制点【与前段】

	/**********组协同运动******************/
	bool bGroupSyncFlag;						//是否开启协同运动
	int iSyncMasterNum;
	double dWTSPos[MaxAxisNum];
	double dWTEPos[MaxAxisNum];

	bool bCoorperMoveFlag;						//是否开启变位机运动

}Para_PreHandle;

//定义直线速度规划结构体
typedef struct _VelPlan
{
	/***********规划信息【公共】************/
	double dTime[TIMECNT];
	double dTSmooth[2];
	bool bReachMaxVel;		

	double dEVel[MaxAxisNum];
	double dAcc[MaxAxisNum];
	double dDec[MaxAxisNum];
	double dJerk[MaxAxisNum];

	double dStopAcc[MaxAxisNum];		    		//停止参数
	double dStopJerk[MaxAxisNum]; 

	/***********规划信息【直线】************/
	double dKSmoothAcc;                     		//平滑段的加速度系数缓存,代表与上一段的平滑段处理    	
    bool bSpeedUpOK;                        		//是否已经满足提速需求点
    bool bSLinePlan;                        		//是否进行了小线段的规划，对调速以及停止都需要有不同的处理方式

}Para_VelPlan;

//静态规划缓存参数
typedef struct _SYNCPARA
{
	Para_PreHandle tPreHandle;						//代表输入信息--提供给速度规划
	Para_VelPlan   tVelPlan;						//速度规划得到结果
}SyncPara;

//加速度规划方式
enum TypeAccPlan
{
    APType_Para	= 0,			//二次抛物线
    APType_Sin,			        //正余弦	
    APType_ParaSin,             //抛物线启动，正余弦停止
};

enum eTypeVelPara
{
    TYPEVP_Normal	= 0,		//正常模式		
    TYPEVP_Stop,			    //停止规划
};

typedef struct _VelPlanPara
{	
	double dTAcc;				//加速段时间
	double dTCon;				//匀速段时间
	double dTDec;				//减速段时间
	double dTAll;				//总时间
	double dTSmoothOff;

	double dDis;				//运动距离
	double dEVel;				//结束速度

	//增加一段加减速运动
	bool bAddModeFlag;
	bool bBackFlag;		        //反向模式
	double dAddKVel;
	double dAddTAcc;	

    double dSVel;				//启动速度
    double dSAcc;				//启动加速度
	
    TypeAccPlan eTypeAcc;
	eTypeVelPara eTypeVel;		//规划类型
}VelPlanPara;

#endif