#pragma once

// 自动运行相关结构体
namespace hsc3
{
namespace algo
{
#ifdef _LINUX_
    #define MaxBuffSize 40
#else
    #define MaxBuffSize 500
#endif

#define INTBUFF			10

    //插补类型
    enum HS_MOVETYPE
    {
        HS_MOVETYPE_MIN = 0,
        MP_Line_Bezier,					//贝塞尔曲线 
        MP_Line,						//直线
        MP_BLine,						//B样条插补【小线段使用】
        MP_Arc,							//圆弧
        MP_Joint,						//关节曲线
        MP_Joint_SP,					//关节曲线样条插补
        MP_Other,						//其他
        MP_Joint_JUMP,                  //门型曲线
        MP_Arc_Bezier,
        MP_Square,                      //方孔加工
        HS_MOVETYPE_MAX
    };

	//摆动模式类型(其他类型后续引入)
	enum HS_WEAVE_TYPE
	{
		WEAVE_SINE = 0,						//正弦摆动【Z字摆动】
		WEAVE_CIRCLE,						//圆形摆动
		WEAVE_L,							//L型摆动
		WEAVE_MOONF,						//正月牙
		WEAVE_MOONB,						//反月牙
		WEAVE_8,							//8字形摆动
	};

	//摆动模式方向
	enum HS_WEAVE_PLANE
	{
		WEAVE_Y = 0,						//Y轴方向
		WEAVE_Z,							//Z轴方向
		WEAVE_X,							//X轴方向
	};

	//摆动端点停留类型
	enum HS_WEAVE_DTYPE
	{
		WEAVE_DMOVE = 0,					//摆动方向停止但主运动不停留
		WEAVE_DSTOP,						//完全停留
	};

	//摆动参数
	typedef struct _HS_WeavePara
	{
		bool bWeaveEn;						//摆动控制开关
		HS_WEAVE_TYPE eWeaveType;			//摆动模式类型
		HS_WEAVE_PLANE eWeavePlane;			//摆焊方向
		double dAmplitude;					//摆动振幅
		double dFrequency;					//摆动频率Hz
		HS_WEAVE_DTYPE eWeaveDType;			//端点停留类型
		double dLDT;						//左端点停留时间，单位s
		double dRDT;						//右端点停留时间，单位s
		double dPitch;						//仰角，绕X轴旋转
		double dOri;						//方位角，绕Z轴旋转
		double dVar1;						//预留参数1
		double dVar2;						//预留参数2
		double dVar3;						//预留参数3
		double dLoc1[6];
		double dLoc2[6];
	}HS_WeavePara;

    //点位的定义
    typedef struct HS_POSINFO
    {
        double dPos[MaxAxisNum];			//点位数据，关节点位对应0~6,7~9为附加轴；空间点位为X/Y/Z/A/B/C,7~9为附加轴
        int iPose;							//位姿--每一位对应一个姿态
        Hs_Coordinate hs_coordinate;		//该点对应的坐标系
    }PosInfo;

    //多转控制参数
    typedef struct _HS_Revolve
    {
        int iTurn;
    } HS_Revolve;

	struct BaseMoveData
	{
		HS_MOVETYPE eTrajType;				// 轨迹类型
		Hs_Coordinate sCurCoordinate;		// 该指令当前坐标系
		PosInfo sStartPos;					// 开始点位
		PosInfo sMidPos;					// 中间点位
		PosInfo sEndPos;					// 结束点位
		bool b2mid;							// 是否需要到中间点，true --到中间点   false----到结束点
		double dVel;						// 直线/关节速度
		double dVort;						// 旋转速度
		double dAcc;						// 加速度
		double dDec;						// 减速度
		int iCntType;						// 平滑类型
		HS_Revolve tRevolve;				// 多转控制参数
		bool bCoorperMove;					// 变位机运动
		HS_WeavePara tWeavePara;			// 摆焊运动
	};

	//自动运行参数【组】
	struct GroupMotionData
	{
		int iLineNum;						// 逻辑行号
		BaseMoveData tBaseMoveData[MAXGROUPNUM];
		HS_GroupRel tHS_GroupRel;
		double dCnt;						// 平滑系数百分比
		double dCR;							// 平滑系数绝对量
		int iSmooth;						// 柔顺等级参数
		bool bExtTool;						// 外部工具
		bool bStartMove;					// 当前段是否为启动段【第一段或者运动到点】，启动段起点为关节，不依赖之前运动
		bool bWristQYFlag;					// 腕部过奇异功能
		FilterControl tFilterControl;		// 前置滤波控制
	};

	struct GroupTrajData
	{
		GroupMotionData tMotionData;			//缓存运动信息
		unsigned char iData[MAXGROUPNUM][4000];
	};

    // 插补信息
    struct IntData
    {
		HS_GroupJPos tGJPos[INTBUFF];
        double dPercent;                      // 插补百分比
		double dDisStart;					  // 距离插补起点的距离
		double dDisEnd;						  // 距离插补结束点的距离
    };

    // 随动参数
    struct FollowData
    {
        int iData;       
    };

    // 传送带跟踪参数
    struct TrackData
    {
        int iData;       
    };
}
}