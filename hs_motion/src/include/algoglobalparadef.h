#pragma once

namespace hsc3
{
namespace algo
{

#define  MaxAxisNum     9              //单轴组轴最大数量
#define  MAXCOORDNUM	16		       //工具工件最大数量
#define  MAXGROUPNUM	4			   //最大轴组个数
#define  MAXCOORDSIZE   20			   //最大变位机个数

//机器人模型
enum HS_RobotType
{
	HSROB_NONE =0,
	HSROB_PUMA,		                    //PUMA
	HSROB_SCARA,	                    //SCARA
	HSROB_DELTA,	                    //DELTA
	HSROB_PUMA_5,       
	HSROB_NONSTANDARD_4,	            //4轴非标设备
	HSROB_LINK_3,			            //3连杆结构
	HSROB_MD410,			            //MD410机型
	HSROB_POLISH,       
	HSROB_SCARA_3,	                    //SCARA3
	HSROB_COORPER,						//变位机	
	HSROB_OTHER
};

//机器人类型子类型
enum HS_RobotType_sub
{
	HSROB_SUB_NONE =0,
	MD_410,
	MD_4110,
	MD_4130,
	JR6300,
	Cobot6,									//协作系列机器人
	HSROB_SUB_OTHER
};

//坐标系定义
enum COORD_SYSTEM
{
	COORD_SYSTEM_MIN = -1,					//最小值
	JOINT_COORD_SYSTEM =0,					//关节坐标
	BASE_COORD_SYSTEM ,						//基坐标
	USER_COORD_SYSTEM,						//工作坐标  工件号:默认 -1 ，0-15User
	TOOL_COORD_SYSTEM,						//工具坐标系 工具号:默认 -1 , 0-15
	WORLD_COORD_SYSTEM,						//世界坐标系
	COORD_SYSTEM_MAX,						//最大值
};

//空间位置的类型
enum CPType
{
	CP_Flange	= 0,						 //法兰在基础坐标系中的空间位置
	CP_ToolWork ,							 //工具点在工作坐标系中的空间位置
	CP_ToolCoord,							 //工具点在变位机坐标系中的空间位置
	CP_WorkTool,							 //工件点在工具坐标系中的空间位置
	CP_EterToolWork,						 //外部工件相对于外部工具的空间位置
	CP_EterBaseSpin,
	CP_ExtCoorper,							 //外部导轨坐标系运动
	CP_ExtCoorper2Base,						 //导轨坐标系转化为基坐标系
};

//轴组关系类型
enum GroupRelType
{
	GRT_NoUse = 0,							//不使用
	GRT_Independent,						//独立组
	GRT_Master,								//主动轴组
	GRT_Slave,								//从动轴组【1级】
	GRT_Slave2,								//从动轴组【2级】
};

//轴组关系映射表
struct HS_GroupRel
{
	GroupRelType eGroupRelType[MAXGROUPNUM];
};

//轴组综合点位【关节】
struct HS_GroupJPos
{
	double dJPos[MAXGROUPNUM][MaxAxisNum];
};

//坐标系
typedef struct HS_Coordinate
{
	COORD_SYSTEM iCoordinate;			//坐标系
	int iToolNum;						//工具号
	int iWorkNum;						//工件号
	bool bExtTool;						//是否外部工具
	bool bExtCoorper;					//外部协同，地轨协同运动
}Hs_Coordinate;

//变位机或者地轨坐标系
struct HS_Coord
{
	int iAxisId[3];								//对应的轴号
	double dExtCoord[3][6];						//外部坐标系--地轨或变位机
};

//模型参数
struct GroupModelPara
{
	HS_RobotType eRobtype;              //机器人类型
    HS_RobotType_sub eRobtype_sub;      //机器人模型，子类
    double DHPara[6][4];                //DH连杆参数矩阵
};

//限位参数
struct LimitPara
{
	double dPmax[MaxAxisNum];           //正限位
    double dPmin[MaxAxisNum];           //负限位
    bool bOpen[MaxAxisNum];             //限位开关
    double dQYPara_Inter;               //内部奇异限制
    double dQYPara_Border;              //边界奇异限制
    double dQYPara_Wrist;               //腕部奇异限制
};

//轴速度参数
struct AxisVelocityPara
{
	double dVmax[MaxAxisNum];           //最大速度
    double dVcruise[MaxAxisNum];        //运行速度
    double dAccelerate[MaxAxisNum];     //运行加速度
    double dJerkrat[MaxAxisNum];        //运行跃度
};  

//空间速度参数
struct GroupVelocityPara
{
	double dVtran;                      //TCP默认移动速度
    double dVrot;                       //TCP默认旋转速度
    double dVtranacc;                   //TCP默认移动加速度
    double dVrotacc;                    //TCP默认旋转加速度
    double dJerkrat;                    //TCP跃度

	double dTFreMin;					//频率时间保护最小值
	double dTFreMax;					//频率时间保护最大值
};  

//轴组综合参数
struct GroupStaticPara
{
	GroupModelPara tGroupModelPara;
	LimitPara tLimitPara;
	AxisVelocityPara tAxisVelocityPara;
	GroupVelocityPara tGroupVelocityPara;
	double dToolCoord[MAXCOORDNUM][6];			//工具坐标系：0~16
	double dWorkCoord[MAXCOORDNUM][6];			//工件坐标系：0~16
	double dWorldCoord[6];						//世界坐标系
	HS_Coord dFlTrackCoord;						//外部地轨坐标系
	HS_Coord dPoCoord;							//变位机坐标系			
};

//整形滤波控制变量
enum FilterType
{
    TYPE_IIR = 0,
    TYPE_FIR,
    TYPE_SHAPE,
};
//整形滤波类型区分
struct FilterPara
{
    FilterType eFilterType;
    double dFre;
    int iGrade;
    int iAxisNum;
};
//整形滤波控制
struct FilterControl
{
    bool bFilterOpenFlag;
    FilterPara tFilterPara;
};

enum HS_MStatus
{
	M_Error		= 0,			//错误
	M_UnInit	= 1,			//等待参数信息
	M_Busy		= 2,			//运行中
	M_Done		= 3,			//完成
	M_StopDone  = 4,			//停止完成
	M_StopDone_F  = 5,			//小线段停止完成当前段运动
};

}
}

