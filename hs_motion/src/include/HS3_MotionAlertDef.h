/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS3_MotionAlertDef.h
* 摘    要：华数III型运动警报定义

* 当前版本：1.0
* 作    者：hj
* 完成日期：2017-7-24
*			
*/
#ifndef _HS3_MOTIONALERTADM_DEF_H_
#define _HS3_MOTIONALERTADM_DEF_H_



enum ERRORTYPE
{
	ERROR_PMAX		=500,				//超限位，正向
	ERROR_PMIN,							//超限位，负向
	ERROR_AHEAD_AREA,					//超区域空间限制,报错停机
	WARNING_AHEAD_AREA,					//超区域空间限制,报警停机
	ERROR_REAL_CRTXE,					//电流超限制
	ERROR_UNREACHABLE,					//目标点超限位,不可达
	ERROR_CIRCLE_CENTER,				//圆弧 无法规划圆心
	ERROR_CIRCLE_POINT,					//点位无法规划圆弧
	ERROR_TRACK_JOINTEPOS,				//传送带跟踪的点位不能是关节坐标
	ERROR_JOCAB_INV,					//机型DH参数异常
	ERROR_QY_BORDER,					//边界奇异
	ERROR_QY_INSIDE,					//内部奇异
	ERROR_QY_WRIST,						//腕部奇异
	ERROR_CPOSTOJPOS,					//空间点位超出机器人可达范围
	WARNING_CIRCLE_REPEAT,				//圆弧点位重复【距离太近】
	ERROR_CIRCLE_EulerOver,				//圆弧运动，前后段的姿态位移比相差过大
	ERROR_EndJPointQY,					//空间运动，目标点为关节，通过奇异区间不可达
	ERROR_Hand_OverVel,					//空间点动靠近奇异区间，降低点动速度或者改为关节点位
	ERROR_MD410_UNION_PMAX,				//MD41系列机型超联合限位，正向
	ERROR_MD410_UNION_PMIN,				//MD41系列机型超联合限位，负向
	ERROR_LINK3_UNREACH,				//LINK3机型【BR326】超出运动范围
	ERROR_SCARA_QYC2J,					//Scara机型，点位太靠近奇异点，导致求解失败
	ERROR_CIRCLE_QYPOINT,				//圆弧运动点位异常，规划路径超出可达空间或靠近奇异
	ERROR_NOPREHANDLED, 				//点位是否预处理
	WARING_REPEAT,  					//重复点位
	ERROR_RESTART_EJPOS,  				//暂停重启启动关节位置与停止位置不一致
	ERROR_VMAX,							//超速度
	ERROR_KINEMATIC,					//插补规划异常
	ERROR_CIRCLE_RESTARTRZERO,			//圆弧暂停重启得到的圆弧半径为0
	ERROR_AUTO_ENDPOS,					//插补异常，停止点位不匹配
	ERROR_STARTPOS,						//点位给定异常，不连续
	ERROR_MOVE_RESTARTEPOS,				//暂停重启时上层点位给定错误，点位不连续
	ERROR_MOVE_MOVETYPE,				//运动类型设定错误
	ERROR_COORDNUM,						//工具工件坐标系设置错误
	ERROR_C2JACCURACY,					//空间运动插补异常【当前运动不可达】
	ERROR_ARC_MOVEBACK,					//圆弧回退时，缓冲区数据与给定点位冲突，无法规划
	ERROR_REAPEAD,						//空间运动的重复点位，但附加轴有位移，转换为关节运动处理
	ERROR_C2J,       					//逆解失败
	
};



//新报警码
#define ERRORNUM_ADD(Base,num)             (Base + num)  


#define Common							0
#define E_MOVE_MOVETYPE					ERRORNUM_ADD(Common,0)					 //运动类型设置错误             
#define E_C2J_PRECISION					ERRORNUM_ADD(Common,1)					 //空间点位转换关节点位位置精度不足
#define E_C2J_ATTITUDE					ERRORNUM_ADD(Common,2)					 //空间点位转换关节点位位置姿态不足
#define E_SETPOS_OVERACC				ERRORNUM_ADD(Common,3)					 //点位下发异常
#define E_SETPOS_OVERVEL				ERRORNUM_ADD(Common,4)					 //点位下发超速异常

#define Joint							100
#define E_J_TARGETOVERLIMIT             ERRORNUM_ADD(Joint,0)                     //关节运动目标点超限位
#define E_J_TARGETUNREABLE              ERRORNUM_ADD(Joint,1)                     //关节运动目标点【空间位置】超出可达范围
#define E_J_ATTUNREABLE_Scara           ERRORNUM_ADD(Joint,2)                     //Scara关节运动目标点【空间位置】的姿态给定异常，超出可达范围
#define E_J_STARTPOS					ERRORNUM_ADD(Joint,3)					  //起点点位不匹配

#define Line							200                                      
#define E_L_MOVEOVERJVEL                ERRORNUM_ADD(Line,0)                      //直线运动过程中速度超出 
#define E_L_MOVEUNRABLE                 ERRORNUM_ADD(Line,1)                      //直线运动过程中存在位置不可达
#define E_L_TARGETOVERLIMIT             ERRORNUM_ADD(Line,2)                      //直线运动目标点超限位
#define E_L_TARGETUNREABLE              ERRORNUM_ADD(Line,3)                      //直线运动目标点【空间位置】超出可达范围
#define E_L_STARTQYBORDER               ERRORNUM_ADD(Line,4)                      //直线运动起点处于边界奇异范围内，不可运动
#define E_L_ENDQYBORDER                 ERRORNUM_ADD(Line,5)                      //直线运动目标点处于边界奇异范围内，不可运动
#define E_L_STARTQYINSIDE               ERRORNUM_ADD(Line,6)                      //直线运动起点处于内部奇异范围内，不可运动
#define E_L_ENDQYINSIDE                 ERRORNUM_ADD(Line,7)                      //直线运动目标点处于内部奇异范围内，不可运动
#define E_L_STARTQYWRIST                ERRORNUM_ADD(Line,8)                      //直线运动起点处于腕部奇异范围内，不可运动
#define E_L_ENDQYWRIST                  ERRORNUM_ADD(Line,9)                      //直线运动目标点处于腕部奇异范围内，不可运动
#define E_L_MOVEQYBORDER                ERRORNUM_ADD(Line,10)                     //直线运动过程中靠近边界奇异导致运动异常
#define E_L_MOVEQYINSIDE                ERRORNUM_ADD(Line,11)                     //直线运动过程中靠近内部奇异导致运动异常 
#define E_L_MOVEQYWRIST                 ERRORNUM_ADD(Line,12)                     //直线运动过程中靠近腕部奇异导致运动异常
#define E_L_QYPASSWRIST                 ERRORNUM_ADD(Line,13)                     //直线运动目标点关节位置通过腕部奇异空间，不可达
#define E_L_QYPASSBORDER                ERRORNUM_ADD(Line,14)                     //直线运动目标点关节位置通过边界奇异空间，不可达
#define E_L_ATTUNREABLE_Scara	        ERRORNUM_ADD(Line,15)                     //Scara直线运动目标点【空间位置】的姿态给定异常，超出可达范围
#define E_L_STARTPOS					ERRORNUM_ADD(Line,16)					  //起点点位不匹配

#define Circle							300
#define E_C_MOVEUNRABLE                 ERRORNUM_ADD(Circle,0)                    //圆弧运动过程中存在位置不可达
#define E_C_TARGETOVERLIMIT             ERRORNUM_ADD(Circle,1)                    //圆弧运动目标点超限位
#define E_C_TARGETUNREABLE              ERRORNUM_ADD(Circle,2)                    //圆弧运动目标点【空间位置】超出可达范围
#define E_C_STARTQYBORDER               ERRORNUM_ADD(Circle,3)                    //圆弧运动起点处于边界奇异范围内，不可运动
#define E_C_ENDQYBORDER                 ERRORNUM_ADD(Circle,4)                    //圆弧运动目标点处于边界奇异范围内，不可运动
#define E_C_STARTQYINSIDE               ERRORNUM_ADD(Circle,5)                    //圆弧运动起点处于内部奇异范围内，不可运动
#define E_C_ENDQYINSIDE                 ERRORNUM_ADD(Circle,6)                    //圆弧运动目标点处于内部奇异范围内，不可运动
#define E_C_STARTQYWRIST                ERRORNUM_ADD(Circle,7)                    //圆弧运动起点处于腕部奇异范围内，不可运动
#define E_C_ENDQYWRIST                  ERRORNUM_ADD(Circle,8)                    //圆弧运动目标点处于腕部奇异范围内，不可运动
#define E_C_MOVEQYBORDER                ERRORNUM_ADD(Circle,9)                    //圆弧运动过程中靠近边界奇异导致运动异常
#define E_C_MOVEQYINSIDE                ERRORNUM_ADD(Circle,10)                   //圆弧运动过程中靠近内部奇异导致运动异常 
#define E_C_MOVEQYWRIST                 ERRORNUM_ADD(Circle,11)                   //圆弧运动过程中靠近腕部奇异导致运动异常
#define E_C_QYPASSWRIST                 ERRORNUM_ADD(Circle,12)                   //圆弧运动目标点关节位置通过腕部奇异空间，不可达
#define E_C_QYPASSBORDER                ERRORNUM_ADD(Circle,13)                   //圆弧运动目标点关节位置通过边界奇异空间，不可达
#define E_C_MOVEOVERJVEL                ERRORNUM_ADD(Circle,14)                   //圆弧运动过程中速度超出 
#define E_C_ERRORPOINT  				ERRORNUM_ADD(Circle,15)	                  //圆弧运动无法规划，点位重复或者共线

#define Hand							400
#define E_H_COORPERHAND  				ERRORNUM_ADD(Hand,0)	                  //变位机不支持空间点动
#define E_H_PARAACC  					ERRORNUM_ADD(Hand,1)					  //点动加速度参数异常
#define E_H_PARKVEL 					ERRORNUM_ADD(Hand,2)					  //错误的速度倍率

#define Waring                          1000
#define W_REPEAT                        ERRORNUM_ADD(Waring,0)                    //重复点位

#endif
