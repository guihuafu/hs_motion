
#ifndef _MATHOPERATION_H
#define _MATHOPERATION_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "HS3_MotionAlertDef.h"

#define PI			3.1415926535897931
const double pi = 3.141592653589793;
const double deg2rad = pi / 180;
const double rad2deg = 180 / pi;
#define angle2Rad(angle) ((angle) / 180.0 *PI)
#define Rad2angle(rad) ((rad) * 180.0 /PI)
#define Max(a,b)    (((a) > (b)) ? (a) : (b))
#define Min(a,b)    (((a) > (b)) ? (b) : (a))

//笛卡尔空间轴
enum
{
	PX = 0,
	PY = 1,
	PZ = 2,
	PA = 3,
	PB = 4,
	PC = 5,
};

#define Eps 1e-6

#define AT_Left			(unsigned char)(0<<0)		//左手
#define AT_Right		(unsigned char)(1<<0)		//右手	
#define AT_Front		(unsigned char)(0<<1)		//正手
#define AT_Back		    (unsigned char)(1<<1)		//反手	
#define AT_Above		(unsigned char)(0<<2)		//蜷起状态，3轴大于奇异位置
#define AT_Below		(unsigned char)(1<<2)		//伸直状态，3轴小于奇异位置	
#define AT_NonFlip		(unsigned char)(0<<3)		//腕部形态，5轴为正，向下
#define AT_Flip			(unsigned char)(1<<3)		//腕部形态，5轴为负，向上


#endif