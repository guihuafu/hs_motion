/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Calibration.cpp
* 摘    要：标定计算

* 当前版本：1.0
* 作    者：cyh
* 完成日期：2017-12-11
*			
*/
#include "HS_Calibration.h"

#include <iostream>
#include <algorithm> //算法头文件，提供迭代器
#include <cassert>
using namespace std;
/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Calibration::HS_Calibration(int iGroupNum)
{
	m_HS_Kinematic = new HS_Kinematics(iGroupNum);	
}

HS_Calibration::~HS_Calibration(void)
{

}
/************************************************
函数功能：工具坐标系六点标定位姿矩阵
参    数：dError-------误差限定（大于0有效）
		 dPos_In1~4---同TCP位置的不同法兰盘位置
		 dPos_In5-----沿工具Z轴运行一段的位置点
		 dPos_In6-----ZX平面上的一个位置点
		 dToolOut-----输出位姿变换矩阵[6]
		 dMaxError----实际标定的最大误差值
返 回 值：错误ID
*************************************************/
int HS_Calibration::Tool_Six(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
	double *dPosIn4,double *dPosIn5,double *dPosIn6,double *dToolOut,double &dMaxError)
{
    double dToolPos[6] = {0};
    //四点求位置变换
    //if(Tool_Four(dError,dPosIn1,dPosIn2,dPosIn3,dPosIn4,dToolPos,dMaxError) != 0)
    //    return -1;

	LOG_ALGO("Tool_Four---PosIn5:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn5[0],dPosIn5[1],dPosIn5[2],dPosIn5[3],dPosIn5[4],dPosIn5[5]);
	LOG_ALGO("Tool_Four---PosIn6:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn6[0],dPosIn6[1],dPosIn6[2],dPosIn6[3],dPosIn6[4],dPosIn6[5]);

	Tool_Four(dError,dPosIn1,dPosIn2,dPosIn3,dPosIn4,dToolPos,dMaxError);
		
    //求齐次矩阵
    double dPOrg[4][4],dPZ[4][4],dPX[4][4];   

    m_HS_Kinematic->HS_CPosToMPos(dPosIn4,dPOrg);
    m_HS_Kinematic->HS_CPosToMPos(dPosIn5,dPZ);
    m_HS_Kinematic->HS_CPosToMPos(dPosIn6,dPX);

    //姿态矩阵的逆
    double dPOrgA[3][3] = {0};
    for(int i = 0;i < 3;i++)
        for(int j = 0;j < 3;j++)
        {
            dPOrgA[i][j] = dPOrg[i][j];
        }
    double dRnOrg[3][3] = {0};
    Matrix_Inverse(3,&dPOrgA[0][0],&dRnOrg[0][0]);

	//计算出Z轴向量与X轴向量【修改方向】
	//方向再次改回【焊接需求241024--建威】
	double dVZ[3],dVX[3];
	dVZ[0] = dPZ[0][3] - dPOrg[0][3];
	dVZ[1] = dPZ[1][3] - dPOrg[1][3];
	dVZ[2] = dPZ[2][3] - dPOrg[2][3];
	//dVZ[0] = dPOrg[0][3] - dPZ[0][3];
	//dVZ[1] = dPOrg[1][3] - dPZ[1][3];
	//dVZ[2] = dPOrg[2][3] - dPZ[2][3];

    dVX[0] = dPX[0][3] - dPZ[0][3];
    dVX[1] = dPX[1][3] - dPZ[1][3];
    dVX[2] = dPX[2][3] - dPZ[2][3];

    double dModZ = sqrt(dVZ[0]*dVZ[0] + dVZ[1]*dVZ[1] + dVZ[2]*dVZ[2]);
    if(dModZ < Eps)
        return -2;

    double dModX = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);
    if(dModX < Eps)
        return -3;

    //求解出A/N向量
    double dTA[3],dTN[3];

    dTA[0] = dVZ[0]/dModZ; dTA[1] = dVZ[1]/dModZ; dTA[2] = dVZ[2]/dModZ;

    dTN[0] = dVX[0]/dModX; dTN[1] = dVX[1]/dModX; dTN[2] = dVX[2]/dModX;

    double dTO[3];
    Matrix_VecCross(dTA,dTN,dTO);

    Matrix_VecCross(dTO,dTA,dTN);

    double dMIn[3][3] = {0};
    dMIn[0][0] = dTN[0];dMIn[1][0] = dTN[1];dMIn[2][0] = dTN[2];
    dMIn[0][1] = dTO[0];dMIn[1][1] = dTO[1];dMIn[2][1] = dTO[2];
    dMIn[0][2] = dTA[0];dMIn[1][2] = dTA[1];dMIn[2][2] = dTA[2];

    double dMOut[3][3] = {0};
    Matrix_Multi(3,3,3,&dRnOrg[0][0],&dMIn[0][0],&dMOut[0][0]);
    //输出
    double dMPos[4][4] = {0};
    for(int i = 0;i < 3;i++)
        for(int j = 0;j < 3;j++)
        {
            dMPos[i][j] = dMOut[i][j];
        }

    dMPos[0][3] = dToolPos[0];
    dMPos[1][3] = dToolPos[1];
    dMPos[2][3] = dToolPos[2];
    dMPos[3][3] = 1;

    m_HS_Kinematic->HS_MPosToCPos(dMPos,dToolOut);
	return 0;
}
/************************************************
函数功能：工具坐标系四点标定位置变换矩阵
参    数：dError-------误差限定（大于0有效）
		 dPos_In1~4---输入四点坐标（同TCP位置的四个基坐标位置）
		 dToolOut-----输出位置变换矩阵[6]
		 dMaxError----实际标定的最大误差值
返 回 值：错误ID
*************************************************/
int HS_Calibration::Tool_Four(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
	double *dPosIn4,double *dToolOut,double &dMaxError)
{	

	LOG_ALGO("Tool_Four---PosIn1:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn1[0],dPosIn1[1],dPosIn1[2],dPosIn1[3],dPosIn1[4],dPosIn1[5]);
	LOG_ALGO("Tool_Four---PosIn2:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn2[0],dPosIn2[1],dPosIn2[2],dPosIn2[3],dPosIn2[4],dPosIn2[5]);
	LOG_ALGO("Tool_Four---PosIn3:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn3[0],dPosIn3[1],dPosIn3[2],dPosIn3[3],dPosIn3[4],dPosIn3[5]);
	LOG_ALGO("Tool_Four---PosIn4:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
		dPosIn4[0],dPosIn4[1],dPosIn4[2],dPosIn4[3],dPosIn4[4],dPosIn4[5]);

	dMaxError = 0;

	double d123[3],d124[3],d134[3],d234[3];
	Tool_Mid(false,dPosIn1,dPosIn2,dPosIn3,dPosIn4,d123);
	Tool_Mid(false,dPosIn1,dPosIn2,dPosIn4,dPosIn4,d124);
	Tool_Mid(false,dPosIn1,dPosIn3,dPosIn4,dPosIn4,d134);
	Tool_Mid(false,dPosIn2,dPosIn3,dPosIn4,dPosIn4,d234);

	//求相互之间的误差
	double dMidError[6];
	dMidError[0] = sqrt((d123[0] - d124[0])*(d123[0] - d124[0]) + (d123[1] - d124[1])*(d123[1] - d124[1]) + (d123[2] - d124[2])*(d123[2] - d124[2]));
	dMidError[1] = sqrt((d123[0] - d134[0])*(d123[0] - d134[0]) + (d123[1] - d134[1])*(d123[1] - d134[1]) + (d123[2] - d134[2])*(d123[2] - d134[2]));
	dMidError[2] = sqrt((d123[0] - d234[0])*(d123[0] - d234[0]) + (d123[1] - d234[1])*(d123[1] - d234[1]) + (d123[2] - d234[2])*(d123[2] - d234[2]));
	dMidError[3] = sqrt((d124[0] - d134[0])*(d124[0] - d134[0]) + (d124[1] - d134[1])*(d124[1] - d134[1]) + (d124[2] - d134[2])*(d124[2] - d134[2]));
	dMidError[4] = sqrt((d124[0] - d234[0])*(d124[0] - d234[0]) + (d124[1] - d234[1])*(d124[1] - d234[1]) + (d124[2] - d234[2])*(d124[2] - d234[2]));
	dMidError[5] = sqrt((d134[0] - d234[0])*(d134[0] - d234[0]) + (d134[1] - d234[1])*(d134[1] - d234[1]) + (d134[2] - d234[2])*(d134[2] - d234[2]));
	for(int i = 0;i < 6;i++)
	{
		if(dMidError[i] > dError)
        {
//            LOG_ALGO("Error:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Limit:%.3lf",
//                dMidError[0],dMidError[1],dMidError[2],dMidError[3],dMidError[4],dMidError[5],dError);
			//return -1;
			dMaxError = dMidError[i];
        }
	}
	
	Tool_Mid(true,dPosIn1,dPosIn2,dPosIn3,dPosIn4,dToolOut);
	dToolOut[3] = 0;dToolOut[4] = 0;dToolOut[5] = 0;

	LOG_ALGO("Tool_Four---Error:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Limit:%.3lf;MaxError:%.3lf",
		dMidError[0],dMidError[1],dMidError[2],dMidError[3],dMidError[4],dMidError[5],dError,dMaxError);

	if(dMaxError > dError)
	{
		return -1;
	}
	return 0;
}
/************************************************
函数功能：工具坐标系计算中间变量
参    数：bFour--------是否使用四点，ture是四点
		 dPos_In1~4---输入四点坐标（同TCP位置的四个基坐标位置）
		 dToolOut-----输出位置变换矩阵
返 回 值：错误ID
*************************************************/
int HS_Calibration::Tool_Mid(bool bFour,double *dPosIn1,double *dPosIn2,double *dPosIn3,
	double *dPosIn4,double *dToolOut)
{
	
	//齐次矩阵
	double dMPos1[4][4],dMPos2[4][4],dMPos3[4][4],dMPos4[4][4];
	m_HS_Kinematic->HS_CPosToMPos(dPosIn1,dMPos1);
	m_HS_Kinematic->HS_CPosToMPos(dPosIn2,dMPos2);
	m_HS_Kinematic->HS_CPosToMPos(dPosIn3,dMPos3);
	m_HS_Kinematic->HS_CPosToMPos(dPosIn4,dMPos4);
	
	//位置矩阵
	double dPos1[3],dPos2[3],dPos3[3],dPos4[3];
	dPos1[0] = dMPos1[0][3];dPos1[1] = dMPos1[1][3];dPos1[2] = dMPos1[2][3];
	dPos2[0] = dMPos2[0][3];dPos2[1] = dMPos2[1][3];dPos2[2] = dMPos2[2][3];
	dPos3[0] = dMPos3[0][3];dPos3[1] = dMPos3[1][3];dPos3[2] = dMPos3[2][3];
	dPos4[0] = dMPos4[0][3];dPos4[1] = dMPos4[1][3];dPos4[2] = dMPos4[2][3];
	
	double dMPos1A[3][3],dMPos2A[3][3],dMPos3A[3][3],dMPos4A[3][3];
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			dMPos1A[i][j] = dMPos1[i][j];
			dMPos2A[i][j] = dMPos2[i][j];
			dMPos3A[i][j] = dMPos3[i][j];
			dMPos4A[i][j] = dMPos4[i][j];
		}
	//姿态逆矩阵
	double dMAtInv1[3][3],dMAtInv2[3][3],dMAtInv3[3][3],dMAtInv4[3][3];
	Matrix_Inverse(3,&dMPos1A[0][0],&dMAtInv1[0][0]);
	Matrix_Inverse(3,&dMPos2A[0][0],&dMAtInv2[0][0]);
	Matrix_Inverse(3,&dMPos3A[0][0],&dMAtInv3[0][0]);
	Matrix_Inverse(3,&dMPos4A[0][0],&dMAtInv4[0][0]);
	
	//姿态逆矩阵的差值
	double dMAtInv12[3][3],dMAtInv13[3][3],dMAtInv14[3][3];
	Matrix_AddOrSub(3,3,1,-1,&dMAtInv1[0][0],&dMAtInv2[0][0],&dMAtInv12[0][0]);
	Matrix_AddOrSub(3,3,1,-1,&dMAtInv1[0][0],&dMAtInv3[0][0],&dMAtInv13[0][0]);
	Matrix_AddOrSub(3,3,1,-1,&dMAtInv1[0][0],&dMAtInv4[0][0],&dMAtInv14[0][0]);
	
	//计算Rn*E
	double dPosAt1[3],dPosAt2[3],dPosAt3[3],dPosAt4[3];
	Matrix_Multi(3,3,1,&dMAtInv1[0][0],dPos1,dPosAt1);
	Matrix_Multi(3,3,1,&dMAtInv2[0][0],dPos2,dPosAt2);
	Matrix_Multi(3,3,1,&dMAtInv3[0][0],dPos3,dPosAt3);
	Matrix_Multi(3,3,1,&dMAtInv4[0][0],dPos4,dPosAt4);
	
	//Rn1*E1-Rnx*Ex
	double dPosAt12[3],dPosAt13[3],dPosAt14[3];
	Matrix_AddOrSub(3,1,1,-1,dPosAt1,dPosAt2,dPosAt12);
	Matrix_AddOrSub(3,1,1,-1,dPosAt1,dPosAt3,dPosAt13);
	Matrix_AddOrSub(3,1,1,-1,dPosAt1,dPosAt4,dPosAt14);
	
	if(bFour == false)	//计算三个点
	{
		double dRn[6][3];
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				dRn[i][j] = dMAtInv12[i][j];	
				dRn[i+3][j] = dMAtInv13[i][j];
			}
		}
		double dEn[6];
		for(int i = 0;i < 3;i++)
		{
			dEn[i] = dPosAt12[i];
			dEn[i+3] = dPosAt13[i];
		}

		double dRnT[3][6];
		Matrix_Transpose(6,3,&dRn[0][0],&dRnT[0][0]);

		double dRnn[3][3];
		Matrix_Multi(3,6,3,&dRnT[0][0],&dRn[0][0],&dRnn[0][0]);
		double dEnn[3];
		Matrix_Multi(3,6,1,&dRnT[0][0],dEn,dEnn);
		
		double dRnnInv[3][3];
		Matrix_Inverse(3,&dRnn[0][0],&dRnnInv[0][0]);

		//得到输出
		Matrix_Multi(3,3,1,&dRnnInv[0][0],dEnn,dToolOut);
	}
	else	//计算四个点
	{
		double dRn[9][3];
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				dRn[i][j] = dMAtInv12[i][j];	
				dRn[i+3][j] = dMAtInv13[i][j];
				dRn[i+6][j] = dMAtInv14[i][j];
			}
		}
		double dEn[9];
		for(int i = 0;i < 3;i++)
		{
			dEn[i] = dPosAt12[i];
			dEn[i+3] = dPosAt13[i];
			dEn[i+6] = dPosAt14[i];
		}

		double dRnT[3][9];
		Matrix_Transpose(9,3,&dRn[0][0],&dRnT[0][0]);

		double dRnn[3][3];
		Matrix_Multi(3,9,3,&dRnT[0][0],&dRn[0][0],&dRnn[0][0]);
		double dEnn[3];
		Matrix_Multi(3,9,1,&dRnT[0][0],dEn,dEnn);

		double dRnnInv[3][3];
		Matrix_Inverse(3,&dRnn[0][0],&dRnnInv[0][0]);

		double dQ[3];
		Matrix_Multi(3,3,1,&dRnnInv[0][0],dEnn,dQ);

		//4个点的姿态矩阵结合
		double dRr[12][3];
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				dRr[i][j] = dMPos1[i][j];
				dRr[i+3][j] = dMPos2[i][j];
				dRr[i+6][j] = dMPos3[i][j];
				dRr[i+9][j] = dMPos4[i][j];
			}
		}
		//计算中间量1
		double dQnE1[3],dQnE2[3],dQnE3[3],dQnE4[3];
		Matrix_AddOrSub(3,1,1,-1,dQ,dPos1,dQnE1);
		Matrix_AddOrSub(3,1,1,-1,dQ,dPos2,dQnE2);
		Matrix_AddOrSub(3,1,1,-1,dQ,dPos3,dQnE3);
		Matrix_AddOrSub(3,1,1,-1,dQ,dPos4,dQnE4);

		//结合矩阵
		double dEr[12];
		for(int i = 0;i < 3;i++)
		{
			dEr[i] = dQnE1[i];
			dEr[i+3] = dQnE2[i];
			dEr[i+6] = dQnE3[i];
			dEr[i+9] = dQnE4[i];
		}
		double dRrT[3][12];
		Matrix_Transpose(12,3,&dRr[0][0],&dRrT[0][0]);
		double dRrr[3][3];
		double dErr[3];
		Matrix_Multi(3,12,3,&dRrT[0][0],&dRr[0][0],&dRrr[0][0]);
		Matrix_Multi(3,12,1,&dRrT[0][0],dEr,dErr);
		double dRrrInv[3][3];
		Matrix_Inverse(3,&dRrr[0][0],&dRrrInv[0][0]);

		//得到输出
		Matrix_Multi(3,3,1,&dRrrInv[0][0],dErr,dToolOut);

	}
	
	return 0;
}

/************************************************
函数功能：工件坐标系标定
参    数：dPo------标定原点
	     dPx------工具坐标x轴上的点
		 dPxy-----工具坐标xy平面上的点
		 dToolOut-----输出位置变换矩阵[6]
返 回 值：错误ID
*************************************************/
int	HS_Calibration::Work_Three(double *dPo,double *dPx,double *dPxy,double *dWorkOut)
{
	//计算出Z轴向量与X轴向量
	double dVX[3],dVXY[3];
	dVX[0] = dPx[0] - dPo[0];
	dVX[1] = dPx[1] - dPo[1];
	dVX[2] = dPx[2] - dPo[2];

	dVXY[0] = dPxy[0] - dPo[0];
	dVXY[1] = dPxy[1] - dPo[1];
	dVXY[2] = dPxy[2] - dPo[2];

	/*double dModX = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);
	if(dModX < Eps)
		return -2;

	double dModXY = sqrt(dVXY[0]*dVXY[0] + dVXY[1]*dVXY[1] + dVXY[2]*dVXY[2]);
	if(dModXY < Eps)
		return -3;*/

	//求解出A/N向量
	double dTX[3],dTXY[3];
	/*dTX[0] = dVX[0]/dModX; dTX[1] = dVX[1]/dModX; dTX[2] = dVX[2]/dModX;
	dTXY[0] = dVXY[0]/dModXY; dTXY[1] = dVXY[1]/dModXY; dTXY[2] = dVXY[2]/dModXY;*/

	double dTZ[3],dTY[3];
	/*Matrix_VecCross(dTX,dTXY,dTZ);
	Matrix_VecCross(dTZ,dTX,dTY);*/

	double dVZ[3];
	Matrix_VecCross(dVX,dVXY,dVZ);

	double dModX = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);
	if(dModX < Eps)
		return -2;

	double dModZ = sqrt(dVZ[0]*dVZ[0] + dVZ[1]*dVZ[1] + dVZ[2]*dVZ[2]);
	if(dModZ < Eps)
		return -3;

	dTX[0] = dVX[0]/dModX; dTX[1] = dVX[1]/dModX; dTX[2] = dVX[2]/dModX;
	dTZ[0] = dVZ[0]/dModZ; dTZ[1] = dVZ[1]/dModZ; dTZ[2] = dVZ[2]/dModZ;

	Matrix_VecCross(dTZ,dTX,dTY);


	//输出
	double dMPos[4][4] = {0};
	dMPos[0][0] = dTX[0];dMPos[1][0] = dTX[1];dMPos[2][0] = dTX[2];dMPos[3][0] = 0;
	dMPos[0][1] = dTY[0];dMPos[1][1] = dTY[1];dMPos[2][1] = dTY[2];dMPos[3][1] = 0;
	dMPos[0][2] = dTZ[0];dMPos[1][2] = dTZ[1];dMPos[2][2] = dTZ[2];dMPos[3][2] = 0;

	dMPos[0][3] = dPo[0];dMPos[1][3] = dPo[1];dMPos[2][3] = dPo[2];dMPos[3][3] = 1;
	m_HS_Kinematic->HS_MPosToCPos(dMPos,dWorkOut);
	return 0;
}
/************************************************
函数功能：外部工件坐标系三点标定位姿矩阵
参    数：dPosIn1-----原点，外部TCP工具在法兰的位置（姿态不变取三点）
		 dPos_In2-----沿工件X轴运行一段的位置点
		 dPos_In3-----XY平面上的一个位置点
		 dToolIn------外部工具坐标系（实际存的是（工件）相对于基坐标的位置）
		 dWorkOut-----输出位姿变换矩阵[6]，外部工件坐标系（实际存的是（工具）相对于法兰的位置）
返 回 值：错误ID
*************************************************/
int HS_Calibration::Enternal_Work_Three(double *dPosIn1,double *dPosIn2,double *dPosIn3,double *dToolIn,double *dWorkOut)
{

	//1，先用无意义的W/T矩阵（dPosIn1）求得B/F的矩阵，A*B=C，B=A逆*C，A=C*B逆
	double df_TWMatrix[4][4]={0};
	double dMPosTemp[12] = {0};
	m_HS_Kinematic->HS_CPosToMPos(dPosIn1,df_TWMatrix);////////
	double df_TBMatrix[4][4]={0};
	double dMWork[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Matrix_Multi(4,4,4,&dMWork[0][0],&df_TWMatrix[0][0],&df_TBMatrix[0][0]);//////
	double d_FBMatrix[4][4]={0};////////
	double dToolInMatrix[4][4]={0};
	double dToolInRMatrix[4][4]={0};
	m_HS_Kinematic->HS_CPosToMPos(dToolIn,dToolInMatrix);
	Matrix_Inverse(4,&dToolInMatrix[0][0],&dToolInRMatrix[0][0]);
	Matrix_Multi(4,4,4,&df_TBMatrix[0][0],&dToolInRMatrix[0][0],&d_FBMatrix[0][0]);//////
	////验证，查看FB坐标系Z轴是否摆正
	double dFB_CPos[6]={0};
	m_HS_Kinematic->HS_MPosToCPos(d_FBMatrix,dFB_CPos);
	LOG_ALGO("dFB_CPos:X=%f,Y=%f,Z=%f,A=%f,B=%f,C=%f\n",
		                       dFB_CPos[PX],dFB_CPos[PY],dFB_CPos[PZ],dFB_CPos[PA],dFB_CPos[PB],dFB_CPos[PC]);
	//2，根据B/W和F/B就可以求出F/W也就是F/T工件坐标系（位置）
	double d_BFMatrix[4][4]={0};
	Matrix_Inverse(4,&d_FBMatrix[0][0],&d_BFMatrix[0][0]);
	double d_TFMatrix[4][4]={0};
	Matrix_Multi(4,4,4,&d_BFMatrix[0][0],&dToolInMatrix[0][0],&d_TFMatrix[0][0]);
	//3，用三点求工件坐标系（获取三点位置，姿态不能变）
	//用无意义的W/T矩阵（dPosIn2，dPosIn3）求得B/F2，B/F3的矩阵，再求得F2/T，F3/T，前面得到F/T的矩阵，三点位置已知，可以确定x,y,z轴
	//dPosIn2
	m_HS_Kinematic->HS_CPosToMPos(dPosIn2,df_TWMatrix);
	Matrix_Multi(4,4,4,&dMWork[0][0],&df_TWMatrix[0][0],&df_TBMatrix[0][0]);
	double d_F2BMatrix[4][4]={0};////////
	Matrix_Multi(4,4,4,&df_TBMatrix[0][0],&dToolInRMatrix[0][0],&d_F2BMatrix[0][0]);
	////
	double d_BF2Matrix[4][4]={0};
	Matrix_Inverse(4,&d_F2BMatrix[0][0],&d_BF2Matrix[0][0]);
	double d_TF2Matrix[4][4]={0};
	Matrix_Multi(4,4,4,&d_BF2Matrix[0][0],&dToolInMatrix[0][0],&d_TF2Matrix[0][0]);
	//dPosIn3
	m_HS_Kinematic->HS_CPosToMPos(dPosIn3,df_TWMatrix);
	Matrix_Multi(4,4,4,&dMWork[0][0],&df_TWMatrix[0][0],&df_TBMatrix[0][0]);
	double d_F3BMatrix[4][4]={0};////////
	Matrix_Multi(4,4,4,&df_TBMatrix[0][0],&dToolInRMatrix[0][0],&d_F3BMatrix[0][0]);
	////
	double d_BF3Matrix[4][4]={0};
	Matrix_Inverse(4,&d_F3BMatrix[0][0],&d_BF3Matrix[0][0]);
	double d_TF3Matrix[4][4]={0};
	Matrix_Multi(4,4,4,&d_BF3Matrix[0][0],&dToolInMatrix[0][0],&d_TF3Matrix[0][0]);
	//将d_TFMatrix，d_TF2Matrix，d_TF3Matrix转换为CPos模式，计算姿态
	double dTF_CPos[6]={0},dTF2_CPos[6]={0},dTF3_CPos[6]={0};
	m_HS_Kinematic->HS_MPosToCPos(d_TFMatrix,dTF_CPos);
	m_HS_Kinematic->HS_MPosToCPos(d_TF2Matrix,dTF2_CPos);
	m_HS_Kinematic->HS_MPosToCPos(d_TF3Matrix,dTF3_CPos);

	//计算出Z轴向量与X轴向量
	double dVX[3],dVXY[3];
	dVX[0] = dTF2_CPos[0] - dTF_CPos[0];
	dVX[1] = dTF2_CPos[1] - dTF_CPos[1];
	dVX[2] = dTF2_CPos[2] - dTF_CPos[2];

	dVXY[0] = dTF3_CPos[0] - dTF_CPos[0];
	dVXY[1] = dTF3_CPos[1] - dTF_CPos[1];
	dVXY[2] = dTF3_CPos[2] - dTF_CPos[2];


	//求解出A/N向量
	double dTX[3],dTXY[3];
	double dTZ[3],dTY[3];
	double dVZ[3];
	Matrix_VecCross(dVX,dVXY,dVZ);

	double dModX = sqrt(dVX[0]*dVX[0] + dVX[1]*dVX[1] + dVX[2]*dVX[2]);
	if(dModX < Eps)
		return -2;

	double dModZ = sqrt(dVZ[0]*dVZ[0] + dVZ[1]*dVZ[1] + dVZ[2]*dVZ[2]);
	if(dModZ < Eps)
		return -3;

	dTX[0] = dVX[0]/dModX; dTX[1] = dVX[1]/dModX; dTX[2] = dVX[2]/dModX;
	dTZ[0] = dVZ[0]/dModZ; dTZ[1] = dVZ[1]/dModZ; dTZ[2] = dVZ[2]/dModZ;

	Matrix_VecCross(dTZ,dTX,dTY);

	//输出
	double dMPos[4][4] = {0};

	dMPos[0][0] = dTX[0];dMPos[1][0] = dTX[1];dMPos[2][0] = dTX[2];dMPos[3][0] = 0;
	dMPos[0][1] = dTY[0];dMPos[1][1] = dTY[1];dMPos[2][1] = dTY[2];dMPos[3][1] = 0;
	dMPos[0][2] = dTZ[0];dMPos[1][2] = dTZ[1];dMPos[2][2] = dTZ[2];dMPos[3][2] = 0;

	dMPos[0][3] = d_TFMatrix[0][3];dMPos[1][3] = d_TFMatrix[1][3];dMPos[2][3] = d_TFMatrix[2][3];dMPos[3][3] = 1;
	m_HS_Kinematic->HS_MPosToCPos(dMPos,dWorkOut);
	LOG_ALGO("dWorkOut:X=%f,Y=%f,Z=%f,A=%f,B=%f,C=%f\n",
		                       dWorkOut[PX],dWorkOut[PY],dWorkOut[PZ],dWorkOut[PA],dWorkOut[PB],dWorkOut[PC]);
	return 0;

}
/************************************************
函数功能：Scara左右手标定，标定2轴
参    数：dLeftJPos------输入同一位置左手标定值（2轴）
	     dRightJPos-----输入同一位置右手标定值（2轴）
		 dZeroCorrect---输出对应的零点漂移补偿值（2轴）
						将当前零点值设为该值
返 回 值：错误ID
*************************************************/
int HS_Calibration::Scara_LeftRightZero(double dLeftJPos,double dRightJPos,double &dZeroCorrect)
{
	dZeroCorrect = (dLeftJPos + dRightJPos)/2;
	return 0;
}
/************************************************
函数功能：Scara两点标定算法
参    数：dPosIn1------输入位置1（空间X/Y/Z/A/B/C）
	     dPosIn2------输入位置2（空间X/Y/Z/A/B/C）
		 dWorkOut-----工具矩阵（X/Y)
返 回 值：错误ID
*************************************************/
int HS_Calibration::Scara_Tool_Two(double *dPosIn1,double *dPosIn2,double *dWorkOut)
{
	//齐次矩阵
	double dMPos1[4][4],dMPos2[4][4] = {0};
	m_HS_Kinematic->HS_CPosToMPos(dPosIn1,dMPos1);
	m_HS_Kinematic->HS_CPosToMPos(dPosIn2,dMPos2);

	//位置矩阵
	double dPos1[3],dPos2[3] = {0};
	dPos1[0] = dMPos1[0][3];dPos1[1] = dMPos1[1][3];dPos1[2] = dMPos1[2][3];
	dPos2[0] = dMPos2[0][3];dPos2[1] = dMPos2[1][3];dPos2[2] = dMPos2[2][3];	


	//姿态矩阵的差值
    //提取姿态矩阵
    double dMPos1N[3][3],dMPos2N[3][3] = {0};
    for(int i = 0;i < 2;i++)
        for(int j = 0;j < 2;j++)
        {
            dMPos1N[i][j] = dMPos1[i][j];
            dMPos2N[i][j] = dMPos2[i][j];
        }
	double dMAt[3][3] = {0};
	Matrix_AddOrSub(3,3,1,-1,&dMPos1N[0][0],&dMPos2N[0][0],&dMAt[0][0]);

	double dMAt12[2][2] = {0};
	for(int i = 0;i < 2;i++)
		for(int j = 0;j < 2;j++)
			dMAt12[i][j] = dMAt[i][j];

	double dMAtInv12[2][2]; //转置矩阵

	if(!Matrix_Inverse(2,&dMAt12[0][0],&dMAtInv12[0][0]))
			return ERROR_JOCAB_INV;

	////位置矩阵的差值
	double dPosAt21[2] = {0};
	Matrix_AddOrSub(2,1,1,-1,dPos2,dPos1,dPosAt21);


	//double dMAtInv12T[2][3]; //转置矩阵
	//Matrix_Transpose(3,2,&dMAtInv12[0][0],&dMAtInv12T[0][0]);
	//double dMulti[2][2];
	//Matrix_Multi(2,3,2,&dMAtInv12T[0][0],&dMAtInv12[0][0],&dMulti[0][0]);
	//double dInv_ATA[2][2];
	//if(!Matrix_Inverse(2,&dMulti[0][0],&dInv_ATA[0][0]))
	//	return ERROR_JOCAB_INV;
	//double dInv_LeftA[2][3];
	//Matrix_Multi(2,2,3,&dInv_ATA[0][0],&dMAtInv12T[0][0],&dInv_LeftA[0][0]);

	//Z轴的偏移量为0
	//dPosAt12[2] = 0;
	Matrix_Multi(2,2,1,&dMAtInv12[0][0],dPosAt21,dWorkOut);
	return 0;
}


/***********变位机坐标系标定*********************/
//输入三个标记点，计算出当前标记点对应变位机轴的变换矩阵
int HS_Calibration::GetPositionerAxisTrans(double *p1,double * p2,double * p3,double (*axisTrans)[4])
{
	//计算标记点对应圆心
	double center[3]={0};
	if(!m_HS_Kinematic->CalCenter(p1,p2,p3,center))
	{
		return -1;
	}

	//建立坐标系,为了简便，假设叉乘的两个向量不共线
	double dP1O[3] = {p1[0]-center[0],p1[1]-center[1],p1[2]-center[2]};
	double dP2O[3] = {p2[0]-center[0],p2[1]-center[1],p2[2]-center[2]};
	double dP3O[3] = {p3[0]-center[0],p3[1]-center[1],p3[2]-center[2]};
	double dCircleR = m_HS_Math.Matrix_Norm(3,1,dP1O);
	LOG_ALGO("center = %.3lf,%.3lf,%.3lf, R=%.3lf",
	center[0],center[1],center[2],dCircleR);
	double n_c[3] = {dP1O[0]/dCircleR,dP1O[1]/dCircleR,dP1O[2]/dCircleR};
	double a_c[3] = {0};
	m_HS_Math.Matrix_VecCross(dP1O,dP3O,a_c);
	double dNorm = m_HS_Math.Matrix_Norm(3,1,a_c);
	a_c[0] = a_c[0]/dNorm;
	a_c[1] = a_c[1]/dNorm;
	a_c[2] = a_c[2]/dNorm;

	//double dNorm = Matrix_Norm(3,1,a_c);
	//bool bOneLine = false;
	//if(dNorm < Eps)	//P0P1与P0P3共线
	//{
	//	Matrix_VecCross(dP1O,dP2O,a_c);
	//	dNorm = Matrix_Norm(3,1,a_c);
	//	if (dNorm < Eps)
	//	{
	//		return ERROR_CIRCLE_POINT;
	//	}
	//	else
	//	{
	//		a_c[0] = a_c[0]/dNorm;
	//		a_c[1] = a_c[1]/dNorm;
	//		a_c[2] = a_c[2]/dNorm;
	//	}
	//	bOneLine = true;
	//}
	//else
	//{
	//	a_c[0] = a_c[0]/dNorm;  
	//	a_c[1] = a_c[1]/dNorm;
	//	a_c[2] = a_c[2]/dNorm;
	//}


	double o_c[3];
	m_HS_Math.Matrix_VecCross(a_c,n_c,o_c);	
	//获取变换矩阵
	axisTrans[0][0] = n_c[0]; axisTrans[0][1] = o_c[0];axisTrans[0][2] = a_c[0];axisTrans[0][3] = center[0];
	axisTrans[1][0] = n_c[1]; axisTrans[1][1] = o_c[1];axisTrans[1][2] = a_c[1];axisTrans[1][3] = center[1];
	axisTrans[2][0] = n_c[2]; axisTrans[2][1] = o_c[2];axisTrans[2][2] = a_c[2];axisTrans[2][3] = center[2];
	axisTrans[3][0] =	   0; axisTrans[3][1] =      0;axisTrans[3][2] =      0;axisTrans[3][3] =         1;

	return 0;
}

/************************************************
函数功能：变位机标定算法
参    数： dPosIn 输入0~7点标定数据
		  df(degree of freedom) 变位机自由度
		  caliRes 标定矩阵
返 回 值：错误ID
*************************************************/
int HS_Calibration::Positioner(double **dPosIn,int df,double **caliRes)
{
	//计算变位机每一个轴{J1}{J2}{J3}在机器人基座标{B}下的变换矩阵
	if(df<1||df>3)
	{
		LOG_ALGO("Invalid Axis num");
		return -1;
	}
	else
	{
		LOG_ALGO("Axis num = %d",df);
	}
	if(df==1)
	{
		double BTJ1[4][4];//Base to Joint1 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[1],dPosIn[2],BTJ1);

		m_HS_Kinematic->HS_MPosToCPos(BTJ1,caliRes[0]);

	}
	else if(df==2)
	{
		double BTJ1[4][4];//Base to Joint1 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[1],dPosIn[2],BTJ1);
		double BTJ2[4][4];//Base to Joint2 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[3],dPosIn[4],BTJ2);

		double J1TB[4][4];
		m_HS_Math.Matrix_Inverse(4,BTJ1[0],J1TB[0]);
		double J1TJ2[4][4];
		m_HS_Math.Matrix_Multi(4,4,4,J1TB[0],BTJ2[0],J1TJ2[0]);

		m_HS_Kinematic->HS_MPosToCPos(BTJ1,caliRes[0]);
		m_HS_Kinematic->HS_MPosToCPos(J1TJ2,caliRes[1]);
		
	}
	else
	{
		double BTJ1[4][4];//Base to Joint1 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[1],dPosIn[2],BTJ1);
		double BTJ2[4][4];//Base to Joint2 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[3],dPosIn[4],BTJ2);
		double BTJ3[4][4];//Base to Joint3 in positioner
		GetPositionerAxisTrans(dPosIn[0],dPosIn[5],dPosIn[6],BTJ3);

		//计算关节间的变换矩阵 J2TJ3 J1TJ2
		double J1TB[4][4];
		m_HS_Math.Matrix_Inverse(4,BTJ1[0],J1TB[0]);
		double J1TJ2[4][4];
		m_HS_Math.Matrix_Multi(4,4,4,J1TB[0],BTJ2[0],J1TJ2[0]);
		double J2TB[4][4];
		m_HS_Math.Matrix_Inverse(4,BTJ2[0],J2TB[0]);
		double J2TJ3[4][4];
		m_HS_Math.Matrix_Multi(4,4,4,J2TB[0],BTJ3[0],J2TJ3[0]);
		
		//转化成坐标形式存储在标定矩阵caliRes中
		m_HS_Kinematic->HS_MPosToCPos(BTJ1,caliRes[0]);
		m_HS_Kinematic->HS_MPosToCPos(J1TJ2,caliRes[1]);
		m_HS_Kinematic->HS_MPosToCPos(J2TJ3,caliRes[2]);
		
	}

	LOG_ALGO(\
		"\n%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",\
		caliRes[0][0],caliRes[0][1],caliRes[0][2],caliRes[0][3],caliRes[0][4],caliRes[0][5],\
		caliRes[1][0],caliRes[1][1],caliRes[1][2],caliRes[1][3],caliRes[1][4],caliRes[1][5],\
		caliRes[2][0],caliRes[2][1],caliRes[2][2],caliRes[2][3],caliRes[2][4],caliRes[2][5]);

	return 0;
}

/************************************************
函数功能：外部轴标定
参    数： dPos1~3 3个点位坐标值
		  dEncoder 3个点位对应的编码器坐标值
		  dExtAxisEncoder 外部轴每圈脉冲数
		  dRatio 减速比（度/编码器脉冲）
返 回 值：错误ID
*************************************************/
int HS_Calibration::CaliExtAxisRatio(double *dPos1,double *dPos2,double *dPos3,double *dEncoder,double &dExtAxisEncoder,double &dRatio)
{
	LOG_ALGO("Pos1 = %.3lf,%.3lf,%.3lf;Pos2 = %.3lf,%.3lf,%.3lf;Pos3 = %.3lf,%.3lf,%.3lf",
					dPos1[0],dPos1[1],dPos1[2],dPos2[0],dPos2[0],dPos2[0],dPos3[0],dPos3[0],dPos3[0]);

	//计算标记点对应圆心
	double center[3]={0};
	if(!m_HS_Kinematic->CalCenter(dPos1,dPos2,dPos3,center))
	{
		return -1;
	}
	LOG_ALGO("center = %.3lf,%.3lf,%.3lf;",center[0],center[1],center[2]);
	//建立坐标系,为了简便，假设叉乘的两个向量不共线
	double dP1C[3] = {dPos1[0]-center[0],dPos1[1]-center[1],dPos1[2]-center[2]};
	double dP2C[3] = {dPos2[0]-center[0],dPos2[1]-center[1],dPos2[2]-center[2]};
	double dP3C[3] = {dPos3[0]-center[0],dPos3[1]-center[1],dPos3[2]-center[2]};
	double dCircleR = m_HS_Math.Matrix_Norm(3,1,dP1C);

	double n_c[3] = {dP1C[0]/dCircleR,dP1C[1]/dCircleR,dP1C[2]/dCircleR};
	double a_c[3] = {0};
	m_HS_Math.Matrix_VecCross(dP1C,dP3C,a_c);
	double dNorm = m_HS_Math.Matrix_Norm(3,1,a_c);
	bool bOneLine = false;
	if(dNorm < Eps)	//P0P1与P0P3共线
	{
		m_HS_Math.Matrix_VecCross(dP1C,dP2C,a_c);
		dNorm = m_HS_Math.Matrix_Norm(3,1,a_c);
		if (dNorm < Eps)
		{
			return ERROR_CIRCLE_POINT;
		}
		else
		{
			a_c[0] = a_c[0]/dNorm;
			a_c[1] = a_c[1]/dNorm;
			a_c[2] = a_c[2]/dNorm;
		}
		bOneLine = true;
	}
	else
	{
		a_c[0] = a_c[0]/dNorm;
		a_c[1] = a_c[1]/dNorm;
		a_c[2] = a_c[2]/dNorm;
	}

	double dSeata = 0;
	double o_c[3];
	m_HS_Math.Matrix_VecCross(a_c,n_c,o_c);	
	double m_TCircle[4][4];						//圆变换矩阵
	//得到由圆坐标系变换到基础坐标系矩阵
	m_TCircle[0][0] = n_c[0]; m_TCircle[0][1] = o_c[0];m_TCircle[0][2] = a_c[0];m_TCircle[0][3] = center[0];
	m_TCircle[1][0] = n_c[1]; m_TCircle[1][1] = o_c[1];m_TCircle[1][2] = a_c[1];m_TCircle[1][3] = center[1];
	m_TCircle[2][0] = n_c[2]; m_TCircle[2][1] = o_c[2];m_TCircle[2][2] = a_c[2];m_TCircle[2][3] = center[2];
	m_TCircle[3][0] =	   0; m_TCircle[3][1] =      0;m_TCircle[3][2] =      0;m_TCircle[3][3] =         1;

	//求逆矩阵
	double dT[4][4];	
	m_HS_Math.Matrix_Inverse(4,&m_TCircle[0][0],&dT[0][0]);	

	//求解P3和P2在新坐标系下的坐标
	double dP2N_C[4];	//圆弧坐标系位置
	double dP2N_W[4] = {dPos2[0],dPos2[1],dPos2[2],1};
	m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP2N_W,dP2N_C);

	double dP3N_C[4];
	double dP3N_W[4] = {dPos3[0],dPos3[1],dPos3[2],1};
	m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP3N_W,dP3N_C);
	double dSeata1 = atan2(dP2N_C[1],dP2N_C[0]);	//中间点
	double dSeata2 = atan2(dP3N_C[1],dP3N_C[0]);
	if(bOneLine)
		dSeata2 = PI;

	double dAngle = fabs(dSeata2*180/PI);		//度
	double dDisEncoder = fabs(dEncoder[2] - dEncoder[0]);
	/*if(dDisEncoder < Eps)
		return -1;
	else
	{
		dRatio = dAngle/dDisEncoder; 
	}*/

	//外部轴旋转圈数
	double dExtACircleNum = 0;
	if(dExtAxisEncoder < Eps)
		return -1;
	else
	{
		dExtACircleNum = dDisEncoder/dExtAxisEncoder;
	}
	//外部轴旋转角度
	double dExtACircleAngle = dExtACircleNum*360;
	//减速比
	if(dAngle < Eps)
		return -1;
	else
	{
		dRatio = dExtACircleAngle/dAngle; 
	}

	LOG_ALGO("dRatio = %.3lf;",dRatio);
	
	return 0;
}


////////////////////////////////////////////////////
void HS_Calibration::buildmodel()
{
	T1[0] = cos(e[0] + g[0]); T1[1] = -sin(e[0] + g[0])*cos(f[0]); T1[2] = sin(e[0] + g[0])*sin(f[0]); T1[3] = a[0] * cos(e[0] + g[0]);
	T1[4] = sin(e[0] + g[0]); T1[5] = cos(e[0] + g[0])*cos(f[0]); T1[6] = -cos(e[0] + g[0])*sin(f[0]); T1[7] = a[0] * sin(e[0] + g[0]);
	T1[8] = 0; T1[9] = sin(f[0]); T1[10] = cos(f[0]); T1[11] = d[0];
	T1[12] = 0; T1[13] = 0; T1[14] = 0; T1[15] = 1;

	T2[0] = cos(e[1] + g[1]); T2[1] = -sin(e[1] + g[1])*cos(f[1]); T2[2] = sin(e[1] + g[1])*sin(f[1]); T2[3] = a[1] * cos(e[1] + g[1]);
	T2[4] = sin(e[1] + g[1]); T2[5] = cos(e[1] + g[1])*cos(f[1]); T2[6] = -cos(e[1] + g[1])*sin(f[1]); T2[7] = a[1] * sin(e[1] + g[1]);
	T2[8] = 0; T2[9] = sin(f[1]); T2[10] = cos(f[1]); T2[11] = d[1];
	T2[12] = 0; T2[13] = 0; T2[14] = 0; T2[15] = 1;

	T3[0] = cos(e[2] + g[2]); T3[1] = -sin(e[2] + g[2])*cos(f[2]); T3[2] = sin(e[2] + g[2])*sin(f[2]); T3[3] = a[2] * cos(e[2] + g[2]);
	T3[4] = sin(e[2] + g[2]); T3[5] = cos(e[2] + g[2])*cos(f[2]); T3[6] = -cos(e[2] + g[2])*sin(f[2]); T3[7] = a[2] * sin(e[2] + g[2]);
	T3[8] = 0; T3[9] = sin(f[2]); T3[10] = cos(f[2]); T3[11] = d[2];
	T3[12] = 0; T3[13] = 0; T3[14] = 0; T3[15] = 1;

	T4[0] = cos(e[3] + g[3]); T4[1] = -sin(e[3] + g[3])*cos(f[3]); T4[2] = sin(e[3] + g[3])*sin(f[3]); T4[3] = a[3] * cos(e[3] + g[3]);
	T4[4] = sin(e[3] + g[3]); T4[5] = cos(e[3] + g[3])*cos(f[3]); T4[6] = -cos(e[3] + g[3])*sin(f[3]); T4[7] = a[3] * sin(e[3] + g[3]);
	T4[8] = 0; T4[9] = sin(f[3]); T4[10] = cos(f[3]); T4[11] = d[3];
	T4[12] = 0; T4[13] = 0; T4[14] = 0; T4[15] = 1;

	T5[0] = cos(e[4] + g[4]); T5[1] = -sin(e[4] + g[4])*cos(f[4]); T5[2] = sin(e[4] + g[4])*sin(f[4]); T5[3] = a[4] * cos(e[4] + g[4]);
	T5[4] = sin(e[4] + g[4]); T5[5] = cos(e[4] + g[4])*cos(f[4]); T5[6] = -cos(e[4] + g[4])*sin(f[4]); T5[7] = a[4] * sin(e[4] + g[4]);
	T5[8] = 0; T5[9] = sin(f[4]); T5[10] = cos(f[4]); T5[11] = d[4];
	T5[12] = 0; T5[13] = 0; T5[14] = 0; T5[15] = 1;

	T6[0] = cos(e[5] + g[5]); T6[1] = -sin(e[5] + g[5])*cos(f[5]); T6[2] = sin(e[5] + g[5])*sin(f[5]); T6[3] = a[5] * cos(e[5] + g[5]);
	T6[4] = sin(e[5] + g[5]); T6[5] = cos(e[5] + g[5])*cos(f[5]); T6[6] = -cos(e[5] + g[5])*sin(f[5]); T6[7] = a[5] * sin(e[5] + g[5]);
	T6[8] = 0; T6[9] = sin(f[5]); T6[10] = cos(f[5]); T6[11] = d[5];
	T6[12] = 0; T6[13] = 0; T6[14] = 0; T6[15] = 1;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			if (i == j)
			{
				Tcam2base[i * 4 + j] = 1;
				Ttool[i * 4 + j] = 1;
			}
			else
			{
				Tcam2base[i * 4 + j] = 0;
				Ttool[i * 4 + j] = 0;
			}
		}
	}
	Tcam2base[3] = bx; Tcam2base[7] = by; Tcam2base[11] = bz;
	Ttool[3] = tx; Ttool[7] = ty; Ttool[11] = tz;
	double Ttemp1[16], Ttemp2[16];
	HS_Math::Matrix_Multi(4, 4, 4, T1, T2, Ttemp1);
	HS_Math::Matrix_Multi(4, 4, 4, Ttemp1, T3, Ttemp2);
	HS_Math::Matrix_Multi(4, 4, 4, Ttemp2, T4, Ttemp1);
	HS_Math::Matrix_Multi(4, 4, 4, Ttemp1, T5, Ttemp2);
	HS_Math::Matrix_Multi(4, 4, 4, Ttemp2, T6, Ttemp1);
	for (int i = 0; i < 16; ++i)
	{
		T06[i] = Ttemp1[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, Ttemp1, Ttool, Ttemp2);
	for (int i = 0; i < 16; ++i)
	{
		T0t[i] = Ttemp2[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, Tcam2base, Ttemp2, Ttemp1);
	for (int i = 0; i < 16; ++i)
	{
		Tcam2t[i] = Ttemp1[i];
	}
}
void HS_Calibration::derivative()
{
	GB[0] = cos(bb)*cos(bc); GB[1] = cos(bb)*sin(bc); GB[2] = -sin(bb);
	GB[3] = cos(bc)*sin(ba)*sin(bb) - cos(ba)*sin(bc); GB[4] = cos(ba)*cos(bc) + sin(ba)*sin(bb)*sin(bc); GB[5] = cos(bb)*sin(ba);
	GB[6] = sin(ba)*sin(bc) + cos(ba)*cos(bc)*sin(bb); GB[7] = cos(ba)*sin(bb)*sin(bc) - cos(bc)*sin(ba); GB[8] = cos(ba)*cos(bb);
	for (int i = 9; i < 18; ++i)
	{
		GB[i] = 0;
	}
	G1[0] = G2[0] = G3[0] = G4[0] = G5[0] = G6[0] = 0;
	G1[3] = G2[3] = G3[3] = G4[3] = G5[3] = G6[3] = 0;
	G1[1] = a[0] * cos(f[0]); G2[1] = a[1] * cos(f[1]); G3[1] = a[2] * cos(f[2]);
	G4[1] = a[3] * cos(f[3]); G5[1] = a[4] * cos(f[4]); G6[1] = a[5] * cos(f[5]);
	G1[2] = - a[0] * sin(f[0]); G2[2] = - a[1] * sin(f[1]); G3[2] = - a[2] * sin(f[2]);
	G4[2] = - a[3] * sin(f[3]); G5[2] = - a[4] * sin(f[4]); G6[2] = - a[5] * sin(f[5]);
	G1[4] = sin(f[0]); G2[4] = sin(f[1]); G3[4] = sin(f[2]);
	G4[4] = sin(f[3]); G5[4] = sin(f[4]); G6[4] = sin(f[5]);
	G1[5] = cos(f[0]); G2[5] = cos(f[1]); G3[5] = cos(f[2]);
	G4[5] = cos(f[3]); G5[5] = cos(f[4]); G6[5] = cos(f[5]);
}
void HS_Calibration::jaco()
{
	double T1t[16], T2t[16], T3t[16], T4t[16], T5t[16], Ttemp1[16], Ttemp2[16];
	HS_Math::Matrix_Multi(4, 4, 4, T6, Ttool, Ttemp1);
	for (int i = 0; i < 16; ++i)
	{
		T5t[i] = Ttemp1[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, T5, Ttemp1, Ttemp2);
	for (int i = 0; i < 16; ++i)
	{
		T4t[i] = Ttemp2[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, T4, Ttemp2, Ttemp1);
	for (int i = 0; i < 16; ++i)
	{
		T3t[i] = Ttemp1[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, T3, Ttemp1, Ttemp2);
	for (int i = 0; i < 16; ++i)
	{
		T2t[i] = Ttemp2[i];
	}
	HS_Math::Matrix_Multi(4, 4, 4, T2, Ttemp2, Ttemp1);
	for (int i = 0; i < 16; ++i)
	{
		T1t[i] = Ttemp1[i];
	}
	//Next is Jin's R
	double R0t[9], R1t[9], R2t[9], R3t[9], R4t[9], R5t[9];
	HS_Math::Matrix_Transpose(4, 4, T5t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R5t);
	HS_Math::Matrix_Transpose(4, 4, T4t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R4t);
	HS_Math::Matrix_Transpose(4, 4, T3t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R3t);
	HS_Math::Matrix_Transpose(4, 4, T2t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R2t);
	HS_Math::Matrix_Transpose(4, 4, T1t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R1t);
	HS_Math::Matrix_Transpose(4, 4, T0t, Ttemp1);
	HS_Math::Block(0, 0, 3, 3, Ttemp1, 4, 4, R0t);
	//Next is Jin's P
	//"_" is cross product
	double n6t[3], o6t[3], a6t[3], p6t[3], p6t_n6t[3], p6t_o6t[3], p6t_a6t[3];
	HS_Math::Block(0, 0, 3, 1, T5t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T5t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T5t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T5t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J5t[0] = R5t[0]; J5t[1] = R5t[1]; J5t[2] = R5t[2]; J5t[3] = p6t_n6t[0]; J5t[4] = p6t_n6t[1]; J5t[5] = p6t_n6t[2];
	J5t[6] = R5t[3]; J5t[7] = R5t[4]; J5t[8] = R5t[5]; J5t[9] = p6t_o6t[0]; J5t[10] = p6t_o6t[1]; J5t[11] = p6t_o6t[2];
	J5t[12] = R5t[6]; J5t[13] = R5t[7]; J5t[14] = R5t[8]; J5t[15] = p6t_a6t[0]; J5t[16] = p6t_a6t[1]; J5t[17] = p6t_a6t[2];
	HS_Math::Block(0, 0, 3, 1, T4t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T4t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T4t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T4t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J4t[0] = R4t[0]; J4t[1] = R4t[1]; J4t[2] = R4t[2]; J4t[3] = p6t_n6t[0]; J4t[4] = p6t_n6t[1]; J4t[5] = p6t_n6t[2];
	J4t[6] = R4t[3]; J4t[7] = R4t[4]; J4t[8] = R4t[5]; J4t[9] = p6t_o6t[0]; J4t[10] = p6t_o6t[1]; J4t[11] = p6t_o6t[2];
	J4t[12] = R4t[6]; J4t[13] = R4t[7]; J4t[14] = R4t[8]; J4t[15] = p6t_a6t[0]; J4t[16] = p6t_a6t[1]; J4t[17] = p6t_a6t[2];
	HS_Math::Block(0, 0, 3, 1, T3t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T3t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T3t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T3t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J3t[0] = R3t[0]; J3t[1] = R3t[1]; J3t[2] = R3t[2]; J3t[3] = p6t_n6t[0]; J3t[4] = p6t_n6t[1]; J3t[5] = p6t_n6t[2];
	J3t[6] = R3t[3]; J3t[7] = R3t[4]; J3t[8] = R3t[5]; J3t[9] = p6t_o6t[0]; J3t[10] = p6t_o6t[1]; J3t[11] = p6t_o6t[2];
	J3t[12] = R3t[6]; J3t[13] = R3t[7]; J3t[14] = R3t[8]; J3t[15] = p6t_a6t[0]; J3t[16] = p6t_a6t[1]; J3t[17] = p6t_a6t[2];
	HS_Math::Block(0, 0, 3, 1, T2t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T2t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T2t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T2t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J2t[0] = R2t[0]; J2t[1] = R2t[1]; J2t[2] = R2t[2]; J2t[3] = p6t_n6t[0]; J2t[4] = p6t_n6t[1]; J2t[5] = p6t_n6t[2];
	J2t[6] = R2t[3]; J2t[7] = R2t[4]; J2t[8] = R2t[5]; J2t[9] = p6t_o6t[0]; J2t[10] = p6t_o6t[1]; J2t[11] = p6t_o6t[2];
	J2t[12] = R2t[6]; J2t[13] = R2t[7]; J2t[14] = R2t[8]; J2t[15] = p6t_a6t[0]; J2t[16] = p6t_a6t[1]; J2t[17] = p6t_a6t[2];
	HS_Math::Block(0, 0, 3, 1, T1t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T1t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T1t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T1t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J1t[0] = R1t[0]; J1t[1] = R1t[1]; J1t[2] = R1t[2]; J1t[3] = p6t_n6t[0]; J1t[4] = p6t_n6t[1]; J1t[5] = p6t_n6t[2];
	J1t[6] = R1t[3]; J1t[7] = R1t[4]; J1t[8] = R1t[5]; J1t[9] = p6t_o6t[0]; J1t[10] = p6t_o6t[1]; J1t[11] = p6t_o6t[2];
	J1t[12] = R1t[6]; J1t[13] = R1t[7]; J1t[14] = R1t[8]; J1t[15] = p6t_a6t[0]; J1t[16] = p6t_a6t[1]; J1t[17] = p6t_a6t[2];
	HS_Math::Block(0, 0, 3, 1, T0t, 4, 4, n6t);
	HS_Math::Block(0, 1, 3, 1, T0t, 4, 4, o6t);
	HS_Math::Block(0, 2, 3, 1, T0t, 4, 4, a6t);
	HS_Math::Block(0, 3, 3, 1, T0t, 4, 4, p6t);
	HS_Math::Matrix_VecCross(p6t, n6t, p6t_n6t);
	HS_Math::Matrix_VecCross(p6t, o6t, p6t_o6t);
	HS_Math::Matrix_VecCross(p6t, a6t, p6t_a6t);
	J0t[0] = R0t[0]; J0t[1] = R0t[1]; J0t[2] = R0t[2]; J0t[3] = p6t_n6t[0]; J0t[4] = p6t_n6t[1]; J0t[5] = p6t_n6t[2];
	J0t[6] = R0t[3]; J0t[7] = R0t[4]; J0t[8] = R0t[5]; J0t[9] = p6t_o6t[0]; J0t[10] = p6t_o6t[1]; J0t[11] = p6t_o6t[2];
	J0t[12] = R0t[6]; J0t[13] = R0t[7]; J0t[14] = R0t[8]; J0t[15] = p6t_a6t[0]; J0t[16] = p6t_a6t[1]; J0t[17] = p6t_a6t[2];
	J6t[0] = 1; J6t[1] = 0; J6t[2] = 0; J6t[3] = 0; J6t[4] = tz; J6t[5] = -ty;
	J6t[0] = 0; J6t[1] = 1; J6t[2] = 0; J6t[3] = -tz; J6t[4] = 0; J6t[5] = tx;
	J6t[0] = 0; J6t[1] = 0; J6t[2] = 1; J6t[3] = ty; J6t[4] = -tx; J6t[5] = 0;
}
void HS_Calibration::h_deltax1(double *H1, double *deltax)
{
	//更新dh参数
	//---------------------------------------
	double QB_before[16],J0tGB[9];
	for (int i = 0; i < datarows; ++i)
	{
		e[0] = xita6[i][0] * PI / 180; e[1] = xita6[i][1] * PI / 180; e[2] = xita6[i][2] * PI / 180;
		e[3] = xita6[i][3] * PI / 180; e[4] = xita6[i][4] * PI / 180; e[5] = xita6[i][5] * PI / 180;
		buildmodel();
		derivative();
		jaco();
		HS_Math::Matrix_Inverse(4, Tcam2t, QB_before);
		deltax[3*i] = QB_before[3]; deltax[3*i+1] = QB_before[7]; deltax[3*i+2] = QB_before[11];
		HS_Math::Matrix_Multi(3, 6, 3, J0t, GB, J0tGB);
		H1[9 * i] = J0tGB[0]; H1[9 * i + 1] = J0tGB[1]; H1[9 * i + 2] = J0tGB[2];
		H1[9 * i + 3] = J0tGB[3]; H1[9 * i + 4] = J0tGB[4]; H1[9 * i + 5] = J0tGB[5];
		H1[9 * i + 6] = J0tGB[6]; H1[9 * i + 7] = J0tGB[7]; H1[9 * i + 8] = J0tGB[8];
	}
}
void HS_Calibration::h_deltax2(double *H2, double *deltax)
{
	//更新dh参数
	//---------------------------------------
	double QB_before[16], J0tGB[9];
	for (int i = 0; i < datarows; ++i)
	{
		e[0] = xita6[i][0] * PI / 180; e[1] = xita6[i][1] * PI / 180; e[2] = xita6[i][2] * PI / 180;
		e[3] = xita6[i][3] * PI / 180; e[4] = xita6[i][4] * PI / 180; e[5] = xita6[i][5] * PI / 180;
		buildmodel();
		derivative();
		jaco();
		HS_Math::Matrix_Inverse(4, Tcam2t, QB_before);
		deltax[3 * i] = QB_before[3]; deltax[3 * i + 1] = QB_before[7]; deltax[3 * i + 2] = QB_before[11];
		HS_Math::Matrix_Multi(3, 6, 3, J0t, GB, J0tGB);
		H2[18 * i] = J0tGB[0]; H2[18 * i + 1] = J0tGB[1]; H2[18 * i + 2] = J0tGB[2];
		H2[18 * i + 3] = 1; H2[18 * i + 4] = 0; H2[18 * i + 5] = 0;

		H2[18 * i + 6] = J0tGB[3]; H2[18 * i + 7] = J0tGB[4]; H2[18 * i + 8] = J0tGB[5];
		H2[18 * i + 9] = 0; H2[18 * i + 10] = 1; H2[18 * i + 11] = 0;

		H2[18 * i + 12] = J0tGB[6]; H2[18 * i + 13] = J0tGB[7]; H2[18 * i + 14] = J0tGB[8];
		H2[18 * i + 15] = 0; H2[18 * i + 16] = 0; H2[18 * i + 17] = 1;
	}
}
void HS_Calibration::h_deltax3(double *H3, double *deltax)
{
	//更新dh参数
	//---------------------------------------
	double QB_before[16], J0tGB[9], J2tG2[3], J3tG3[3], J4tG4[3], J5tG5[3];//J1tG1[3],
	for (int i = 0; i < datarows; ++i)
	{
		e[0] = xita6[i][0] * PI / 180; e[1] = xita6[i][1] * PI / 180; e[2] = xita6[i][2] * PI / 180;
		e[3] = xita6[i][3] * PI / 180; e[4] = xita6[i][4] * PI / 180; e[5] = xita6[i][5] * PI / 180;
		buildmodel();
		derivative();
		jaco();
		HS_Math::Matrix_Inverse(4, Tcam2t, QB_before);
		deltax[3 * i] = QB_before[3]; deltax[3 * i + 1] = QB_before[7]; deltax[3 * i + 2] = QB_before[11];
		HS_Math::Matrix_Multi(3, 6, 3, J0t, GB, J0tGB);
		//HS_Math::Matrix_Multi(3, 6, 1, J1t, G1, J1tG1);
		HS_Math::Matrix_Multi(3, 6, 1, J2t, G2, J2tG2);
		HS_Math::Matrix_Multi(3, 6, 1, J3t, G3, J3tG3);
		HS_Math::Matrix_Multi(3, 6, 1, J4t, G4, J4tG4);
		HS_Math::Matrix_Multi(3, 6, 1, J5t, G5, J5tG5);

		H3[30 * i] = J2tG2[0]; H3[30 * i + 1] = J3tG3[0]; H3[30 * i + 2] = J4tG4[0]; H3[30 * i + 3] = J5tG5[0];
		H3[30 * i + 4] = 1; H3[30 * i + 5] = 0; H3[30 * i + 6] = 0; H3[30 * i + 7] = J0tGB[0]; H3[30 * i + 8] = J0tGB[1]; H3[30 * i + 9] = J0tGB[2];

		H3[30 * i + 10] = J2tG2[1]; H3[30 * i + 11] = J3tG3[1]; H3[30 * i + 12] = J4tG4[1]; H3[30 * i + 13] = J5tG5[1];
		H3[30 * i + 14] = 0; H3[30 * i + 15] = 1; H3[30 * i + 16] = 0; H3[30 * i + 17] = J0tGB[3]; H3[30 * i + 18] = J0tGB[4]; H3[30 * i + 19] = J0tGB[5];

		H3[30 * i + 20] = J2tG2[2]; H3[30 * i + 21] = J3tG3[2]; H3[30 * i + 22] = J4tG4[2]; H3[30 * i + 23] = J5tG5[2];
		H3[30 * i + 24] = 0; H3[30 * i + 25] = 0; H3[30 * i + 26] = 1; H3[30 * i + 27] = J0tGB[6]; H3[30 * i + 28] = J0tGB[7]; H3[30 * i + 29] = J0tGB[8];
	}
}
void HS_Calibration::verify()
{
	//更新dh参数
	//---------------------------------------
	double *T_afid=new double[3 * datarows];
	for (int i = 0; i < datarows; ++i)
	{
		e[0] = xita6[i][0] * PI / 180; e[1] = xita6[i][1] * PI / 180; e[2] = xita6[i][2] * PI / 180;
		e[3] = xita6[i][3] * PI / 180; e[4] = xita6[i][4] * PI / 180; e[5] = xita6[i][5] * PI / 180;
		buildmodel();
		for (int j = 0; j < 3; ++j)
		{
			T_afid[3 * i + j] = Tcam2t[4 * j + 3];
		}
	}
	double *Differ_pos=new double[datarows];
	double temp;
	for (int i = 0; i < datarows; ++i)
	{
		temp = T_afid[3 * i] * T_afid[3 * i] + T_afid[3 * i + 1] * T_afid[3 * i + 1] + T_afid[3 * i + 2] * T_afid[3 * i + 2];
		Differ_pos[i] = sqrt(temp);
	}
	double tempb = 0.0, sum = 0.0;
	for (int j = 0; j < datarows - 1; ++j)
	{
		if (Differ_pos[j] > Differ_pos[j + 1])
		{
			tempb = Differ_pos[j];
			Differ_pos[j] = Differ_pos[j + 1];
			Differ_pos[j + 1] = tempb;
		}
	}
	for (int i = 0; i < datarows; ++i)
	{
		sum = sum + Differ_pos[i];
	}
	max_error = Differ_pos[datarows - 1];
	aver_error = sum / datarows;
	delete[]T_afid;
	delete[]Differ_pos;
}
void HS_Calibration::Kalman(double* H_,double* deltax)
{
	int N = datarows;
	double H_temp[3 * 11];
	for (int i = 0; i < 33 ; ++i)
	{
		H_temp[i] = H_[i];
	}
	double P[11 * 11] = { 0 }, P_pre[11 * 11] = { 0 }, Q[11 * 11] = { 0 }, I[11 * 11] = { 0 };
	for (int i = 0; i < 11; ++i)
	{
		P[12 * i] = 0.0001;
		Q[12 * i] = 0.00000001;
		I[12 * i] = 1;
	}
	double R[3 * 3] = { 0 };
	R[0] = 0.000001; R[4] = 0.000001; R[8] = 0.000001;
	double Kg[11 * 3], H_trans[11 * 3];
	double Xkf[11] = { 0 }, X_pre[11],Kge[11];
	double Z[3], e[3], H_Xpre[3];
	Z[0] = deltax[0]; Z[1] = deltax[1]; Z[2] = deltax[2];
	double Kg1[11 * 3],Kg2[3 * 11],Kg3[3 * 3], Kg3i[3 * 3];
	double KgHtemp[11 * 11], IKgHtemp[11 * 11];
	int kl;
	for (kl = 1; kl < N; ++kl)
	{
		for (int i = 0; i < 11; ++i)
		{
			X_pre[i] = Xkf[i];
		}
		for (int i = 0; i < 121; ++i)
		{
			P_pre[i] = P[i] + Q[i];
		}
		HS_Math::Matrix_Transpose(3, 11, H_temp, H_trans);
		HS_Math::Matrix_Multi(11, 11, 3, P_pre, H_trans, Kg1);
		HS_Math::Matrix_Multi(3, 11, 3, H_temp, Kg1, Kg2);
		for (int i = 0; i < 9; ++i)
		{
			Kg3[i] = Kg2[i] + R[i];
		}
		HS_Math::Matrix_Inverse(3, Kg3, Kg3i);
		HS_Math::Matrix_Multi(11, 3, 3, Kg1, Kg3i, Kg);
		HS_Math::Matrix_Multi(3, 11, 1, H_temp, X_pre , H_Xpre);
		for (int i = 0; i < 3; ++i)
		{
			e[i] = Z[i] - H_Xpre[i];
		}
		HS_Math::Matrix_Multi(11, 3, 1, Kg, e, Kge);
		for (int i = 0; i < 11; ++i)
		{
			Xkf[i] = X_pre[i] + Kge[i];
		}
		HS_Math::Matrix_Multi(11, 3, 11, Kg, H_temp, KgHtemp);
		for (int i = 0; i < 121; ++i)
		{
			IKgHtemp[i] = I[i] - KgHtemp[i];
		}
		HS_Math::Matrix_Multi(11, 11, 11, IKgHtemp, P_pre, P);
		Z[0] = deltax[3 * kl]; Z[1] = deltax[3 * kl + 1]; Z[2] = deltax[3 * kl + 2];
		for (int i = 0; i < 33; ++i)
		{
			H_temp[i] = H_[33 * kl + i];
		}
	}
	for (int i = 0; i < 11; ++i)
	{
		X_pre[i] = Xkf[i];
	}
	for (int i = 0; i < 121; ++i)
	{
		P_pre[i] = P[i] + Q[i];
	}
	HS_Math::Matrix_Transpose(3, 11, H_temp, H_trans);
	HS_Math::Matrix_Multi(11, 11, 3, P_pre, H_trans, Kg1);
	HS_Math::Matrix_Multi(3, 11, 3, H_temp, Kg1, Kg2);
	for (int i = 0; i < 9; ++i)
	{
		Kg3[i] = Kg2[i] + R[i];
	}
	HS_Math::Matrix_Inverse(3, Kg3, Kg3i);
	HS_Math::Matrix_Multi(11, 3, 3, Kg1, Kg3i, Kg);
	HS_Math::Matrix_Multi(3, 11, 1, H_temp, X_pre, H_Xpre);
	for (int i = 0; i < 3; ++i)
	{
		e[i] = Z[i] - H_Xpre[i];
	}
	HS_Math::Matrix_Multi(11, 3, 1, Kg, e, Kge);
	for (int i = 0; i < 11; ++i)
	{
		solx3[i] = X_pre[i] + Kge[i];
	}
}
void HS_Calibration::setdh(double dDHPara[6][3])
{
	for (int i = 0; i < 6; i++)
	{
		g[i] = 0;
		a[i] = dDHPara[i][0];
		d[i] = dDHPara[i][1];
		f[i] = dDHPara[i][2]*PI/180;
	};
	bx = 0; by = 0; bz = 0; ba = 0; bb = 0; bc = 0;
	tx = 0; ty = 0; tz = 0;
}
void HS_Calibration::printdh()
{
	LOG_ALGO("-----------------平均误差和最大误差------------------");
	LOG_ALGO("平均误差:%fmm,最大误差:%fmm\n",aver_error,max_error);
	LOG_ALGO("-----------------------------打印零漂-------------------------");
	LOG_ALGO("θ1 = %f, θ2 = %f, θ3 = %f, θ4 = %f, θ5 = %f, θ6 = %f",g[0] * 180 / PI,g[1] * 180 / PI,g[2] * 180 / PI,g[3] * 180 / PI,g[4] * 180 / PI,g[5] * 180 / PI);
	LOG_ALGO("------------------------------打印TCP----------------------------");
	LOG_ALGO("tx = %f, ty = %f, tz = %f",tx,ty,tz);
	LOG_ALGO("---------------------------------End-----------------------------");
}
int HS_Calibration::getdatarows(double dDataIn[20][6])
{
	int i = 0;
	for (; i < 20 && (dDataIn[i][0] > 1e-8 || dDataIn[i][0] < -1e-8 || dDataIn[i][1] > 1e-8 || dDataIn[i][1] < -1e-8 || dDataIn[i][2] > 1e-8 ||
		dDataIn[i][2] < -1e-8 || dDataIn[i][3] > 1e-8 || dDataIn[i][3] < -1e-8 || dDataIn[i][4] > 1e-8 || dDataIn[i][4] < -1e-8 ||
		dDataIn[i][5] > 1e-8 || dDataIn[i][5] < -1e-8) ; ++i);

    //取消该断言，会导致崩溃
	//assert(i > 9);
	return i;
}
/************************************************
函数功能：综合标定算法
参    数：dDataIn---标定输入点位
		 dOut1-----标定输出1
		 dOut2-----标定输出2
返 回 值：错误ID
*************************************************/
int HS_Calibration::HS_IntegratedCalibration(double (*dDHPara)[3], double (*dDataIn)[6],double *dOut1,double *dOut2)
{
	setdh(dDHPara);
	datarows = getdatarows(dDataIn);
	double H1temp[9],H1tempi[9],H1temp2[3];
	double *H1 = new double[9 * datarows];
	double *H1temp1 = new double[9 * datarows];
	double *deltax = new double[3 * datarows];
	xita6 = dDataIn;
	for (int kk = 0; kk < 6; ++kk)
	{
		h_deltax1(H1,deltax);
		HS_Math::Matrix_Transpose(3 * datarows, 3, H1, H1temp1);
		HS_Math::Matrix_Multi(3, 3 * datarows, 3, H1temp1, H1, H1temp);
		HS_Math::Matrix_Inverse(3, H1temp, H1tempi);
		HS_Math::Matrix_Multi(3, 3 * datarows, 1, H1temp1, deltax, H1temp2);
		HS_Math::Matrix_Multi(3, 3, 1, H1tempi, H1temp2, solx1);
		bx = bx + solx1[0]; by = by + solx1[1]; bz = bz + solx1[2];
	}
	delete[] H1; delete[] H1temp1;
	double *H2 = new double[18 * datarows];
	double H2temp[36], H2tempi[36], H2temp2[6];
	double *H2temp1(new double[18 * datarows]());
	for (int kk = 0; kk < 6; ++kk)
	{
		h_deltax2(H2,deltax);
		HS_Math::Matrix_Transpose(3 * datarows, 6, H2, H2temp1);
		HS_Math::Matrix_Multi(6, 3 * datarows, 6, H2temp1, H2, H2temp);
		HS_Math::Matrix_Inverse(6, H2temp, H2tempi);
		HS_Math::Matrix_Multi(6, 3 * datarows, 1, H2temp1, deltax, H2temp2);
		HS_Math::Matrix_Multi(6, 6, 1, H2tempi, H2temp2, solx2);
		bx = bx + solx2[0]; by = by + solx2[1]; bz = bz + solx2[2];
		tx = tx + solx2[3]; ty = ty + solx2[4]; tz = tz + solx2[5];
	}
	delete[] H2; delete[] H2temp1;
	double D[10 * 10] = { 0 };
	double temp3,dtemp;
	double *H3 = new double[30 * datarows];
	double *H3t = new double[30 * datarows];
	double H3temp[100], H3tempi[100], H3temp2[10];
	double *H3temp1(new double[30 * datarows]());
	for (int kk = 0; kk < 2; ++kk)
	{
		h_deltax3(H3,deltax);
		for (int nn = 0; nn < 10; ++nn)
		{
			temp3 = HS_Math::Matrix_ColNorm(3 * datarows, 10, nn, H3);
			dtemp = 1.0 / temp3;
			D[nn * 11] = dtemp;
		}
		HS_Math::Matrix_Multi(3 * datarows, 10, 10, H3, D, H3t);
		//lse();
		HS_Math::Matrix_Transpose(3 * datarows, 10, H3t, H3temp1);
		HS_Math::Matrix_Multi(10, 3 * datarows, 10, H3temp1, H3t, H3temp);
		HS_Math::Matrix_Inverse(10, H3temp, H3tempi);
		HS_Math::Matrix_Multi(10, 3 * datarows, 1, H3temp1, deltax, H3temp2);
		HS_Math::Matrix_Multi(10,10, 1, H3tempi, H3temp2, solx3);
		//Kalman(H3t,deltax);
		HS_Math::Matrix_Multi(10, 10, 1, D, solx3, solx);
		g[1] += solx[0]; g[2] += solx[1]; g[3] += solx[2]; g[4] += solx[3];
		bx += solx[7]; by += solx[8]; bz += solx[9];
		tx += solx[4]; ty += solx[5]; tz += solx[6];
	}
	delete[] H3; delete[] deltax; delete[] H3t; delete[] H3temp1;
	verify();
	dOut1[0] = g[0] * 180 / PI; dOut1[1] = g[1] * 180 / PI; dOut1[2] = g[2] * 180 / PI;
	dOut1[3] = g[3] * 180 / PI; dOut1[4] = g[4] * 180 / PI; dOut1[5] = g[5] * 180 / PI;
	dOut2[0] = tx; dOut2[1] = ty; dOut2[2] = tz;
	printdh();
	return 0;
}