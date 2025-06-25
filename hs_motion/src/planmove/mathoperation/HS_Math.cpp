/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Math.cpp
* 摘    要：数学库计算

* 当前版本：1.0
* 作    者：cyh
* 完成日期：2017-7-12
*			
*/
#include "HS_Math.h"

//#define DEF_EIGEN_CAL

/* 使用eigen 矩阵运算*/
//#include <Eigen/Dense>

/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Math::HS_Math(void)
{
}
HS_Math::~HS_Math(void)
{
}
/************************************************
函数功能：计算矩阵的模长
参    数：iRow iCol pdMatrix---输入矩阵  wantCol,想计算的列数
返 回 值：模长
*************************************************/
double HS_Math::Matrix_ColNorm(int iRow, int iCol, int wantCol, const double *pdMatrix)
{
	int i, j;
	if (iRow <= 0 || iRow <= 0)
		return 0;
	double dtmp = 0.0;
	for (i = 0; i < iRow; i++)
	{
		dtmp = dtmp + pow(pdMatrix[i*iCol + wantCol], 2);
	}
	return sqrt(dtmp);
}
/***************************************************************
函数功能：所取块的矩阵
参    数：x与y分别是块的左上角元素在矩阵中的坐标
		  r与c分别为所取块的行数与列数,beBlocked待分矩阵
		  ab为行列，m接收
返 回 值：无
***************************************************************/
void HS_Math::Block(int x, int y, unsigned int r, unsigned int c, double *beBlocked, int a, int b, double *m)
{
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			m[i * c + j] = beBlocked[(i + x)*b + j + y];
		}
	}
}
/************************************************
函数功能：矩阵乘法
参    数：iRowA iColA pdMatrixA---输入矩阵
		 pdMatrixB--输入矩阵
返 回 值：pdMatrixC--输出矩阵
*************************************************/
void HS_Math::Matrix_Multi(int iRowA,int iColA,int iColB,const double *pdMatrixA,const double *pdMatrixB,double *pdMatrixC)
{
	int i,j,k;
	double dSum;							//中间变量

	//检查输入参数是否有误
	if(iRowA<=0 || iColA<=0 || iColB<=0)
		return;

#ifdef DEF_EIGEN_CAL 

	double dTemp[36] = {0};
	for(i=0;i<iRowA;i++)
	{
		for(j=0;j<iColB;j++)
		{
			/*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
			dSum = 0.0;
			for(k=0;k<iColA;k++)
				dSum += pdMatrixA[i*iColA+k]*pdMatrixB[k*iColB+j];
			dTemp[i*iColB+j] = dSum;
		}
	}

	//检查输入参数是否有误，要求：1.必须是方阵  2.阶数必须是3~6
	if(iRowA == iColA&&iColA == iColB&&iRowA >= 3&&iRowA <= 6)
	{
		if(iRowA == 3){
			Eigen::Matrix3d aa;
			Eigen::Matrix3d bb;
			Eigen::Matrix3d cc;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iRowA * iColA); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iRowA * iColA); 
			cc.noalias() = bb * aa;
			memcpy(pdMatrixC, cc.data(), sizeof(double) * iRowA * iColA);


		}else if(iRowA == 4){
			Eigen::Matrix4d aa;
			Eigen::Matrix4d bb;
			Eigen::Matrix4d cc;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iRowA * iColA); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iRowA * iColA); 
			cc.noalias() = bb * aa;
			memcpy(pdMatrixC, cc.data(), sizeof(double) * iRowA * iColA);


		}else if(iRowA == 5){
			Eigen::Matrix<double, 5, 5> aa;
			Eigen::Matrix<double, 5, 5> bb;
			Eigen::Matrix<double, 5, 5> cc;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iRowA * iColA); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iRowA * iColA); 
			cc.noalias() = bb * aa;
			memcpy(pdMatrixC, cc.data(), sizeof(double) * iRowA * iColA);

		}else if(iRowA == 6){
			Eigen::Matrix<double, 6, 6> aa;
			Eigen::Matrix<double, 6, 6> bb;
			Eigen::Matrix<double, 6, 6> cc;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iRowA * iColA); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iRowA * iColA); 
			cc.noalias() = aa * bb;
			memcpy(pdMatrixC, cc.data(), sizeof(double) * iRowA * iColA);

		}
	}
	else
	{
		for(i=0;i<iRowA;i++)
		{
			for(j=0;j<iColB;j++)
			{
				/*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
				dSum = 0.0;
				for(k=0;k<iColA;k++)
					dSum += pdMatrixA[i*iColA+k]*pdMatrixB[k*iColB+j];
				pdMatrixC[i*iColB+j] = dSum;
			}
		}
	}
#else

	/*嵌套循环计算结果矩阵（m*p）的每个元素*/
	for(i=0;i<iRowA;i++)
	{
		for(j=0;j<iColB;j++)
		{
			/*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
			dSum = 0.0;
			for(k=0;k<iColA;k++)
				dSum += pdMatrixA[i*iColA+k]*pdMatrixB[k*iColB+j];
			pdMatrixC[i*iColB+j] = dSum;
		}
	}
#endif
}
/************************************************
函数功能：矩阵乘法 矩阵的维数为A，乘法的维数为B
参    数：iRowA iColA pdMatrixA---输入矩阵
		 pdMatrixB--输入矩阵
返 回 值：pdMatrixC--输出矩阵
*************************************************/
void HS_Math::Matrix_Multi(int iA,int iB,const double *pdMatrixA,const double *pdMatrixB,double *pdMatrixC)
{
	int i,j,k;
	double dSum;							//中间变量

	/*嵌套循环计算结果矩阵（m*p）的每个元素*/
	for(i=0;i<iB;i++)
	{
		for(j=0;j<iB;j++)
		{
			/*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
			dSum = 0.0;
			for(k=0;k<iB;k++)
				dSum += pdMatrixA[i*iA+k]*pdMatrixB[k*iA+j];
			pdMatrixC[i*iA+j] = dSum;
		}
	}
}
/***************************************************************************************************
*函数名：Matrix_AddOrSub
*编写者：黄键
*日期：2017.3.16
*功能：矩阵运算--> C = iARatio*A + iBRatio*B
*输入参数：
*iRow-------------矩阵行数
*iCol-------------矩阵列数
*iARatio----------矩阵A数乘系数
*iBRatio----------矩阵B数乘系数
*pdMatrisA--------加数矩阵A
*pdMatrisB--------加数矩阵B
*pdMatrisC--------结果矩阵C
*输出参数
*返回0，正常计算。返回-1，输入行列数有误。
****************************************************************************************************/
int HS_Math::Matrix_AddOrSub(int iRow,int iCol,int iARatio,int iBRatio,double *pdMatrisA,double *pdMatrisB,double *pdMatrisC)
{
	int i,j;				//循环计数器
	//检查输入参数是否有误
	if(iRow<=0 || iCol<=0)
		return -1;

	//执行计算
	for(i=0;i<iRow;i++)
	{
		for(j=0;j<iCol;j++)
		{
			pdMatrisC[i*iCol+j] = iARatio*pdMatrisA[i*iCol+j] + iBRatio*pdMatrisB[i*iCol+j];
		}
	}

	//输出
	return 0;
}
/************************************************
函数功能：齐次变换矩阵求逆简化算法
参    数：pdMatrixA---输入矩阵
返 回 值：pdMatrixB---逆矩阵
*************************************************/
bool HS_Math::Matrix_Inverse_H(const double *pdMatrixA,double *pdMatrixB)
{
	//姿态矩阵转置
	int iOrder = 4;
	double dMTMatrix[16] = {0};
	for(int i = 0;i < iOrder;i++)
	{
		for(int j = 0;j < iOrder;j++)
		{
			dMTMatrix[j*iOrder +i] = pdMatrixA[i*iOrder+j];
		}
	}

	double dA[9] = {0};
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			dA[i*3 + j] = dMTMatrix[i*4 + j];
		}

	double dP[3] = {pdMatrixA[3],pdMatrixA[7],pdMatrixA[11]};

	double dPN[3] = {0};

	Matrix_Multi(3,3,1,dA,dP,dPN);

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			pdMatrixB[i*4 + j] = dMTMatrix[i*4 + j];
		}	
	
	pdMatrixB[3] = -dPN[0];
	pdMatrixB[7] = -dPN[1];
	pdMatrixB[11] = -dPN[2];

	pdMatrixB[12] = 0;
	pdMatrixB[13] = 0;
	pdMatrixB[14] = 0;
	pdMatrixB[15] = 1;

	return true;
}

/************************************************
函数功能：求解绕Z轴旋转角度对应的矩阵坐标
参    数：dAngle---旋转角度值
		 dMPos----矩阵坐标
返 回 值：错误码
*************************************************/
void HS_Math::HS_RotZMPos(double dAngle,double dMPos[4][4])
{
	dMPos[0][0] = 0;dMPos[0][1] = 0;dMPos[0][2] = 0;dMPos[0][3] = 0;
	dMPos[1][0] = 0;dMPos[1][1] = 0;dMPos[1][2] = 0;dMPos[1][3] = 0;
	dMPos[2][0] = 0;dMPos[2][1] = 0;dMPos[2][2] = 1;dMPos[2][3] = 0;
	dMPos[3][0] = 0;dMPos[3][1] = 0;dMPos[3][2] = 0;dMPos[3][3] = 1;
	dMPos[0][0] = cos(angle2Rad(dAngle));
	dMPos[0][1] = -sin(angle2Rad(dAngle));
	dMPos[1][0] = sin(angle2Rad(dAngle));
	dMPos[1][1] = cos(angle2Rad(dAngle));
}
/************************************************
函数功能：求解绕Y轴旋转角度对应的矩阵坐标
参    数：dAngle---旋转角度值
		 dMPos----矩阵坐标
返 回 值：错误码
*************************************************/
void HS_Math::HS_RotYMPos(double dAngle,double dMPos[4][4])
{
	dMPos[0][0] = 0;dMPos[0][1] = 0;dMPos[0][2] = 0;dMPos[0][3] = 0;
	dMPos[1][0] = 0;dMPos[1][1] = 1;dMPos[1][2] = 0;dMPos[1][3] = 0;
	dMPos[2][0] = 0;dMPos[2][1] = 0;dMPos[2][2] = 0;dMPos[2][3] = 0;
	dMPos[3][0] = 0;dMPos[3][1] = 0;dMPos[3][2] = 0;dMPos[3][3] = 1;
	dMPos[0][0] = cos(angle2Rad(dAngle));
	dMPos[2][0] = -sin(angle2Rad(dAngle));
	dMPos[0][2] = sin(angle2Rad(dAngle));
	dMPos[2][2] = cos(angle2Rad(dAngle));
}
/************************************************
函数功能：求解绕X轴旋转角度对应的矩阵坐标
参    数：dAngle---旋转角度值
		 dMPos----矩阵坐标
返 回 值：错误码
*************************************************/
void HS_Math::HS_RotXMPos(double dAngle,double dMPos[4][4])
{
	dMPos[0][0] = 1;dMPos[0][1] = 0;dMPos[0][2] = 0;dMPos[0][3] = 0;
	dMPos[1][0] = 0;dMPos[1][1] = 0;dMPos[1][2] = 0;dMPos[1][3] = 0;
	dMPos[2][0] = 0;dMPos[2][1] = 0;dMPos[2][2] = 0;dMPos[2][3] = 0;
	dMPos[3][0] = 0;dMPos[3][1] = 0;dMPos[3][2] = 0;dMPos[3][3] = 1;
	dMPos[1][1] = cos(angle2Rad(dAngle));
	dMPos[2][1] = sin(angle2Rad(dAngle));
	dMPos[1][2] = -sin(angle2Rad(dAngle));
	dMPos[2][2] = cos(angle2Rad(dAngle));
}
/************************************************
函数功能：矩阵求逆
参    数：pdMatrixA---输入矩阵
		 iOrder------阶数
返 回 值：pdMatrixB---逆矩阵
*************************************************/
bool HS_Math::Matrix_Inverse(int iOrder,const double *pdMatrixA,double *pdMatrixB)
{

#ifdef DEF_EIGEN_CAL /* 利用eigen矩阵运算 */

	if(iOrder >= 3&&iOrder <= 6)
	{
		if(iOrder == 3){
			Eigen::Matrix3d aa;
			Eigen::Matrix3d bb;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iOrder * iOrder); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iOrder * iOrder);
			bb = aa.inverse();
			memcpy(pdMatrixB, bb.data(), sizeof(double) * iOrder * iOrder);


		}else if(iOrder == 4){
			Eigen::Matrix4d aa;
			Eigen::Matrix4d bb;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iOrder * iOrder); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iOrder * iOrder);
			bb = aa.inverse();
			memcpy(pdMatrixB, bb.data(), sizeof(double) * iOrder * iOrder);


		}else if(iOrder == 5){
			Eigen::Matrix<double, 5, 5> aa;
			Eigen::Matrix<double, 5, 5> bb;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iOrder * iOrder); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iOrder * iOrder);
			bb = aa.inverse();
			memcpy(pdMatrixB, bb.data(), sizeof(double) * iOrder * iOrder);


		}else if(iOrder == 6){
			Eigen::Matrix<double, 6, 6> aa;
			Eigen::Matrix<double, 6, 6> bb;
			memcpy(aa.data(), pdMatrixA, sizeof(double) * iOrder * iOrder); 
			memcpy(bb.data(), pdMatrixB, sizeof(double) * iOrder * iOrder);
			bb = aa.inverse();
			memcpy(pdMatrixB, bb.data(), sizeof(double) * iOrder * iOrder);
		}
	}
	else
	{
		int i, j, k;
		double dMax, dTemp;				//中间变量
		double dT[32][32];              //临时矩阵t
		//防止输入参数有误
		if (iOrder <= 0)
			return false;

		//将A矩阵存放在临时矩阵t[n][n]中
		for (i = 0; i < iOrder; i++)
		{
			for (j = 0; j < iOrder; j++)
			{
				dT[i][j] = pdMatrixA[i*iOrder+j];
			}
		}

		//初始化B矩阵为单位阵
		for (i = 0; i < iOrder; i++)
		{
			for (j = 0; j < iOrder; j++)
			{
				pdMatrixB[i*iOrder+j] = (i == j) ? 1.0 : 0;
			}
		}
		for (i = 0; i < iOrder; i++)
		{
			//寻找主元
			dMax = dT[i][i];
			k = i;
			for (j = i+1; j < iOrder; j++)
			{
				if (fabs(dT[j][i]) > fabs(dMax))
				{
					dMax = dT[j][i];
					k = j;
				}
			}
			//如果主元所在行不是第i行，进行行交换
			if (k != i)
			{
				for (j = 0; j < iOrder; j++)
				{
					dTemp = dT[i][j];
					dT[i][j] = dT[k][j];
					dT[k][j] = dTemp;
					//B伴随交换
					dTemp = pdMatrixB[i*iOrder+j];
					pdMatrixB[i*iOrder+j] = pdMatrixB[k*iOrder+j];
					pdMatrixB[k*iOrder+j] = dTemp;
				}
			}
			//判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵
			if (fabs(dT[i][i]) < Eps)
			{
				return false;
			}
			//消去A的第i列除去i行以外的各行元素
			dTemp = dT[i][i];
			for (j = 0; j < iOrder; j++)
			{
				dT[i][j] = dT[i][j]/dTemp;						//主对角线上的元素变为1
				pdMatrixB[i*iOrder+j] = pdMatrixB[i*iOrder+j]/dTemp;	//伴随计算
			}
			for (j = 0; j < iOrder; j++)							//第0行->第n行
			{
				if (j != i)											//不是第i行
				{
					dTemp = dT[j][i];
					for (k = 0; k < iOrder; k++)					//第j行元素 - i行元素*j列i行元素
					{
						dT[j][k] = dT[j][k] - dT[i][k]*dTemp;
						pdMatrixB[j*iOrder+k] -= pdMatrixB[i*iOrder+k]*dTemp;
					}
				}
			}
		}
	}
#else
	int i, j, k;
	double dMax, dTemp;				//中间变量
	double dT[32][32];              //临时矩阵t
	//防止输入参数有误
	if (iOrder <= 0)
		return false;

	//将A矩阵存放在临时矩阵t[n][n]中
	for (i = 0; i < iOrder; i++)
	{
		for (j = 0; j < iOrder; j++)
		{
			dT[i][j] = pdMatrixA[i*iOrder+j];
		}
	}

	//初始化B矩阵为单位阵
	for (i = 0; i < iOrder; i++)
	{
		for (j = 0; j < iOrder; j++)
		{
			pdMatrixB[i*iOrder+j] = (i == j) ? 1.0 : 0;
		}
	}
	for (i = 0; i < iOrder; i++)
	{
		//寻找主元
		dMax = dT[i][i];
		k = i;
		for (j = i+1; j < iOrder; j++)
		{
			if (fabs(dT[j][i]) > fabs(dMax))
			{
				dMax = dT[j][i];
				k = j;
			}
		}
		//如果主元所在行不是第i行，进行行交换
		if (k != i)
		{
			for (j = 0; j < iOrder; j++)
			{
				dTemp = dT[i][j];
				dT[i][j] = dT[k][j];
				dT[k][j] = dTemp;
				//B伴随交换
				dTemp = pdMatrixB[i*iOrder+j];
				pdMatrixB[i*iOrder+j] = pdMatrixB[k*iOrder+j];
				pdMatrixB[k*iOrder+j] = dTemp;
			}
		}
		//判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵
		if (fabs(dT[i][i]) < Eps)
		{
			return false;
		}
		//消去A的第i列除去i行以外的各行元素
		dTemp = dT[i][i];
		for (j = 0; j < iOrder; j++)
		{
			dT[i][j] = dT[i][j]/dTemp;						//主对角线上的元素变为1
			pdMatrixB[i*iOrder+j] = pdMatrixB[i*iOrder+j]/dTemp;	//伴随计算
		}
		for (j = 0; j < iOrder; j++)							//第0行->第n行
		{
			if (j != i)											//不是第i行
			{
				dTemp = dT[j][i];
				for (k = 0; k < iOrder; k++)					//第j行元素 - i行元素*j列i行元素
				{
					dT[j][k] = dT[j][k] - dT[i][k]*dTemp;
					pdMatrixB[j*iOrder+k] -= pdMatrixB[i*iOrder+k]*dTemp;
				}
			}
		}
	}
#endif
	return true;
}
/************************************************
函数功能：矩阵转置
参    数：pdMatrixA---输入矩阵pdMatrixA[iRowA][iColA]
返 回 值：pdMatrixB---逆矩阵pdMatrixB[iColA][iRowA]
*************************************************/
void HS_Math::Matrix_Transpose(int iRowA,int iColA,const double *pdMatrixA,double *pdMatrixB)
{
	int i,j;

	//检查输入参数是否有误
	if(iRowA<=0 || iColA<=0)
		return ;

	//矩阵转置
	for(i=0;i<iRowA;i++)
	{
		for(j=0;j<iColA;j++)
		{
			pdMatrixB[j*iRowA +i] = pdMatrixA[i*iColA+j];
		}
	}
}
/************************************************
函数功能：生成单位矩阵
参    数：iRow iCol pdMatrix---输入矩阵		 
返 回 值：模长
*************************************************/
void HS_Math::Matrix_Eye(int iRow, double *pdMatrix)
{
	for(int i = 0;i < iRow;i++)
	{
		for(int j = 0;j < iRow;j++)
		{
			if(i == j)
				pdMatrix[i*iRow + j] = 1.0;
			else
				pdMatrix[i*iRow + j] = 0;
		}
	}
}
/************************************************
函数功能：计算矩阵的模长
参    数：iRow iCol pdMatrix---输入矩阵		 
返 回 值：模长
*************************************************/
double HS_Math::Matrix_Norm(int iRow, int iCol,const double *pdMatrix)
{
	int i,j;
	if(iRow <= 0 || iRow <= 0)
		return 0;
	double dtmp=0.0;
	for( i=0;i<iRow;i++)
	{
		for( j=0;j<iCol;j++)
		{
			dtmp = dtmp+pow(pdMatrix[i*iCol+j],2);
		}
	}
	return sqrt(dtmp);
}
/************************************************
函数功能：3维向量叉乘
参    数：dA dB 输出向量		 
返 回 值：dC输出向量
*************************************************/
void HS_Math::Matrix_VecCross(const double *dA,const double *dB,double *dC)
{
	dC[0] = dA[1]*dB[2] - dA[2]*dB[1];
	dC[1] = dA[2]*dB[0] - dA[0]*dB[2];
	dC[2] = dA[0]*dB[1] - dA[1]*dB[0];
}

/************************************************
函数功能：向量归一化
参    数：向量长度iSize，向量Q
返 回 值：
*************************************************/
void HS_Math::Normalize(int iSize, double* Q)
{
	double length = 0;
	for (int i = 0; i < iSize; i++)
		length += Q[i] * Q[i];
	length = sqrt(length);
	if (length < 1e-6)
		return;
	for (int i = 0; i < iSize; i++)
		Q[i] /= length;
}
/************************************************
函数功能：3*3姿态矩阵转换为四元数
参    数：dM输入姿态矩阵		 
返 回 值：dQ输出四元数w,x,y,z
*************************************************/
void HS_Math::Matrix_MToQ(const double *dM,double *dQ)
{
	double dT[3][3];
	int i,j;
	for(i =0;i<3;i++)
		for(j =0;j<3;j++)
			dT[i][j] = dM[3*i+j];

	dQ[0] = sqrt(fabs(1+dT[0][0]+dT[1][1]+dT[2][2]))*0.5;
	dQ[1] = sqrt(fabs(1+dT[0][0]-dT[1][1]-dT[2][2]))*0.5;
	dQ[2] = sqrt(fabs(1-dT[0][0]+dT[1][1]-dT[2][2]))*0.5;
	dQ[3] = sqrt(fabs(1-dT[0][0]-dT[1][1]+dT[2][2]))*0.5;
	if(dT[1][2] < dT[2][1])
		dQ[1] = -dQ[1];
	if(dT[2][0] < dT[0][2])
		dQ[2] = -dQ[2];
	if(dT[0][1] < dT[1][0])
		dQ[3] = -dQ[3];
	//归一化
	//double d = sqrt(Matrix_QMul(dQ,dQ));
	//dQ[0] /= d;
	//dQ[1] /= d;
	//dQ[2] /= d;
	//dQ[3] /= d;
}
/************************************************
函数功能：四元数内积
参    数：	 
返 回 值：
*************************************************/
double HS_Math::Matrix_QMul(const double *dQ1,const double *dQ2)
{
	return (dQ1[0]*dQ2[0] + dQ1[1]*dQ2[1] + dQ1[2]*dQ2[2] + dQ1[3]*dQ2[3]); 
}
/************************************************
函数功能：四元数转换为3*3姿态矩阵
参    数：dQ输出四元数w,x,y,z		 
返 回 值：dM输入姿态矩阵
*************************************************/
void HS_Math::Matrix_QToM(const double *dQ,double *dM)
{
	double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
	
	x2 = 2*dQ[1];
	y2 = 2*dQ[2];
	z2 = 2*dQ[3];

	xx = dQ[1]*x2;
	yy = dQ[2]*y2;
	zz = dQ[3]*z2;

	xy = dQ[1]*y2;
	xz = dQ[1]*z2;
	yz = dQ[2]*z2;

	wx = dQ[0]*x2;
	wy = dQ[0]*y2;
	wz = dQ[0]*z2;	

/*	dM[DH_NX] = 1.0 - (yy + zz);
	dM[DH_NY] = xy - wz;
	dM[DH_NZ] = xz + wy;

	dM[DH_OX] = xy + wz;
	dM[DH_OY] = 1.0 - (xx + zz);
	dM[DH_OZ] = yz - wx;

	dM[DH_AX] = xz - wy;
	dM[DH_AY] = yz + wx;
	dM[DH_AZ] = 1.0 - (xx + yy);*/	
}
/************************************************
函数功能：四元数转换为轴角
参    数：dQ输出四元数w,x,y,z		 
返 回 值：dM输入姿态矩阵角度，fx，fy，fz
*************************************************/
void HS_Math::Matrix_QToA(const double *dQ,double *dA)
{
	dA[0] = acos(dQ[0]);
	double dSinA = sin(dA[0]);
	if(fabs(dSinA) < 1e-6)
	{
		dA[1] = 0;
		dA[2] = 0;
		dA[3] = 0;
	}
	else
	{
		dA[1] = dQ[1]/dSinA;
		dA[2] = dQ[2]/dSinA;
		dA[3] = dQ[3]/dSinA;
	}
	dA[0] = dA[0]*2;
}
/************************************************
函数功能：矩阵转换为轴角
参    数：		 
返 回 值：角度，fx，fy，fz
*************************************************/
void HS_Math::Matrix_MToA(const double *dM,double *dA)
{
	double dT[3][3] = {0};
	int i,j = 0;
	for(i =0;i<3;i++)
		for(j =0;j<3;j++)
			dT[i][j] = dM[3*i+j];

	double qs = dT[0][0]+dT[1][1]+dT[2][2]+1.0;

	if(qs < -Eps)
		return;
	else
	{
        //修改位移量计算，原为1e-6，对应的位移量约为180度，但是存在截断误差，修改为1e-12
        if(qs < 1e-12)
        //if(qs < 1e-6)
            qs = 0;
		qs = sqrt(qs);
		qs =  qs/2.0;
	}
	if(qs > 1.0)
		qs = 1.0;

	double kx = dT[2][1] - dT[1][2];	// Oz - Ay
	double ky = dT[0][2] - dT[2][0];	// Ax - Nz
	double kz = dT[1][0] - dT[0][1];	// Ny - Ox
	double kx1,ky1,kz1;
	bool add = 0;
	if((dT[0][0] >= dT[1][1]) && (dT[0][0] >= dT[2][2])) 
	{
		kx1 = dT[0][0] - dT[1][1] - dT[2][2] + 1;	// Nx - Oy - Az + 1
		ky1 = dT[1][0] + dT[0][1];			// Ny + Ox
		kz1 = dT[2][0] + dT[0][2];			// Nz + Ax
		add = (kx >= 0);
	}
	else if (dT[1][1] >= dT[2][2])
	{
		kx1 = dT[1][0] + dT[0][1];			// Ny + Ox
		ky1 = dT[1][1] - dT[0][0] - dT[2][2] + 1;	// Oy - Nx - Az + 1
		kz1 = dT[2][1] + dT[1][2];			// Oz + Ay
		add = (ky >= 0);
	}
	else
	{
		kx1 = dT[2][0] + dT[0][2];			// Nz + Ax
		ky1 = dT[2][1] + dT[1][2];			// Oz + Ay
		kz1 = dT[2][2] - dT[0][0] - dT[1][1] + 1;	// Az - Nx - Oy + 1
		add = (kz >= 0);
	}

	if(add) 
	{
		kx = kx + kx1;
		ky = ky + ky1;
		kz = kz + kz1;
	}
	else
	{
		kx = kx - kx1;
		ky = ky - ky1;
		kz = kz - kz1;
	}
	double norm = sqrt(kx*kx + ky*ky + kz*kz)*100;
	if(norm > Eps)
	{
		dA[1] = kx*100/norm;
		dA[2] = ky*100/norm;
		dA[3] = kz*100/norm;
	}
	else
	{
		dA[1] = 0;
		dA[2] = 0;
		dA[3] = 0;
	}
	dA[0] = 2*acos(qs);


	//欧拉角360修改：如果DA=0,其他参数不能全为0
	if(dA[0] < Eps)
	{
		dA[0] = 0;
		dA[1] = 0;
		dA[2] = 0;
		dA[3] = 1;
	}

	//if(dA[0] > PI/2)
	//{
	//	dA[0] = PI - dA[0];
	//	dA[1] = -dA[1];
	//	dA[2] = -dA[2];
	//	dA[3] = -dA[3];
	//}
}
/************************************************
函数功能：轴角转换为姿态矩阵
参    数：角度，fx，fy，fz	 
返 回 值：
*************************************************/
//void HS_Math::Matrix_AToM(const double *dQ,double *dM)
//{
//	double cth = cos(dQ[0]);
//	double sth = sin(dQ[0]);
//
//	double vth = (1 - cth);
//	double kx = dQ[1]; 
//	double ky = dQ[2]; 
//	double kz = dQ[3];
//
//	dM[0] = kx*kx*vth+cth;
//	dM[1] = ky*kx*vth-kz*sth;
//	dM[2] = kz*kx*vth+ky*sth;
//	dM[3] = kx*ky*vth+kz*sth;
//	dM[4] = ky*ky*vth+cth;
//	dM[5] = kz*ky*vth-kx*sth;
//	dM[6] = kx*kz*vth-ky*sth;
//	dM[7] = ky*kz*vth+kx*sth;
//	dM[8] = kz*kz*vth+cth;
//}

void HS_Math::Matrix_AToM(const double *dQ,double dM[4][4])
{
	double cth = cos(dQ[0]);
	double sth = sin(dQ[0]);

	double vth = (1 - cth);
	double kx = dQ[1]; 
	double ky = dQ[2]; 
	double kz = dQ[3];
    if(fabs(kx) < Eps&&fabs(ky) < Eps&&fabs(kz) < Eps)
    {
        cth = 1;
        vth = 0;
    }

	dM[0][0] = kx*kx*vth+cth;
	dM[0][1] = ky*kx*vth-kz*sth;
	dM[0][2] = kz*kx*vth+ky*sth;
	dM[0][3] = 0;
	dM[1][0] = kx*ky*vth+kz*sth;
	dM[1][1] = ky*ky*vth+cth;
	dM[1][2] = kz*ky*vth-kx*sth;
	dM[1][3] = 0;
	dM[2][0] = kx*kz*vth-ky*sth;
	dM[2][1] = ky*kz*vth+kx*sth;
	dM[2][2] = kz*kz*vth+cth;
	dM[2][3] = 0;
	dM[3][0] = 0;
	dM[3][1] = 0;
	dM[3][2] = 0;
	dM[3][3] = 1;
}

void HS_Math::Matrix_transl(double x, double y, double z, double (*pdMatrix)[4])
{
	pdMatrix[0][0] = 1;
	pdMatrix[0][1] = 0;
	pdMatrix[0][2] = 0;
	pdMatrix[0][3] = x;
	pdMatrix[1][0] = 0;
	pdMatrix[1][1] = 1;
	pdMatrix[1][2] = 0;
	pdMatrix[1][3] = y;
	pdMatrix[2][0] = 0;
	pdMatrix[2][1] = 0;
	pdMatrix[2][2] = 1;
	pdMatrix[2][3] = z;
	pdMatrix[3][0] = 0;
	pdMatrix[3][1] = 0;
	pdMatrix[3][2] = 0;
	pdMatrix[3][3] = 1;
}

//bool HS_Math::Matrix_EulRot(double deg1, double deg2, double deg3, SpinSequence seq, double (*pdMatrix)[4])
//{
//	double IdentityMatrix[4][4] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
//	double tmpMat[4][4];
//	double CurRotMat[4][4];
//
//	for (int i = 0; i < 4; i++)
//	{
//		for (int j = 0; j < 4; j++)
//		{
//			tmpMat[i][j] = IdentityMatrix[i][j];
//			CurRotMat[i][j] = IdentityMatrix[i][j];
//			pdMatrix[i][j] = IdentityMatrix[i][j];
//		}
//	}
//
//	double deg;
//	int adjustedseq = seq;
//	if (seq & 0x40) //if the Eule Rot is Absolute, Adjust angle and spin axis
//	{
//		adjustedseq = ((seq & 0x0d)) + ((seq & 0x03) << 4) + ((seq & 0x30) >> 4);
//		double tempdeg = deg1;
//		deg1 = deg3;
//		deg3 = tempdeg;
//		//cout<<"adjustedseq = "<<"0x"<<hex<<adjustedseq<<endl;
//	}
//	for (int i = 0; i < 3; i++)
//	{
//		deg = deg1;
//		if (i == 1)
//			deg = deg2;
//		if (i == 2)
//			deg = deg3;
//		switch (((adjustedseq >> (2 * i)) & 0x03))
//		{
//		case 0:
//			{
//				if (fabs(deg) > EPS)
//				{
//					//cout << "x spin " << deg << " degree" << endl;
//					double cx = cos(deg * deg2rad);
//					double sx = sin(deg * deg2rad);
//					CurRotMat[1][1] = cx;
//					CurRotMat[1][2] = -sx;
//					CurRotMat[2][1] = sx;
//					CurRotMat[2][2] = cx;
//				}
//				else
//				{
//					// cout << "x still" << endl;
//				}
//				break;
//			}
//		case 1:
//			{
//				if (fabs(deg) > EPS)
//				{
//					//cout << "y spin " << deg << " degree" << endl;
//					double cy = cos(deg * deg2rad);
//					double sy = sin(deg * deg2rad);
//					CurRotMat[0][0] = cy;
//					CurRotMat[0][2] = sy;
//					CurRotMat[2][0] = -sy;
//					CurRotMat[2][2] = cy;
//				}
//				else
//				{
//					//cout << "y still" << endl;
//				}
//				break;
//			}
//		case 2:
//			{
//				if (fabs(deg) > EPS)
//				{
//					//cout << "z spin " << deg << " degree" << endl;
//					double cz = cos(deg * deg2rad);
//					double sz = sin(deg * deg2rad);
//					CurRotMat[0][0] = cz;
//					CurRotMat[0][1] = -sz;
//					CurRotMat[1][0] = sz;
//					CurRotMat[1][1] = cz;
//				}
//				else
//				{
//					// cout << "z still" << endl;
//				}
//				break;
//			}
//		default:
//			{
//				//cout << "Invalid seq" << endl;
//				return false;
//			}
//		}
//
//		double MatResult[4][4]; //intermediate result
//		Matrix_Multi(4, 4, 4, &tmpMat[0][0], &CurRotMat[0][0], &MatResult[0][0]);
//
//		for (int i = 0; i < 4; i++)
//		{
//			for (int j = 0; j < 4; j++)
//			{
//				tmpMat[i][j] = MatResult[i][j];
//				CurRotMat[i][j] = IdentityMatrix[i][j];
//			}
//		}
//		// memcpy(tmpMat, MatResult, sizeof(MatResult));
//		// memcpy(CurRotMat, IdentityMatrix, sizeof(IdentityMatrix)); //retore to previous Identity Matrix
//	}
//
//	for (int i = 0; i < 4; i++)
//	{
//		for (int j = 0; j < 4; j++)
//		{
//			pdMatrix[i][j] = tmpMat[i][j];
//			CurRotMat[i][j] = IdentityMatrix[i][j];
//		}
//	}
//	//memcpy(pdMatrix, tmpMat, sizeof(tmpMat));
//	return true;
//}