

#ifndef _HS_MATH_H
#define _HS_MATH_H

#include "MathOperation.h"

//HS_Math 类
class HS_Math
{
public:
	HS_Math(void);
	~HS_Math(void);
	static void Matrix_Multi(int iRowA,int iColA,int iColB,const double *pdMatrixA,const double *pdMatrixB,double *pdMatrixC);
	static void Matrix_Multi(int iA,int iB,const double *pdMatrixA,const double *pdMatrixB,double *pdMatrixC);
	static int Matrix_AddOrSub(int iRow,int iCol,int iARatio,int iBRatio,double *pdMatrisA,double *pdMatrisB,double *pdMatrisC);
	static bool Matrix_Inverse(int iOrder,const double *pdMatrixA,double *pdMatrixB);
	static void Matrix_Transpose(int iRowA,int iColA,const double *pdMatrixA,double *pdMatrixB);
	static double Matrix_Norm(int iRow, int iCol,const double *pdMatrix);
	static void Matrix_Eye(int iRow, double *pdMatrix);
	static void Matrix_VecCross(const double *dA,const double *dB,double *dC);
	static void Matrix_MToQ(const double *dM,double *dQ);
	static void Matrix_QToM(const double *dQ,double *dM);
	static double Matrix_QMul(const double *dQ1,const double *dQ2);
	static void Matrix_QToA(const double *dQ,double *dA);
	static void Matrix_MToA(const double *dM,double *dA);
	//static void Matrix_AToM(const double *dQ,double *dM);
	static void Matrix_AToM(const double *dQ,double dM[4][4]);
	static void Matrix_transl(double x, double y, double z, double (*pdMatrix)[4]);
	//static bool Matrix_EulRot(double deg1, double deg2, double deg3, SpinSequence seq, double (*pdMatrix)[4]);

	static double Matrix_ColNorm(int iRow, int iCol, int wantCol, const double *pdMatrix);
	static void Block(int x, int y, unsigned int r, unsigned int c , double *beBlocked, int a, int b, double *m);
	static void Normalize(int iSize, double* Q);

	static bool Matrix_Inverse_H(const double *pdMatrixA,double *pdMatrixB);

	static void HS_RotZMPos(double dAngle,double dMPos[4][4]);
	static void HS_RotYMPos(double dAngle,double dMPos[4][4]);
	static void HS_RotXMPos(double dAngle,double dMPos[4][4]);
};

#endif