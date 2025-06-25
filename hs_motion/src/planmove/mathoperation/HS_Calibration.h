
#ifndef _HS_CALIBRATION_H
#define _HS_CALIBRATION_H

#include "HS_Kinematics.h"
#include "MathOperation.h"
#include "HS_Math.h"

class HS_Calibration:HS_Math
{
public:	
	HS_Calibration(int iGroupNum);	
	~HS_Calibration(void);
/***********工具坐标系标定***********************/
	int Tool_Six(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
				double *dPosIn4,double *dPosIn5,double *dPosIn6,double *dToolOut,double &dMaxError);

	int Tool_Four(double dError,double *dPosIn1,double *dPosIn2,double *dPosIn3,
		double *dPosIn4,double *dToolOut,double &dMaxError);

	int Tool_Mid(bool bFour,double *dPosIn1,double *dPosIn2,double *dPosIn3,
		double *dPosIn4,double *dToolOut);
/***********工件坐标系标定***********************/
	int	Work_Three(double *dPo,double *dPx,double *dPxy,double *dWorkOut);

	int Enternal_Work_Three(double *dPosIn1,double *dPosIn2,double *dPosIn3,double *dToolIn,double *dWorkOut);

/***********Scara标定***********************/
	int Scara_LeftRightZero(double dLeftJPos,double dRightJPos,double &dZeroCorrect);
	int Scara_Tool_Two(double *dPosIn1,double *dPosIn2,double *dWorkOut);
/***********变位机标定***********************/
	int GetPositionerAxisTrans(double *p1,double * p2,double * p3,double (*axisTrans)[4]);
	int Positioner(double **dPosIn,int df,double **caliRes);
	int CaliExtAxisRatio(double *dPos1,double *dPos2,double *dPos3,double *dEncoder,double &dExtAxisEncoder,double &dRatio);
/***********地轨标定***********************/
	
/***********20点法标定***********************/
	int HS_IntegratedCalibration(double (*dDHPara)[3], double (*dDataIn)[6],double *dOut1,double *dOut2);
	void buildmodel();
	void derivative();
	void jaco();
	void h_deltax1(double *H1, double *deltax);
	void h_deltax2(double *H2, double *deltax);
	void h_deltax3(double *H3, double *deltax);
	void verify();
	void Kalman(double* H_, double* deltax);
	void printdh();
	int getdatarows(double dDataIn[20][6]);
	void setdh(double dDHPara[6][3]);

private:
	HS_Kinematics *m_HS_Kinematic;
	HS_Math	m_HS_Math;

	double (*xita6)[6];//Matrix<double, Dynamic, 6> xita6;//接各点位关节角值
	double T1[16], T2[16], T3[16], T4[16], T5[16], T6[16], Tcam2base[16], Ttool[16], T06[16], T0t[16], Tcam2t[16];//Matrix4d 
	double GB[18];//Matrix<double, 6, 3> GB;
	double G1[6], G2[6], G3[6], G4[6], G5[6], G6[6];//Matrix<double, 6, 1>
	double J0t[18], J1t[18], J2t[18], J3t[18], J4t[18], J5t[18], J6t[18];//Matrix<double, 3, 6> 
	double solx1[3],solx2[6],solx3[11],solx[11];
	double aver_error, max_error;
	double g[6], e[6], a[6], d[6], f[6], aa[6], dd[6], ff[6], gg[6];
	double bx, by, bz, ba, bb, bc;
	double tx, ty, tz;
	int datarows;
};

#endif
