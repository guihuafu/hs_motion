//
// Created by Admin on 2024/6/3.
//

#include "BezierCurve.h"

BezierCurve::BezierCurve()
{

}

BezierCurve::BezierCurve( double (*arr)[6]) {
    init(arr);
}

void BezierCurve::init( double (*arr)[6]) {
    // 设置控制点
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < PNUM; j++) {
            m_controlPoints[i][j] = arr[i][j];
        }
    }

	m_bCalcLengthFlag = false;
	memset(m_dLengthBuff,0,sizeof(m_dLengthBuff));

    // 计算位移长度
    m_length = calcSimpsonLen(0, 1, DIVSIZE);

	m_bCalcLengthFlag = true;

    // 计算最大曲率点的u值
    // double N = 20;
    //double kmax = 0;
    //for (int i = 0; i < N; i++) {
    //    double u = i / N;
    //    double k = calcCurature(u);
    //    if (k > kmax) {
    //        kmax = k;
    //        m_kmaxU = u;
    //    }
    //}
}

void BezierCurve::calcBezierPos( double u, double *pos)  {
    double dUR = 1 - u;
    for(int i = 0; i < PNUM; i++) {
        pos[i] = pow(dUR,5) * m_controlPoints[0][i] + \
                 5 * u * pow(dUR,4) * m_controlPoints[1][i] + \
			     10 * pow(u,2) * pow(dUR,3) * m_controlPoints[2][i] + \
                 10 * pow(u,3) * pow(dUR,2) * m_controlPoints[3][i] + \
			     5 * pow(u,4) * dUR * m_controlPoints[4][i] + \
                 pow(u,5) * m_controlPoints[5][i];
    }
}

void BezierCurve::calcBezierPrime( double u, double *prime)  {
    double dUR = 1 - u;
	//for(int i = 0; i < PNUM; i++) 
	//{
	//	prime[i] = 5 * pow(dUR,4) * (m_controlPoints[1][i] - m_controlPoints[0][i]) + \
	//		20 * u * pow(dUR,3) * (m_controlPoints[2][i] - m_controlPoints[1][i]) + \
	//		30 * pow(u,2) * pow(dUR,2) * (m_controlPoints[3][i] - m_controlPoints[2][i]) + \
	//		20 * pow(u,3) * dUR * (m_controlPoints[4][i] - m_controlPoints[3][i]) + \
	//		5 * pow(u,4) * (m_controlPoints[5][i] - m_controlPoints[4][i]);
	//}

	//耗时优化，提前计算系数
	double dUR2 = dUR*dUR;
	double dUR3 = dUR*dUR2;
	double dUR4 = dUR2*dUR2;

	double dU2 = u*u;
	double dU3 = u*dU2;
	double dU4 = dU2*dU2;

	double dK1 = 5 * dUR4;
	double dK2 = 20 * u * dUR3;
	double dK3 = 30 * dU2 * dUR2;
	double dK4 = 20 * dU3 * dUR;
	double dK5 = 5 * dU4;

	for(int i = 0; i < PNUM; i++) 
	{
		prime[i] = dK1 * (m_controlPoints[1][i] - m_controlPoints[0][i]) + \
			dK2 * (m_controlPoints[2][i] - m_controlPoints[1][i]) + \
			dK3 * (m_controlPoints[3][i] - m_controlPoints[2][i]) + \
			dK4 * (m_controlPoints[4][i] - m_controlPoints[3][i]) + \
			dK5 * (m_controlPoints[5][i] - m_controlPoints[4][i]);
	}
}

void BezierCurve::calcBezierPrime2( double u, double *prime2)  {
    double dUR = 1 - u;
    for(int i = 0; i < PNUM; i++) {
        prime2[i] = 20 * pow(dUR,3) * (m_controlPoints[2][i] - 2 * m_controlPoints[1][i] + m_controlPoints[0][i]) + \
                    60 * u * pow(dUR,2) * (m_controlPoints[3][i] - 2 * m_controlPoints[2][i] + m_controlPoints[1][i]) + \
                    60 * pow(u,2) * dUR * (m_controlPoints[4][i] - 2 * m_controlPoints[3][i] + m_controlPoints[2][i]) + \
                    20 * pow(u,3) * (m_controlPoints[5][i] - 2 * m_controlPoints[4][i] + m_controlPoints[3][i]);
    }
}

double BezierCurve::calcSimpsonLen( double a,  double b,  int n) {
    // 计算速度
    //auto f = [this](double u) -> double {
    //    double prime[6] = {0};
    //    calcBezierPrime(u, prime);
    //    double dv = 0;
    //    for (int i = 0; i < 3; i++)
    //        dv += (prime[i] * prime[i]);
    //    return sqrt(dv);
    //};

    // Simpson积分公式计算长度
    double h = (b - a) / n;
    double s1 = fcalc(a + h / 2);
    double s2 = 0;
    for(int i = 1; i < n; i++) {
        s1 += fcalc(a + (i * h) + (h / 2));
        s2 += fcalc(a + (i * h));
    }
    return h * (fcalc(a) + 4 * s1 + 2 * s2 + fcalc(b)) / 6;
}

double BezierCurve::fcalc( double u) 
{
	double dL = 0;
	if(!m_bCalcLengthFlag)
	{
		double prime[6] = {0};
		calcBezierPrime(u, prime);
		double dv = 0;
		for (int i = 0; i < 3; i++)
			dv += (prime[i] * prime[i]);
		dL = sqrt(dv);

		double dU = u*2*DIVSIZE;
		int iIndex = (int)dU;
		if(dU - iIndex > 0.5)
			iIndex++;

		m_dLengthBuff[iIndex] = dL;
	}
	else
	{
		double dU = u*2*DIVSIZE;
		int iIndex = (int)dU;
		if(dU - iIndex > 0.5)
			iIndex++;

		dL = m_dLengthBuff[iIndex];
	}
	return dL;
}

double BezierCurve::calcDeltaNxtU( double ds,  double u)  {
     int max_loop_cnt = 20;
     double l = calcSimpsonLen(0, u, 100) + ds;

    // lc + ds 要保证大于0
    if (l < 0)
        return .0;

    // 二分法查找u值
    double m = (ds > 0)? u : 0;
    double n = (ds > 0)? 1 : u;
    double k = (m + n) / 2;
    for (int i = 0; i < max_loop_cnt; i++) {
        double ln = calcSimpsonLen(0, k, 100);
        if (fabs(ln - l) < Eps)
            break;
        if (ln > l)
            n = k;
        else
            m = k;
        k = (m + n) / 2;
    }
    return k;
}

void BezierCurve::calcPosByLenght( double dL,double* pos) {

    int max_loop_cnt = 20;
    double l = dL;
    double dU = 0;

	double dErrorLimit = 0.01;
	double dDivCnt = 20;

    // lc + ds 要保证大于0
    if (l < 0)
        dU = 0;
    else
    {
        // 二分法查找u值
        double m = 0;
        double n = 1;
        double k = (m + n) / 2;
        for (int i = 0; i < max_loop_cnt; i++) 
		{
            double ln = calcSimpsonLen(0, k, dDivCnt);
            if (fabs(ln - l) < dErrorLimit)
                break;
            if (ln > l)
                n = k;
            else
                m = k;
            k = (m + n) / 2;
        }

        dU = k;
    }

    calcBezierPos(dU,pos);
}

double BezierCurve::calcCurature( double u)  {
    double prime[6] = {0}, prime2[6] = {0};
    calcBezierPrime(u, prime);
    calcBezierPrime2(u, prime2);

    // 如果一阶导数全部为0，则曲率为0
    if (fabs(prime[0]) < Eps && fabs(prime[1]) < Eps && fabs(prime[2]) < Eps) {
        return .0;
    }

    // 计算曲率
    double dx = prime[0], dy = prime[1], dz = prime[2];
    double ddx = prime2[0], ddy = prime2[1], ddz = prime2[2];
    double numerator = sqrt(pow(dy * ddz - dz * ddy, 2) + \
                               pow(dz * ddx - dx * ddz, 2) + \
                               pow(dx * ddy - dy * ddx, 2));
    double denominator = pow(sqrt(dx * dx + dy * dy + dz * dz), 3);
    return numerator / denominator;
}

double BezierCurve::getLength()  {
    return m_length;
}
