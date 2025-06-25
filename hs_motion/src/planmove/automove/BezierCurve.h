//
// Created by Admin on 2024/6/3.
//

#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H
#include <cmath>
#include <string>
#include <cstring>

#define Eps (1e-6)
#define PNUM	3
#define DIVSIZE 50


class BezierCurve {
public:
    BezierCurve();
    BezierCurve( double (*arr)[6]);

    void init( double (*arr)[6]);
    /**
    * @brief ����Bezier������uֵ���λ��
    * @param u: ��Ӧuֵ��0 \<= u \<= 1
    * @param pos: ��Ӧλ��
    * */
    void calcBezierPos( double u, double* pos) ;

    /**
    * @brief ����Bezier������uֵ���һ�׵���
    * @param u: ��Ӧuֵ��0 \<= u \<= 1
    * @param prime: ��Ӧһ�׵���ֵ
    * */
    void calcBezierPrime( double u, double* prime) ;

    /**
    * @brief ����Bezier������uֵ��Ķ��׵���
    * @param u: ��Ӧuֵ��0 \<= u \<= 1
    * @param prime2: ��Ӧ���׵���ֵ
    * */
    void calcBezierPrime2( double u, double* prime2) ;

    /**
    * @brief ����Bezier��������uֵ����ĳ���
    * @param a, b: ��Ӧuֵ���䣬a \<= u \<= b
    * @param n: ���ָ���
    * @return ��Ӧ���߳���
    * */
    double calcSimpsonLen( double a,  double b,  int n) ;

    /**
    * @brief ����Bezier������ǰ�ƶ�ds������uֵ
    * @param ds: ��Ӧλ��ֵ
    * @param u: ��ǰ������uֵ
    * @return ��Ӧuֵ
    * */
    double calcDeltaNxtU( double ds,  double u) ;

     /**
    * @brief ����Bezier������ǰ�ƶ�ds������uֵ
    * @param ds: ��Ӧλ��ֵ
    * @param u: ��ǰ������uֵ
    * @return ��Ӧuֵ
    * */
    void calcPosByLenght( double dL,double* pos) ;

    /**
    * @brief ����Bezier������uֵ�������
    * @param u: ��Ӧuֵ
    * @return ��Ӧ����ֵ
    * */
    double calcCurature( double u) ;

    /**
    * @brief ��ȡBezier����λ�Ƴ���
    * @return λ�Ƴ���
    * */
    double getLength() ;

private:
    double m_controlPoints[6][6];       // ���Ƶ�
    double m_length;                    // Bezier����λ�Ƴ���
    double m_kmaxU;                     // Bezier���������������Ӧ��uֵ

	double fcalc( double u);
	bool m_bCalcLengthFlag;				// ���㳤�ȱ�ʶ���Ż���ʱ
	double m_dLengthBuff[DIVSIZE*2 + 1];
};


#endif //BEZIERCURVE_H
