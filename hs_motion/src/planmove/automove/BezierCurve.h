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
    * @brief 计算Bezier曲线在u值点的位姿
    * @param u: 对应u值，0 \<= u \<= 1
    * @param pos: 对应位姿
    * */
    void calcBezierPos( double u, double* pos) ;

    /**
    * @brief 计算Bezier曲线在u值点的一阶导数
    * @param u: 对应u值，0 \<= u \<= 1
    * @param prime: 对应一阶导数值
    * */
    void calcBezierPrime( double u, double* prime) ;

    /**
    * @brief 计算Bezier曲线在u值点的二阶导数
    * @param u: 对应u值，0 \<= u \<= 1
    * @param prime2: 对应二阶导数值
    * */
    void calcBezierPrime2( double u, double* prime2) ;

    /**
    * @brief 计算Bezier曲线所在u值区间的长度
    * @param a, b: 对应u值区间，a \<= u \<= b
    * @param n: 划分个数
    * @return 对应曲线长度
    * */
    double calcSimpsonLen( double a,  double b,  int n) ;

    /**
    * @brief 计算Bezier曲线往前移动ds距离后的u值
    * @param ds: 对应位移值
    * @param u: 当前点所在u值
    * @return 对应u值
    * */
    double calcDeltaNxtU( double ds,  double u) ;

     /**
    * @brief 计算Bezier曲线往前移动ds距离后的u值
    * @param ds: 对应位移值
    * @param u: 当前点所在u值
    * @return 对应u值
    * */
    void calcPosByLenght( double dL,double* pos) ;

    /**
    * @brief 计算Bezier曲线在u值点的曲率
    * @param u: 对应u值
    * @return 对应曲率值
    * */
    double calcCurature( double u) ;

    /**
    * @brief 获取Bezier曲线位移长度
    * @return 位移长度
    * */
    double getLength() ;

private:
    double m_controlPoints[6][6];       // 控制点
    double m_length;                    // Bezier曲线位移长度
    double m_kmaxU;                     // Bezier曲线曲率最大所对应的u值

	double fcalc( double u);
	bool m_bCalcLengthFlag;				// 计算长度标识，优化耗时
	double m_dLengthBuff[DIVSIZE*2 + 1];
};


#endif //BEZIERCURVE_H
