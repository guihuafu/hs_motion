/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Kinematics.h
* 摘    要：运动学基础运算库，主要包含正逆解以及各类坐标系下的运算处理

* 当前版本：2.0
* 作    者：cyh
* 完成日期：2023-11-20
*			
*/
#ifndef _HS_KINRMSTICS_H
#define _HS_KINRMSTICS_H

#include "HS_BasicPara.h"
#include "HS_Math.h"
#include "automoveparadef.h"

using namespace hsc3::algo;

//奇异处理结构体参数
struct QYHandle
{
    bool bIsMoveing;                  //是否在运动中信号，启动时刻清除，用来判断是否在运动中
    bool bQYWristInFlag;              //是否初步腕部奇异中【用来进行出奇异的修正】
    bool bQYQuitHandleFLag;           //腕部奇异出奇异空间修正处理
    double dJVel[6];                  //缓存各轴的速度值，奇异处理计算加速度使用
    double dJAcc[6];                  //加速度值
    double dWristQYAngle;             //腕部奇异阈值
    double dWristKDamp;               //腕部奇异阻尼影响因子，值越小，阻尼效果越弱，更平稳，值越大，阻尼越强，加速度变化也越大
    bool bSetQYAngle;
    double dAngleVel[3];              //姿态速度值
    double dFBMPos[4][4];             //缓存上个周期的下发位姿，用来进行开环的姿态速度计算
};

class HS_Kinematics:public HS_Math
{
public:	
	HS_Kinematics(int iGroupNum);
    ~HS_Kinematics();
    /************辅助函数************************/
    int PrintKeyInfo();
    void InitPara();
    int HS_PrintCoord(int iTool,int iWork);
    /************工具工件设置等************************/
    int HS_SetCoordinate(int iToolNum,int iWorkNum);
	int HS_PrintCoordinate(int iToolNum,int iWorkNum);
    int HS_SetTCoordinate(int iToolNum,double dJPos[6]);
    void HS_SetManualWristQY(bool bOpenFlag,double dRealPos[MaxAxisNum]);
    int HS_ResetQYHandle();
    /************点位转换相关【正解】*******************/
	int HS_JPosToMPos(double dJPos[MaxAxisNum],CPType eCPType,double dMPos[5][4]);	
	int HS_JPosToMPos(double dJPos[MaxAxisNum],double dMPos[5][4]);
	int HS_JPosToCPos(double dJPos[MaxAxisNum],CPType eCPType,double dCPos[MaxAxisNum]);
	int HS_JPosToCPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dCPos[MaxAxisNum]);
	int HS_JPosToMPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dMPos[5][4]);
	int HS_JPosToCPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dFBCPos[6],double dTWCPos[6],bool bExtCoorper);
    int HS_MPosToCPos(double dMPos[4][4],double dCPos[6]);
    unsigned char HS_JPosToAState(double dJPos[6]);	
	int HS_JPosToTCMPos(double dJPos[MaxAxisNum],int iToolNum,double dTCMPos[5][4]);
    /************点位转换相关【逆解】*******************/
    int HS_MPosToJPos_JXJ(double dMPos[4][4],int iToolNum,int iWorkNum,unsigned char eState,CPType eCPType,double dJPos[6],bool bWristQyFlag = false);
    int HS_MPosToJPos_LJ(double dMPos[5][4],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9],bool bWristQyFlag = false);
    int HS_CPosToJPos_LJ(double dCPos[6],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9],bool bWristQyFlag = false);
    int HS_MPosToJPos(double dMPos[5][4],CPType eCPType,double dLJPos[6],double dCJPos[6],bool bWristQyFlag = false);
    int HS_CPosToJPos(double dCPos[6],CPType eCPType,double dLJPos[6],double dCJPos[6]);
    int HS_CPosToJPos_JXJ(double *dCPos,int iToolNum,int iWorkNum,unsigned char eState,double dJPos[MaxAxisNum],bool bWristQyFlag = false);
    int HS_CPosToJPos_JXJ(double *dCPos,int iToolNum,int iWorkNum,unsigned char eState,double dSJPos[MaxAxisNum],double dJPos[MaxAxisNum],bool bWristQyFlag = false);
    int HS_CPosChangeCoord(double dCPosA[6],int iToolNumA,int iWorkNumA,int iToolNumB,int iWorkNumB,double dCPosB[6]);
    int HS_CPosToMPos(double dCPos[6],double dMPos[4][4]);
    int HS_CPosToJPos_Hand(double dCPos[6],double dInitCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iHandMoveAxis);
    int HS_CPosToJPos_HandAhead(double dCPos[6],double dInitCPos[6],double dLJPos[6],double dCJPos[6]);
	int HS_SyncRelativePosChange(double dMasterCPos[6],double dSlaveCPos[6],double dRelativeCPos[6]);
	int HS_SyncRelativeToSlave(double dMasterCPos[6],double dRelativeMPos[5][4],double dSlaveMPos[5][4]);
	int HS_TWPosToTCPos(double dTWCPos[MaxAxisNum],double dTCCPos[MaxAxisNum]);
	int HS_TCMPosToTWMPos(double dTCMPos[5][4],double dTWMPos[5][4]);
	int HS_TCMPosToJPos(double dBaseTCMPos[5][4],int iToolNum,double dRJPos[MaxAxisNum]);
    /************点位计算相关**************************/
    int EulerZYX_CalcQ(double dCPosA[6],double dCPosB[6],double dQ[4]);
    int EulerZYX_CalcDis(double *dEuler1,double *dEuler2,double *dDis,bool bEulerQY);
    bool CalCenter(double *p1,double *p2,double *p3,double *center);
    int EnlerNearstHandle(double *dCPosRef,double *dCPosIn);
    void HS_CPosOffset_RotLeft(double dCPosIn[6],double dCPosOffset[6],double dCPosOut[6]);
    int HS_Jacobian(double dJPos[6],double *dJacobian,bool bTool);
    /************点位检测相关**************************/
    int HS_JPosLimitCheck(double *dPos);    
    int HS_NearestPoint(double &dPosSet,double dRefPos,int iLimitAxis);
    int HS_JPosNearestHandle(double dSetJPos[6],double dRegJPos[6]);
    int HS_QYErrorCheck(double dJPos[6]);
    int HS_QYLimitCheck(double dJPos[6]);
    int HS_QYDynCheck(double dCurJPos[6],double dNexJPos[6]);
	bool HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB);
    /************奇异处理相关**************************/
    bool HS_SetWristQYHandle(bool bWristQYHandleFlag);
    /************参数获取相关**************************/
    int AutoHandleCVel(double dCVel[2],double dCAcc[2]);
    HS_RobotType GetRobotType();
    bool GetA360Flag();
private:
    void Test();
    int HS_JPosToMPos_Puma(double dJPos[6],double dMPos[4][4]);
    int HS_JPosToMPos_Scara(double dJPos[6],double dMPos[4][4]);

    int HS_MPosToJPos_Pieper(double dFBMatrix[][4],unsigned char eState,double *dJPos,bool bWristQyFlag = false);
	int HS_MPosToJPos_PieperCobot(double dFBMatrix[][4],unsigned char eState,double *dJPos,bool bWristQyFlag = false);
    int HS_MPosToJPos_Iter(double dMPos[4][4],unsigned char eState,double dSJPos[6],double dJPos[6]);
	int HS_MPosToJPos_Iter_Cobot(double dMPos[4][4],unsigned char eState,double dSJPos[6],double dJPos[6]);
    int HS_MPosToJPos_Scara(double dFBMatrix[][4],unsigned char eState,double dJPos[6]);
    int HS_MPosToJPos_Scara(double dMPos[4][4],double dLJPos[6],double dCJPos[6]);
	int HS_FBMPosToJPos_Puma(double dFBMPos[4][4],unsigned char eState,double dJPos[6],bool bWristQyFlag = false);

	int HS_MPosToJPos_QYHandle_BR(double dMPos[4][4],double dLJPos[6],double dCJPos[6]);
    void HS_MPosToCVel(double dCurMPos[4][4],double dLastMPos[4][4],double dCVel[6]);
    int HS_CVToJV(double dJPos[6],double dCVel[6],double dJVel[6],bool bTool = false,int iToolNum = 0,bool bWristQyFlag = false);
	int HS_CVToJV_BR(double dJPos[6],double dCVel[6],double dJVel[6]);
    int HS_Jacobian(double dJPos[6],double dJacobian[6][6],bool bTool = false,int iToolNum = 0);
	int HS_JacobianQYWOpt(double dJPos[4],double dJacobian[6][5]);
    int HS_JacobianTool(double dJPos[6],double dJacobian[6][6]);
    int HS_JacobianScara(double dJPos[4],double dJacobian[6][4],bool bTool = false,int iToolNum = 0);
    int HS_CVelAutoLimit(double dCVel[6],double dLJPos[6],double dJVel[6],double &dKCVel,int iMoveAxis);
    int HS_MPosToJPos(double dMPos[4][4],double dLJPos[6],double dCJPos[6],bool bTool = false,int iToolNum = 0,bool bWristQyFlag = false);
    void HS_TransformPlus(double *pdMatrix,double dJPos,double dA,double dD,double dC,double dB);
    int HS_FBMPosToTWMPos(double dFBMPos[4][4],CPType eCPType,double dTWMPos[4][4]);
    int HS_FBMPosToTWMPos(double dFBMPos[4][4],int iToolNum,int iWorkNum,double dTWMPos[4][4]);
    int HS_FBMPosToTWMPos_ExtCoorper(double dFBMPos[4][4],int iToolNum,int iWorkNum,double dTWMPos[4][4]);
    int HS_TWMPosToFBMPos(double dTWMPos[5][4],CPType eCPType,double dFBMPos[4][4]);
	int HS_TWMPosToFBMPos(double dTWMPos[5][4],CPType eCPType,double dFBMPos[4][4],double dTBMPos[4][4]);
    int HS_TWMPosToFBMPos(double dTWMPos[4][4],int iToolNum,int iWorkNum,double dFBMPos[4][4]);
	int HS_TWMPosToFBMPos(double dTWMPos[4][4],int iToolNum,int iWorkNum,double dFBMPos[4][4],double dTBMPos[4][4]);
    double Dis_ZYX(double *dZYX1,double *dZYX2);
    int HS_ZYXChange(double *pZYXIn,double *pZYXOut);
    double HS_MatrixDis(double dMPosA[4][4],double dMPosB[4][4]);
    double BorderCenterDis(double dJPos[6]);
    int HS_CPos2JPos_QYHandle(double dCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iMoveAxis);
	int HS_CPos2JPos_QYHandle_BR(double dCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iMoveAxis);
    int HS_QYStaticCheck(double dJPos[6],double dQYPara_Inter,double dQYPara_Border,double dQYPara_Wrist);

	int HS_JPosToFlMPos(double dJPos[3],double dFlBMPos[4][4]);
	int HS_JPosToPoMPos(double dJPos[MaxAxisNum],double dPoBMPos[4][4]);

    HS_BasicPara *m_HS_BasicPara;
    double (*m_dDHPara)[4];
	HS_Coord *m_dFlTrackCoord;						//地轨
	HS_Coord *m_dPositionerCoord;					//变位机
    bool m_bTypeBR;
    HS_RobotType *m_eRobotType;
	HS_RobotType_sub *m_eRobotType_sub;
	bool m_bCobot6Flag;								//6轴协作机器人标识
    double m_QYKbPara;
    LimitPara *m_tLimitPara;
    double *m_dJVelPara;
	double *m_dJAccPara;
	double (*m_dToolCoord)[6];			            //工具坐标系：0~16,0为基坐标系
	double (*m_dWorkCoord)[6];                      //工件坐标系：0~16,0为基坐标系
    double *m_dWorldCoord;                          //世界坐标系
    double m_dCycle;
	int m_iGroupNum;
    bool   m_bScaraA360Flag;               
    double m_dA360BaseAngle;
    double m_dA360TWOffset;

    //当前实时使用的工具工件相关坐标变换矩阵
	double m_dTFMatrix[4][4];						
	double m_dFTMatrix[4][4];
	double m_dWBMatrix[4][4];
	double m_dBWMatrix[4][4];
	double m_dTFMatrix_T[4][4];						//工具雅可比使用	
    double m_dExtCoorper[6];                        //地轨协同坐标系:地轨坐标系相对基础坐标系
    double m_dEBCoord[4][4];                        //齐次矩阵，地轨相对基坐标
    double m_dBECoord[4][4];                        //齐次矩阵，基坐标相对地轨
    int m_iExtNum;                                  //地轨轴轴号
    int m_iToolNum;                                 //使用的工具号
    int m_iWorkNum;                                 //使用的工件号

    QYHandle m_tQYHandle;
public:
    bool m_bWristQYHandleFlag;                      //腕部奇异处理【阻尼过奇异优化】
};


#endif