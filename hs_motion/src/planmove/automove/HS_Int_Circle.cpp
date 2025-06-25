
/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Circle.cpp
* 摘    要：圆弧运动同步插补算法

* 当前版本：2.0
* 作    者：cyh
* 完成日期：
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HS_Int_Circle.h"
/************************************************
函数功能：构造函数
参    数：
返 回 值：无
*************************************************/
HS_Int_Circle::~HS_Int_Circle()
{
}

HS_Int_Circle::HS_Int_Circle(HS_GroupKin *pGroupKin)
{
	m_HS_GroupKin = pGroupKin;
	m_eMoveType = MP_Arc;
}

/************************************************
函数功能：参数复位
参   数：    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::ResetData()
{
	return 0;
}
/************************************************
函数功能：执行 运动的预处理过程
参   数：elemt----运动输入信息
		trajout---规划缓存信息    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum)
{
	int iErrorId = 0;

	BaseMoveData *tMoveData = &tGroupMotionData.tBaseMoveData[iGroupNum];
		
	Para_PreHandle tPH_Circle;

	GlobalPreHandle(tGroupMotionData,tTrajData,iIndex,iGroupNum,tPH_Circle,tMoveData);

	while(1)
	{
		iErrorId = GetStartPos(*tMoveData,tPH_Circle);
		if(iErrorId != 0) break;

		iErrorId = GetMidPos(*tMoveData,tPH_Circle);
		if(iErrorId != 0) break;

		iErrorId = GetEndPos(*tMoveData,tPH_Circle);
		if(iErrorId != 0) break;

		iErrorId = GroupSyncPosChange(tPH_Circle);
		if(iErrorId != 0) break;

		iErrorId = GetMoveDis(*tMoveData,tPH_Circle);
		if(iErrorId != 0) 
		{
			//无法规划圆弧，重复点位或者三点共线
			iErrorId = E_C_ERRORPOINT;
			LOG_ALGO("Circle Point Error!");
			break;
		}

		iErrorId = CheckPosLimit(tPH_Circle);
		if(iErrorId != 0) 
		{
			LOG_ALGO("Check Pos Limit ErrorNum = %d",iErrorId);
			break;
		}

		if(!tPH_Circle.bRepeatPos)
		{
			iErrorId = AutoAdjustPara(*tMoveData,tPH_Circle);
			if(iErrorId != 0) 
			{
				if(iErrorId == ERROR_CPOSTOJPOS)
				{
					iErrorId = E_C_MOVEUNRABLE;
				}
				break;
			}
		}
		break;
	}

	//存储预处理数据
	tPH_Circle.iPreHandledError = iErrorId;
	tPH_Circle.eState = m_eState;
	memcpy(tTrajData[iIndex].iData[iGroupNum],&tPH_Circle,sizeof(Para_PreHandle));	
	
	return iErrorId;
}
/************************************************
函数功能：获取设置的工具工件号
参   数：elemt----运动输入信息
返 回 值：无
*************************************************/
void HS_Int_Circle::GetToolWorkNum(BaseMoveData tMoveData)
{
	if(tMoveData.sCurCoordinate.iCoordinate == BASE_COORD_SYSTEM||tMoveData.sCurCoordinate.iCoordinate == JOINT_COORD_SYSTEM)
	{
		m_iToolNum = -1;
		m_iWorkNum = -1;
	}
	else if(tMoveData.sCurCoordinate.iCoordinate == USER_COORD_SYSTEM||tMoveData.sCurCoordinate.iCoordinate == TOOL_COORD_SYSTEM)
	{
		m_iToolNum = tMoveData.sCurCoordinate.iToolNum;
		m_iWorkNum = tMoveData.sCurCoordinate.iWorkNum;
	}
}
/************************************************
函数功能：获取运动起点的坐标
参   数：elemt----运动输入信息
		tPH_Circle---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetStartPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle)
{
    int iErrorId = 0;
    HS_Coordinate sCoord = tMoveData.sStartPos.hs_coordinate;

    //前后段坐标匹配
    if(!(m_tGTrajData[m_iIndex].tMotionData.bStartMove&&sCoord.iCoordinate == JOINT_COORD_SYSTEM))
    {
        //获取前一段的数据
        double dEndJPos[MaxAxisNum] = {0};
        bool bRet = GetPreMovePara(m_tGTrajData,m_iIndex,dEndJPos);
        if(bRet)
        {
            memcpy(tPH_Circle.dSetJPos[0],dEndJPos,sizeof(double)*MaxAxisNum);
            m_HS_Kinematic->HS_JPosToCPos(dEndJPos,m_iToolNum,m_iWorkNum,tPH_Circle.dSPos);	
            m_eState = m_HS_Kinematic->HS_JPosToAState(tPH_Circle.dSetJPos[0]);
        }
        else
        {
            LOG_ALGO("Error NoStartMove Can't Get Pre Data!");
            m_eState = 0;
            iErrorId = ERROR_STARTPOS;
        }
    }
    else if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
    {
        //启动段
        double dSJPos[MaxAxisNum] = {0};
        memcpy(dSJPos,tMoveData.sStartPos.dPos,sizeof(double)*MaxAxisNum);
        m_HS_Kinematic->HS_JPosToCPos(dSJPos,m_iToolNum,m_iWorkNum,tPH_Circle.dSPos);	
        memcpy(tPH_Circle.dSetJPos[0],dSJPos,sizeof(double)*MaxAxisNum);

        m_eState = m_HS_Kinematic->HS_JPosToAState(dSJPos);

        LOG_ALGO("Start JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
            dSJPos[0],dSJPos[1],dSJPos[2],dSJPos[3],dSJPos[4],dSJPos[5],dSJPos[6],dSJPos[7],dSJPos[8]);
    }
    else
    {
        LOG_ALGO("Error StartCPos With StartMoveFlag!");
        iErrorId = ERROR_STARTPOS;
    }

    LOG_ALGO("eState = %d",(int)m_eState);

    LOG_ALGO("SetSPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;Coord = %d",\
        tMoveData.sStartPos.dPos[0],tMoveData.sStartPos.dPos[1],tMoveData.sStartPos.dPos[2],tMoveData.sStartPos.dPos[3],
        tMoveData.sStartPos.dPos[4],tMoveData.sStartPos.dPos[5],tMoveData.sStartPos.dPos[6],tMoveData.sStartPos.dPos[7],
        tMoveData.sStartPos.dPos[8],sCoord.iCoordinate);

	m_HS_Kinematic->HS_JPosToCPos(tPH_Circle.dSetJPos[0],m_iToolNum,-1,tPH_Circle.dWTSPos);

    return iErrorId;
}
/************************************************
函数功能：获取运动中间点的坐标
参   数：elemt----运动输入信息
		tPH_Circle---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetMidPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sMidPos.hs_coordinate;

    LOG_ALGO("SetMPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf;Coord = %d",\
        tMoveData.sMidPos.dPos[0],tMoveData.sMidPos.dPos[1],tMoveData.sMidPos.dPos[2],tMoveData.sMidPos.dPos[3],
        tMoveData.sMidPos.dPos[4],tMoveData.sMidPos.dPos[5],tMoveData.sMidPos.dPos[6],tMoveData.sMidPos.dPos[7],
        tMoveData.sMidPos.dPos[8],sCoord.iCoordinate);

	if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
		double dSJPos[MaxAxisNum] = {0};
		memcpy(dSJPos,tMoveData.sMidPos.dPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_JPosToCPos(dSJPos,m_iToolNum,m_iWorkNum,tPH_Circle.dMPos);	
	}
	else
	{	
		double dSCPos[MaxAxisNum] = {0};
		memcpy(dSCPos,tMoveData.sMidPos.dPos,sizeof(double)*MaxAxisNum);
		memcpy(tPH_Circle.dMPos,tMoveData.sMidPos.dPos,sizeof(double)*MaxAxisNum);
		iErrorId = m_HS_Kinematic->HS_CPosChangeCoord(dSCPos,sCoord.iToolNum,sCoord.iWorkNum,m_iToolNum,m_iWorkNum,tPH_Circle.dMPos);

        if(tMoveData.b2mid)
            iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(dSCPos,sCoord.iToolNum,sCoord.iWorkNum,m_eState,tPH_Circle.dSetJPos[0],tPH_Circle.dSetJPos[1]);
		if(iErrorId != 0)
		{
            LOG_ALGO("EndCPos C2JError!");
            m_HS_Kinematic->HS_PrintCoord(sCoord.iToolNum,sCoord.iWorkNum);
			return ERROR_CPOSTOJPOS;
		}
	}

	if(tMoveData.b2mid)
		m_HS_Kinematic->HS_JPosToCPos(tPH_Circle.dSetJPos[1],m_iToolNum,-1,tPH_Circle.dWTEPos);

	return iErrorId;
}
/************************************************
函数功能：获取运动终点的坐标
参   数：elemt----运动输入信息
		tPH_Circle---预处理输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetEndPos(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle)
{
	int iErrorId = 0;
	HS_Coordinate sCoord = tMoveData.sEndPos.hs_coordinate;

    LOG_ALGO("SetEPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf; Coord = %d",\
        tMoveData.sEndPos.dPos[0],tMoveData.sEndPos.dPos[1],tMoveData.sEndPos.dPos[2],tMoveData.sEndPos.dPos[3],
        tMoveData.sEndPos.dPos[4],tMoveData.sEndPos.dPos[5],tMoveData.sEndPos.dPos[6],tMoveData.sEndPos.dPos[7],
        tMoveData.sEndPos.dPos[8],sCoord.iCoordinate);

	if(sCoord.iCoordinate == JOINT_COORD_SYSTEM)
	{
		double dEJPos[MaxAxisNum] = {0};
		memcpy(dEJPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		m_HS_Kinematic->HS_JPosToCPos(dEJPos,m_iToolNum,m_iWorkNum,tPH_Circle.dEPos);	
		memcpy(tPH_Circle.dSetJPos[1],dEJPos,sizeof(double)*MaxAxisNum);

        iErrorId = CheckQYPass(tPH_Circle);
        if(iErrorId != 0)
        {
            LOG_ALGO("ErrorEndJPos Pass QY ErrorNum = %d",iErrorId);
        }
	}
	else
	{	
		double dECPos[MaxAxisNum] = {0};
		memcpy(dECPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		memcpy(tPH_Circle.dEPos,tMoveData.sEndPos.dPos,sizeof(double)*MaxAxisNum);
		iErrorId = m_HS_Kinematic->HS_CPosChangeCoord(dECPos,sCoord.iToolNum,sCoord.iWorkNum,m_iToolNum,m_iWorkNum,tPH_Circle.dEPos);

        if(!tMoveData.b2mid)
            iErrorId = m_HS_Kinematic->HS_CPosToJPos_JXJ(dECPos,sCoord.iToolNum,sCoord.iWorkNum,m_eState,tPH_Circle.dSetJPos[0],tPH_Circle.dSetJPos[1]);

		if(iErrorId != 0)
		{
            LOG_ALGO("EndCPos C2JError!");
            m_HS_Kinematic->HS_PrintCoord(sCoord.iToolNum,sCoord.iWorkNum);
			return E_C_TARGETUNREABLE;
		}
	}

	if(!tMoveData.b2mid)
		m_HS_Kinematic->HS_JPosToCPos(tPH_Circle.dSetJPos[1],m_iToolNum,-1,tPH_Circle.dWTEPos);

	return iErrorId;
}
/************************************************
函数功能：获取运动位移量等信息
参   数：tPH_Circle---预处理输入输出信息	    
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetMoveDis(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle)
{
    int iErrorId = 0;
    //求解圆心
    double P1[3],P2[3],P3[3];
    P1[0] = tPH_Circle.dSPos[0]; P1[1] = tPH_Circle.dSPos[1]; P1[2] = tPH_Circle.dSPos[2];
    P2[0] = tPH_Circle.dMPos[0]; P2[1] = tPH_Circle.dMPos[1]; P2[2] = tPH_Circle.dMPos[2];
    P3[0] = tPH_Circle.dEPos[0]; P3[1] = tPH_Circle.dEPos[1]; P3[2] = tPH_Circle.dEPos[2];	
    //计算圆心
    double center[3] = {0};
    if(!CalCenter(P1,P2,P3,center))
        return ERROR_CIRCLE_CENTER;

    LOG_ALGO("center = %.3lf,%.3lf,%.3lf",center[0],center[1],center[2]);

    //求解转动角度以及变换矩阵
    double dP1C[3] = {P1[0]-center[0],P1[1]-center[1],P1[2]-center[2]};
    double dP2C[3] = {P2[0]-center[0],P2[1]-center[1],P2[2]-center[2]};
    double dP3C[3] = {P3[0]-center[0],P3[1]-center[1],P3[2]-center[2]};
    m_dCircleR = m_HS_Math.Matrix_Norm(3,1,dP1C);

    double n_c[3] = {dP1C[0]/m_dCircleR,dP1C[1]/m_dCircleR,dP1C[2]/m_dCircleR};
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
    double dP2N_W[4] = {P2[0],P2[1],P2[2],1};
    m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP2N_W,dP2N_C);

    double dP3N_C[4];
    double dP3N_W[4] = {P3[0],P3[1],P3[2],1};
    m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP3N_W,dP3N_C);
    double dSeata1 = atan2(dP2N_C[1],dP2N_C[0]);	//中间点
    double dSeata2 = atan2(dP3N_C[1],dP3N_C[0]);
    if(bOneLine)
        dSeata2 = PI;     

    bool bChangeDir = false;

    //P2点在外面
    if(dSeata1 < Eps || dSeata1 > dSeata2)
        bChangeDir = true; 

    if(bChangeDir)
    {
        tPH_Circle.bMidOutFlag = true;
        m_TCircle[0][1] = -m_TCircle[0][1];m_TCircle[0][2] = -m_TCircle[0][2];
        m_TCircle[1][1] = -m_TCircle[1][1];m_TCircle[1][2] = -m_TCircle[1][2];
        m_TCircle[2][1] = -m_TCircle[2][1];m_TCircle[2][2] = -m_TCircle[2][2];
        m_HS_Math.Matrix_Inverse(4,&m_TCircle[0][0],&dT[0][0]);	
   
        m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP3N_W,dP3N_C);
        dSeata = atan2(dP3N_C[1],dP3N_C[0]);      

        m_HS_Math.Matrix_Multi(4,4,1,&dT[0][0],dP2N_W,dP2N_C);
        dSeata1 = atan2(dP2N_C[1],dP2N_C[0]);

        if(dSeata1 < 0)
            dSeata1 = dSeata1 + 2*PI;	

        if(dSeata < 0)
            dSeata = dSeata + 2*PI;		
    }
    else
    {        
        dSeata = dSeata2;        	
    }	

    LOG_ALGO("dSeata1 = %.3lf,dSeata2 = %.3lf",dSeata1,dSeata);

    double dECPos[MaxAxisNum] = {0};

    if(tMoveData.b2mid)
    {
        LOG_ALGO("Circle Move To Mid!");
        tPH_Circle.dLength = dSeata1*m_dCircleR;

        memcpy(dECPos,tPH_Circle.dMPos,sizeof(double)*MaxAxisNum);
    }
    else
    {
        //如果圆弧运动的角度接近正圆，此处做特殊处理，让其运行反向小圆
        if(fabs(dSeata - 2*PI) < 0.1)
        {
            LOG_ALGO("Nearly A Whole Circle,Seata = %.3lf!",dSeata);
            m_TCircle[0][1] = -m_TCircle[0][1];m_TCircle[0][2] = -m_TCircle[0][2];
            m_TCircle[1][1] = -m_TCircle[1][1];m_TCircle[1][2] = -m_TCircle[1][2];
            m_TCircle[2][1] = -m_TCircle[2][1];m_TCircle[2][2] = -m_TCircle[2][2];				
            dSeata = 2*PI - dSeata;
        }

        LOG_ALGO("Circle Move To End!");
        tPH_Circle.dLength = dSeata*m_dCircleR;

        memcpy(dECPos,tPH_Circle.dEPos,sizeof(double)*MaxAxisNum);
    }	

    memcpy(tPH_Circle.dMTrans,m_TCircle,sizeof(double)*16);
    tPH_Circle.dCircleR = m_dCircleR;  

	//1、位置空间位移
	for(int i = 0;i < 3;i++)
		tPH_Circle.dDis[i] = dECPos[i] - tPH_Circle.dSPos[i];

	//姿态四元数求解
	m_HS_Kinematic->EulerZYX_CalcQ(tPH_Circle.dSPos,dECPos,tPH_Circle.dQ);

	//姿态位移
	m_HS_Kinematic->EulerZYX_CalcDis(&tPH_Circle.dSPos[3],&dECPos[3],&tPH_Circle.dDis[3],true);

	//对附加轴进行处理，都是 坐标
	for(int i = 6;i < MaxAxisNum;i++)
	{
		tPH_Circle.dDis[i] = dECPos[i] - tPH_Circle.dSPos[i];
	}

    tPH_Circle.dSetDis[0] = tPH_Circle.dLength;	
    //姿态变化四元数角度值，并将其转化为角度制
    tPH_Circle.dSetDis[1] = Rad2angle(tPH_Circle.dQ[0]);
    //附加轴
    for(int i = 2;i < SpaceAxisNum;i++)
    {
        tPH_Circle.dSetVel[i] = m_dJVelPara[i+4];
        tPH_Circle.dSetDis[i] = tPH_Circle.dDis[i+4];
    }

    memcpy(tPH_Circle.dOrigDis,tPH_Circle.dSetDis,sizeof(double)*SpaceAxisNum);

	double dSumDis = 0;		//总的位移长度
	double dMaxDis = 0;

	//起点末端点都是 坐标
    tPH_Circle.iMaxAxis = 0;
	for(int i = 0;i < MaxAxisNum;i++)
	{
		double dDis  = dECPos[i] - tPH_Circle.dSPos[i];
		dSumDis += fabs(dDis);
	}
	if(dSumDis < Eps)
	{
		tPH_Circle.bRepeatPos = true;
	}
	else
	{
		MoveAdjust(tPH_Circle);
	}

	LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
		tPH_Circle.dSPos[0],tPH_Circle.dSPos[1],tPH_Circle.dSPos[2],tPH_Circle.dSPos[3],tPH_Circle.dSPos[4],tPH_Circle.dSPos[5],\
		tPH_Circle.dSPos[6],tPH_Circle.dSPos[7],tPH_Circle.dSPos[8]);
	LOG_ALGO("MPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;MidOut = %d",\
		tPH_Circle.dMPos[0],tPH_Circle.dMPos[1],tPH_Circle.dMPos[2],tPH_Circle.dMPos[3],tPH_Circle.dMPos[4],tPH_Circle.dMPos[5],\
		tPH_Circle.dMPos[6],tPH_Circle.dMPos[7],tPH_Circle.dMPos[8],(int)tPH_Circle.bMidOutFlag);
	LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;",\
		tPH_Circle.dEPos[0],tPH_Circle.dEPos[1],tPH_Circle.dEPos[2],tPH_Circle.dEPos[3],tPH_Circle.dEPos[4],tPH_Circle.dEPos[5],\
		tPH_Circle.dEPos[6],tPH_Circle.dEPos[7],tPH_Circle.dEPos[8]);
	LOG_ALGO("SJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
		tPH_Circle.dSetJPos[0][0],tPH_Circle.dSetJPos[0][1],tPH_Circle.dSetJPos[0][2],tPH_Circle.dSetJPos[0][3],tPH_Circle.dSetJPos[0][4],tPH_Circle.dSetJPos[0][5],
		tPH_Circle.dSetJPos[0][6],tPH_Circle.dSetJPos[0][7],tPH_Circle.dSetJPos[0][8]);
	LOG_ALGO("EJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",
		tPH_Circle.dSetJPos[1][0],tPH_Circle.dSetJPos[1][1],tPH_Circle.dSetJPos[1][2],tPH_Circle.dSetJPos[1][3],tPH_Circle.dSetJPos[1][4],tPH_Circle.dSetJPos[1][5],
		tPH_Circle.dSetJPos[1][6],tPH_Circle.dSetJPos[1][7],tPH_Circle.dSetJPos[1][8]);
	return iErrorId;
}
/************************************************
函数功能：根据3点求圆心
参    数：p1~p3，输入3点
返 回 值：center-求解得到圆心
*************************************************/
bool HS_Int_Circle::CalCenter(double *p1,double *p2,double *p3,double *center)
{
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
    x1 = p1[0];y1 = p1[1];z1 = p1[2];
    x2 = p2[0];y2 = p2[1];z2 = p2[2];
    x3 = p3[0];y3 = p3[1];z3 = p3[2];
    double m[3][3] =
    {	
        y1*(z2-z3)+y2*(z3-z1)+y3*(z1-z2),	//A1
        x1*(z3-z2)+x2*(z1-z3)+x3*(z2-z1),	//B1
        x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2),	//C1
        2*(x2-x1),							//A2
        2*(y2-y1),							//B2
        2*(z2-z1),							//C2
        2*(x3-x1),							//A3
        2*(y3-y1),							//B3
        2*(z3-z1)							//C3
    };
    double D[3] =
    {
        x1*(y3*z2-y2*z3)+x2*(y1*z3-y3*z1)+x3*(y2*z1-y1*z2),
        x1*x1+y1*y1+z1*z1-x2*x2-y2*y2-z2*z2,
        x1*x1+y1*y1+z1*z1-x3*x3-y3*y3-z3*z3
    };
    double m_inv[3][3];
    //求逆错误，无法规划圆心
    if(!m_HS_Math.Matrix_Inverse(3,&m[0][0],&m_inv[0][0]))
        return false;
    double C[3];
    m_HS_Math.Matrix_Multi(3,3,1,&m_inv[0][0],D,C);
    center[0] = -C[0];
    center[1] = -C[1];
    center[2] = -C[2];
    return true;
}
/************************************************
函数功能：设定参数的自适应调整，主要是对 速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：elemt------运动信息
		 tPH_Circle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Circle::CheckPosLimit(Para_PreHandle &tPH_Circle)
{
    int iErrorId = 0;

    iErrorId = m_HS_Kinematic->HS_JPosLimitCheck(tPH_Circle.dSetJPos[1]);
    if(iErrorId != 0) 
        return E_C_TARGETOVERLIMIT;

    iErrorId = m_HS_Kinematic->HS_QYLimitCheck(tPH_Circle.dSetJPos[0]);
    if(iErrorId != 0) 
    {
        if(iErrorId == ERROR_QY_BORDER)
            return E_C_STARTQYBORDER;
        else if(iErrorId == ERROR_QY_INSIDE)
            return E_C_STARTQYINSIDE;
        else if(iErrorId == ERROR_QY_WRIST)
            return E_C_STARTQYWRIST;

        return iErrorId;
    }

    iErrorId = m_HS_Kinematic->HS_QYLimitCheck(tPH_Circle.dSetJPos[1]);
    if(iErrorId != 0) 
    {
        if(iErrorId == ERROR_QY_BORDER)
            return E_C_ENDQYBORDER;
        else if(iErrorId == ERROR_QY_INSIDE)
            return E_C_ENDQYINSIDE;
        else if(iErrorId == ERROR_QY_WRIST)
            return E_C_ENDQYWRIST;

        return iErrorId;
    }

    return iErrorId;
}

/************************************************
函数功能：设定参数的自适应调整，主要是对 速度比例参数进行获取，保证不同的倍率下有不同的速度
参    数：elemt------运动信息
		 tPH_Circle---处理缓存结构体
返 回 值：错误ID
*************************************************/
int HS_Int_Circle::AutoAdjustPara(BaseMoveData &tMoveData,Para_PreHandle &tPH_Circle)
{
    int iErrorId = 0;
  	//初始参数设置
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPH_Circle.dSetKVel[i] = 1.0;					
	}
	tPH_Circle.dSetKAcc[0] = 1.0;
	tPH_Circle.dSetKAcc[1] = 1.0;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPH_Circle.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPH_Circle.dSetMaxAcc = tMoveData.dDec/100;

	tPH_Circle.dSetVel[0] = tMoveData.dVel;
	tPH_Circle.dSetVel[1] = m_dCVelPara[1]*tMoveData.dVort/100;


	Para_PreHandle tPH = tPH_Circle;

	if(tMoveData.dAcc < tMoveData.dDec)
		tPH_Circle.dSetMaxAcc = tMoveData.dAcc/100;
	else
		tPH_Circle.dSetMaxAcc = tMoveData.dDec/100;	
	iErrorId = AutoAdjustJAcc(tPH_Circle);
	return iErrorId;
}
/************************************************
函数功能：基于加速度自适应限制
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Circle::AutoAdjustJAcc(Para_PreHandle &tPH_Circle)
{
	Para_PreHandle tPH = tPH_Circle;
	SyncPara tSyncPara;
    memset(&tSyncPara,0,sizeof(tSyncPara));
	int iErrorId = 0;
	double dMaxKJVel[3] = {0};					//运动的 速度比例系数，速度的非线性处理	
	for(int iCnt = 0;iCnt < MAXCNT;iCnt++)
	{
/************STEP1：规划参数的设定******************************/
		for(int i = 0;i < SpaceAxisNum;i++)
		{
			tPH.dSetVel[i] = tPH_Circle.dSetVel[i]*tPH.dSetKVel[i];
			//按照加减速比例约束
			if(i < 2)
			{
				tPH.dSetAcc[i] = m_dCAccPara[i]*tPH.dSetMaxAcc*tPH.dSetKAcc[0];
				tPH.dSetDec[i] = m_dCAccPara[i]*tPH.dSetMaxAcc*tPH.dSetKAcc[1];
			}
			else
			{
				tPH.dSetAcc[i] = m_dJAccPara[i]*tPH.dSetMaxAcc;
				tPH.dSetDec[i] = m_dJAccPara[i]*tPH.dSetMaxAcc;
			}
		}

		tSyncPara.tPreHandle = tPH;
		SyncByAcc(tSyncPara);

		/************STEP2：加速度自适应参数调整******************************/
		double dMaxKAcc[2] = {0};
        double dKJVelTemp = 0;
        double dMaxKVel = 0;
		double dTime = tSyncPara.tVelPlan.dTime[TACC]/2;
		iErrorId = GetKAccScaleByTime(dTime,tSyncPara,dMaxKAcc[0],dKJVelTemp);
		if(iErrorId != 0) return iErrorId;
        dMaxKVel = Max(dMaxKVel,dKJVelTemp);

		dTime = tSyncPara.tVelPlan.dTime[TALL] - tSyncPara.tVelPlan.dTime[TDEC]/2;
		iErrorId = GetKAccScaleByTime(dTime,tSyncPara,dMaxKAcc[1],dKJVelTemp);
		if(iErrorId != 0) return iErrorId;
        dMaxKVel = Max(dMaxKVel,dKJVelTemp);

/************STEP3：速度自适应参数调整，约束空间速度匹配 速度******************************/	
		double dKVel[3][MaxAxisNum] = {0};
		dTime = tSyncPara.tVelPlan.dTime[TACC];
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[0]);
		if(iErrorId != 0) return iErrorId;

		dTime = tSyncPara.tVelPlan.dTime[TACC] + tSyncPara.tVelPlan.dTime[TCON]/2;
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[1]);
		if(iErrorId != 0) return iErrorId;

		dTime = tSyncPara.tVelPlan.dTime[TALL] - tSyncPara.tVelPlan.dTime[TDEC];
		iErrorId = GetKVelScaleByTime(dTime,tSyncPara,dKVel[2]);
		if(iErrorId != 0) return iErrorId;

		memset(dMaxKJVel,0,sizeof(dMaxKJVel));
		for(int i = 0;i < MaxAxisNum;i++)
		{
			dMaxKJVel[0] = Max(dMaxKJVel[0],dKVel[0][i]);
			dMaxKJVel[1] = Max(dMaxKJVel[1],dKVel[1][i]);
			dMaxKJVel[2] = Max(dMaxKJVel[2],dKVel[2][i]);
		}
/************STEP4：捷度和速度比例系数调整计算******************************/
		//最大 速度比值		
		for(int i = 0;i < MaxAxisNum;i++)
		{
			double dTemp = Max(dKVel[0][i],dKVel[1][i]);
			dTemp = Max(dTemp,dKVel[2][i]);
			dMaxKVel = Max(dMaxKVel,dTemp);
		}

		//缓存当前的 速度比例值
		tPH_Circle.dOrigKJVel = dMaxKVel;

		if(dMaxKVel > 10)   //如果 速度求解超出限制值10倍，则表示有奇异位置或者不可达
		{
			//m_HS_Printer->outDebugInfo("Motion_P","Line","AutoAdjustPara",0,AllDeb,"UnReachable KMAX = %.3lf",dMaxKVel);
			return ERROR_UNREACHABLE;
		}
		//根据 执行的速度比例系数对设定速度进行缩放处理，但最终设定速度不超出原有的设定速度值
		//当前实际速度与设定速度的比例值
		double dKRealVel;
		for(int i = 0;i < 2;i++)
		{
			if(tPH.dSetVel[i] < Eps)			
				dKRealVel = 0;			
			else
				dKRealVel = fabs(tSyncPara.tVelPlan.dEVel[i])/tPH.dSetVel[i];//欧拉角360修改

			if(dMaxKVel > Eps)
				tPH.dSetKVel[i] = tPH.dSetKVel[i]*dKRealVel/dMaxKVel;	

			if(tPH.dSetKVel[i] > 1.0)
				tPH.dSetKVel[i] = 1.0;
			else if(tPH.dSetKVel[i] < 0.0001)		//避免值太小
				tPH.dSetKVel[i] = 0.0001;
		}
		
		//求解捷度放大比例参数，取较小值
		for(int i = 0;i < 2;i++)
		{
			if(dMaxKAcc[i] > Eps)
				tPH.dSetKAcc[i] = tPH.dSetKAcc[i]/dMaxKAcc[i];
		}				
	}	
    memcpy(tPH_Circle.dPreHandleVel,tSyncPara.tVelPlan.dEVel,sizeof(double)*SpaceAxisNum);
	const double MAXKJACC = 50.0;
	const double MINKJACC = 0.001;
	//对最大最小比例进行限制
	for(int i = 0;i < SpaceAxisNum;i++)
	{
		tPH_Circle.dSetKVel[i] = tPH.dSetKVel[i];
	}
	for(int i = 0;i < 2;i++)
	{
		tPH_Circle.dSetKAcc[i] = tPH.dSetKAcc[i];
		if(tPH_Circle.dSetKAcc[i] < MINKJACC)
			tPH_Circle.dSetKAcc[i] = MINKJACC;
		else if(tPH_Circle.dSetKAcc[i] > MAXKJACC)
			tPH_Circle.dSetKAcc[i] = MAXKJACC;
	}
	return 0;
}


/************************************************
函数功能：运动规划
参    数：dSDece-----减速运行时的减速度
		 pMCur------运行段信息
		 dRatio-----速度比例因子
		 dRealJPos--当前实际的 位置
返 回 值：错误ID
*************************************************/
int HS_Int_Circle::Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos)
{
	int iErrorId = 0;
	m_bPlanFlag = false;
	iErrorId = GlobalPlan(tTrajData,iIndex,iGroupNum,dRatio,tHS_GroupJPos);
	if(iErrorId != 0) return iErrorId;  

	iErrorId = GetSysPara(tTrajData,iIndex,dRatio);
	if(iErrorId != 0) return iErrorId;  

	iErrorId = StartPosCheckAndReplan();
	if(iErrorId != 0) return iErrorId;

	iErrorId = TrajectoryPlan();
	if(iErrorId != 0) return iErrorId;  

	VelPlan();

	WeaveMovePlan();

	m_bPlanFlag = true;
	m_bWristQYHandleFlag[m_iGroupNum] = false;
	m_dWeaveMoveBVel = m_tSync.tVelPlan.dEVel[0];
	return iErrorId;
}

/************************************************
函数功能：运动规划的起点点位检测以及重新规划
参    数：
返 回 值：错误ID
*************************************************/
int HS_Int_Circle::StartPosCheckAndReplan()
{
	int iErrorId = 0;

	if(CheckStopWeaveType()&&m_tGTrajData[m_iIndex].tMotionData.dCnt > Eps)
	{
		m_tGTrajData[m_iIndex].tMotionData.dCnt = 0;
		LOG_ALGO("Stop Weave Type Clear Cnt!");
	}

	memcpy(m_dRJPos,m_tSync.tPreHandle.dSetJPos[0],sizeof(double)*MaxAxisNum);

	if(m_bSmoothMoveFlag[m_iGroupNum]||m_bStopSineNextFlag)
	{
		if(m_bSmoothSynthFlag[m_iGroupNum])
		{
			memcpy(m_dRJPos,m_dRealSJPos,sizeof(double)*MaxAxisNum);
		}
		return iErrorId;
	}

	bool bRepeatFlag = bCheckRepeatJPos(m_dRealSJPos,m_tSync.tPreHandle.dSetJPos[0]);

	bool bReplanFlag = false;
	if(!bRepeatFlag)
	{
		if(m_bWristQYHandleFlag[m_iGroupNum])
		{
			LOG_ALGO("Wrist QYHandle RePlan!");
			bReplanFlag = true;
		}	
		else 
		{
			LOG_ALGO("SetJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_tSync.tPreHandle.dSetJPos[0][0],m_tSync.tPreHandle.dSetJPos[0][1],m_tSync.tPreHandle.dSetJPos[0][2],m_tSync.tPreHandle.dSetJPos[0][3],m_tSync.tPreHandle.dSetJPos[0][4],m_tSync.tPreHandle.dSetJPos[0][5],\
				m_tSync.tPreHandle.dSetJPos[0][6],m_tSync.tPreHandle.dSetJPos[0][7],m_tSync.tPreHandle.dSetJPos[0][8]);
			LOG_ALGO("RealJPos = %.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf---%.6lf,%.6lf,%.6lf",\
				m_dRealSJPos[0],m_dRealSJPos[1],m_dRealSJPos[2],m_dRealSJPos[3],m_dRealSJPos[4],m_dRealSJPos[5],\
				m_dRealSJPos[6],m_dRealSJPos[7],m_dRealSJPos[8]);
			iErrorId = E_L_STARTPOS;
			LOG_ALGO("ErrorNum:%d,Line Error StartPos!",iErrorId);
			return iErrorId;
		}
	}

	if(bReplanFlag)
	{
		
	}
	return iErrorId;
}
/************************************************
函数功能：暂停重启运动规划
参    数：
		 ratio-------速度比例因子
		 realjpos----当前实际的关节位置
返 回 值：错误ID
*************************************************/
int HS_Int_Circle::RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll)
{
    int iErrorId = 0;

    m_dSetRatio = dRatio;    

	LOG_ALGO("Restart Plan,Id = %d,GroupNum = %d,RstState = %d!",m_iIndex,m_iGroupNum,eRstState);

	memset(m_tSyncAhead[m_iGroupNum],0,sizeof(m_tSyncAhead[m_iGroupNum]));
	m_eRstState = eRstState;
	m_dTRSTPreAll = dTAll;
	m_iLookAhead[m_iGroupNum] = 0;

    //直接缓存停止距离，重新规划
    iErrorId = TrajectoryPlan();
    if(iErrorId != 0) return iErrorId;  

    VelPlan();

	if(eRstState != RST_SMOOTHPRE)
	{
		WeaveMovePlan();
	}
	else
	{
		HS_WeavePara tWeavePara = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum].tWeavePara;

		if(tWeavePara.bWeaveEn)
		{
			m_bSmoothSynthFlag[m_iGroupNum] = true;
		}
	}

    dTAll = m_dTAll;

    return iErrorId;
}

/************************************************
函数功能：从上层给定的运动结构体数据中获取待使用的信息
参    数：pMCur---运动结构体
		 dRatio--速度比例因子
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetSysPara(GroupTrajData *tTrajData,int iIndex,double dRatio)
{	
	m_tGTrajData = tTrajData;
	m_iIndex = iIndex;
	m_dSetRatio = dRatio;

	COORD_SYSTEM iCoordinate = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iCoordinate;	

    //设置系统工具工件坐标系
    if(iCoordinate == BASE_COORD_SYSTEM||iCoordinate == JOINT_COORD_SYSTEM)
    {
        m_iToolNum = -1;
        m_iWorkNum = -1;
    }
    else if(iCoordinate == USER_COORD_SYSTEM||iCoordinate == TOOL_COORD_SYSTEM)
    {
        m_iToolNum = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iToolNum;
        m_iWorkNum = tTrajData[iIndex].tMotionData.tBaseMoveData[m_iGroupNum].sCurCoordinate.iWorkNum;
    }

    m_HS_Kinematic->HS_SetCoordinate(m_iToolNum,m_iWorkNum);
	return 0;
}
/************************************************
函数功能：插补规划，根据给定的位置速度加速度等信息进行速度、轨迹的插补规划
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_Circle::TrajectoryPlan(void)
{	
	m_bStopDoneFlag = false;

    memset(&m_tSync,0,sizeof(SyncPara));
    GetInputPara(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSync.tPreHandle,1.0);
    m_eState = m_tSync.tPreHandle.eState;
    SyncByAcc(m_tSync,true,true,true);

    GetInputPara(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSync.tPreHandle,m_dSetRatio);
    SyncByAcc(m_tSync,true,true);	

    //平滑规划【单段式前瞻】
    int iLookAhead = 0;
    int iErrorId = 0;
    m_bDynSmoothFlag = false;

    for(iLookAhead = 0;iLookAhead < MAXLOOKAHEAD;iLookAhead++)
    {
		int iBuffCur = (m_iIndex + iLookAhead)%MaxBuffSize;
		GroupTrajData tTrajDataCur = m_tGTrajData[iBuffCur];
		int iBuffNext = (m_iIndex + iLookAhead + 1)%MaxBuffSize;
		GroupTrajData tTrajDataNex = m_tGTrajData[iBuffNext];  

        //已规划部分
        if(iLookAhead < m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
            continue;
        }
        else if(iLookAhead == m_iLookAhead[m_iGroupNum] - 1)
        {
            m_tSyncAhead[m_iGroupNum][iLookAhead] = m_tSyncAhead[m_iGroupNum][iLookAhead+1];
        }
        else if(iLookAhead == 0)
        {
            memset(&m_tSyncAhead[m_iGroupNum][iLookAhead],0,sizeof(SyncPara));
            GetInputParaGE(m_tGTrajData,m_iIndex,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead].tPreHandle,m_dSetRatio);
            if(m_eRstState == RST_SMOOTHPRE)
            {
                SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
                m_tGTrajData[m_iIndex].tMotionData.dCnt = 0;
                break;
            }
            else
            {
                if(tTrajDataCur.tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iBuffNext) == 0)
				{
					if((WeaveMoveCheck(tTrajDataCur)&&!WeaveMoveCheck(tTrajDataNex))||
						(!WeaveMoveCheck(tTrajDataCur)&&WeaveMoveCheck(tTrajDataNex)))
					{
						tTrajDataCur.tMotionData.dCnt = 0;
						LOG_ALGO("Weave Move Clear Cnt!");
						SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
						break;
					}
					else
						SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true);	
				}
                else
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead],true,true);	
            }

            if(m_eRstState == RET_SMOOTHNEX)
            {
                if(m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TACC] < m_dTRSTPreAll)
                {
                    SyncByTime(m_dTRSTPreAll,m_tSyncAhead[m_iGroupNum][iLookAhead].tVelPlan.dTime[TDEC],m_tSyncAhead[m_iGroupNum][iLookAhead]);
                }
            }
        } 

        if(tTrajDataCur.tMotionData.dCnt > Eps)
        {
            int iRet = CheckTrajDataAvailable(m_tGTrajData,iBuffNext);
            if(iRet == 0)
            {
				if((WeaveMoveCheck(tTrajDataCur)&&!WeaveMoveCheck(tTrajDataNex))||
					(!WeaveMoveCheck(tTrajDataCur)&&WeaveMoveCheck(tTrajDataNex)))
				{
					tTrajDataCur.tMotionData.dCnt = 0;
					LOG_ALGO("Weave Move Clear Cnt!");
					break;
				}
                //混合平滑处理
                memset(&m_tSyncAhead[m_iGroupNum][iLookAhead+1],0,sizeof(SyncPara));
                GetInputParaGE(m_tGTrajData,iBuffNext,m_iGroupNum,m_tSyncAhead[m_iGroupNum][iLookAhead+1].tPreHandle,m_dSetRatio);

                int iBuffNext2 = (m_iIndex + iLookAhead + 2)%MaxBuffSize;
                if(tTrajDataNex.tMotionData.dCnt > Eps&&CheckTrajDataAvailable(m_tGTrajData,iBuffNext2) == 0)
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                else
                    SyncByAcc(m_tSyncAhead[m_iGroupNum][iLookAhead+1],false,true);

                iErrorId = SmoothHandle(m_tSyncAhead[m_iGroupNum][iLookAhead],m_tSyncAhead[m_iGroupNum][iLookAhead+1]);
                if(iErrorId != 0)
                    return iErrorId;
            }
            else if(iRet == -1)
            {
                //无点位，动态平滑规划
                m_bDynSmoothFlag = true;
                LOG_ALGO("Need Dyn Smooth Plan!");
                break;
            }
            else
            {
                //有点位，但是点位异常
                break;
            }
        }
        else
        {
            break;
        }
    }

    m_iLookAhead[m_iGroupNum] = iLookAhead;

    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dAcc,m_tSync.tVelPlan.dAcc,sizeof(m_tSync.tVelPlan.dAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dDec,m_tSync.tVelPlan.dDec,sizeof(m_tSync.tVelPlan.dDec));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dJerk,m_tSync.tVelPlan.dJerk,sizeof(m_tSync.tVelPlan.dJerk));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopAcc,m_tSync.tVelPlan.dStopAcc,sizeof(m_tSync.tVelPlan.dStopAcc));
    memcpy(m_tSyncAhead[m_iGroupNum][0].tVelPlan.dStopJerk,m_tSync.tVelPlan.dStopJerk,sizeof(m_tSync.tVelPlan.dStopJerk));
    m_tSync = m_tSyncAhead[m_iGroupNum][0];

	if(m_tSync.tVelPlan.dTSmooth[NEX] > Eps)
	{
		m_bSmoothMoveFlag[m_iGroupNum] = true;
		m_bSmoothNextFlag = true;
	}
	else 
	{
		m_bSmoothMoveFlag[m_iGroupNum] = false;
		m_bSmoothNextFlag = false;
	}

    m_dTAcc = m_tSync.tVelPlan.dTime[TACC];
    m_dTCon = m_tSync.tVelPlan.dTime[TCON];
    m_dTDec = m_tSync.tVelPlan.dTime[TDEC];
    m_dTAll = m_tSync.tVelPlan.dTime[TALL];
	return 0;
}

/************************************************
函数功能：获取输出信息进行数据的存储
参    数：		 
返 回 值：错误码
*************************************************/
int HS_Int_Circle::GetInputPara(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPH_Circle,double dRatio)
{
	int iErrorId = 0;

	memcpy(&tPH_Circle,tTrajData[iIndex].iData[iGroupNum],sizeof(Para_PreHandle));    

	tPH_Circle.dSetVel[0] = tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVel*dRatio*tPH_Circle.dSetKVel[0];
	tPH_Circle.dSetVel[1] = m_dCVelPara[1]*tTrajData[iIndex].tMotionData.tBaseMoveData[iGroupNum].dVort/100*dRatio*tPH_Circle.dSetKVel[1];
	for(int i = 0;i < 2;i++)
	{
		//按照加减速比例约束
		tPH_Circle.dSetAcc[i] = m_dCAccPara[i]*tPH_Circle.dSetMaxAcc*tPH_Circle.dSetKAcc[0]*dRatio;
		tPH_Circle.dSetDec[i] = m_dCAccPara[i]*tPH_Circle.dSetMaxAcc*tPH_Circle.dSetKAcc[1]*dRatio;
        tPH_Circle.dSetDis[i] = tPH_Circle.dOrigDis[i] - m_dStopPos[i];
	}
	//附加轴
	for(int i = 2;i < SpaceAxisNum;i++)
	{
		tPH_Circle.dSetVel[i] = m_dJVelPara[i+4]*dRatio;
		tPH_Circle.dSetAcc[i] = m_dJAccPara[i+4]*dRatio*tPH_Circle.dSetMaxAcc;
		tPH_Circle.dSetDec[i] = m_dJAccPara[i+4]*dRatio*tPH_Circle.dSetMaxAcc;
        tPH_Circle.dSetDis[i] = tPH_Circle.dOrigDis[i] - m_dStopPos[i];
	}

    m_dTFreProtect = m_HS_BasicPara->HS_GetTFre(tPH_Circle.dSetJPos[0],tTrajData[iIndex].tMotionData.iSmooth,m_iGroupNum);
	return iErrorId;
}

/************************************************
函数功能：速度规划
参    数：
返 回 值：错误码
*************************************************/
int HS_Int_Circle::VelPlan(void)
{
	//执行速度规划
	memset(m_tVelPlanPara,0,sizeof(m_tVelPlanPara));	

    FilterControl tFilterControl = m_tGTrajData[m_iIndex].tMotionData.tFilterControl;

	//暂不使用
	tFilterControl.bFilterOpenFlag = false;

	for(int i = 0;i < SpaceAxisNum;i++)
	{
        m_tVelPlanPara[i].eTypeAcc = APType_Para;
		m_tVelPlanPara[i].dTCon = m_tSync.tVelPlan.dTime[TCON];
		m_tVelPlanPara[i].dTAcc = m_tSync.tVelPlan.dTime[TACC];
		m_tVelPlanPara[i].dTDec = m_tSync.tVelPlan.dTime[TDEC];
		m_tVelPlanPara[i].dTAll = m_tSync.tVelPlan.dTime[TALL];
        m_tVelPlanPara[i].dTSmoothOff = m_dTSmoothOff[m_iGroupNum];
		m_tVelPlanPara[i].dDis = m_tSync.tPreHandle.dSetDis[i];
		m_tVelPlanPara[i].dEVel = m_tSync.tVelPlan.dEVel[i];
        if(tFilterControl.bFilterOpenFlag)
        {
            m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i],tFilterControl.tFilterPara);
        }
        else
            m_HS_VelPlan_Para[i]->Plan(m_tVelPlanPara[i]);	
	}

    //初始化
	//memcpy(m_dMovePos,m_tSync.tPreHandle.dSPos,sizeof(m_dMovePos));
    memset(m_dMovePos,0,sizeof(m_dMovePos));
	memset(m_dMoveVel,0,sizeof(m_dMoveVel));
	memset(m_dMoveAcc,0,sizeof(m_dMoveAcc));
	m_bStopFlag = false;

	m_dTCur = m_dCycle + m_dTSmoothOff[m_iGroupNum];
	m_dTSmoothOff[m_iGroupNum] = 0;

    // //如果是与前一段同步段平滑，则不清空时间数据
    // if(!(m_eTrackModePre == TRM_Sync&&m_bSmoothPreFlag))
    //     m_dTAddTrack = m_dTCur;
	// //打印设定参数
	// HS_geninfo *m_pMCur = m_pMCurBase +  m_iBuffP;

    // //传送带跟踪状态打印

	BaseMoveData tBaseMoveData = m_tGTrajData[m_iIndex].tMotionData.tBaseMoveData[m_iGroupNum];

	LOG_ALGO("SetVel = %.0lf,%.0lf---%.0lf,%.0lf,%.0lf;SetAcc = %.0lf,%.0lf;Smooth = %d;TSmooth = %.3lf;",\
         tBaseMoveData.dVel,tBaseMoveData.dVort,m_dJVelPara[6],m_dJVelPara[7],m_dJVelPara[8],\
		 tBaseMoveData.dAcc,tBaseMoveData.dDec,m_tGTrajData[m_iIndex].tMotionData.iSmooth,m_dTFreProtect);
	LOG_ALGO("SPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSPos[0],m_tSync.tPreHandle.dSPos[1],m_tSync.tPreHandle.dSPos[2],m_tSync.tPreHandle.dSPos[3],m_tSync.tPreHandle.dSPos[4],m_tSync.tPreHandle.dSPos[5],\
		m_tSync.tPreHandle.dSPos[6],m_tSync.tPreHandle.dSPos[7],m_tSync.tPreHandle.dSPos[8]);
    LOG_ALGO("MPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tPreHandle.dMPos[0],m_tSync.tPreHandle.dMPos[1],m_tSync.tPreHandle.dMPos[2],m_tSync.tPreHandle.dMPos[3],m_tSync.tPreHandle.dMPos[4],m_tSync.tPreHandle.dMPos[5],\
        m_tSync.tPreHandle.dMPos[6],m_tSync.tPreHandle.dMPos[7],m_tSync.tPreHandle.dMPos[8]);
	LOG_ALGO("EPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dEPos[0],m_tSync.tPreHandle.dEPos[1],m_tSync.tPreHandle.dEPos[2],m_tSync.tPreHandle.dEPos[3],m_tSync.tPreHandle.dEPos[4],m_tSync.tPreHandle.dEPos[5],\
		m_tSync.tPreHandle.dEPos[6],m_tSync.tPreHandle.dEPos[7],m_tSync.tPreHandle.dEPos[8]);
	LOG_ALGO("Dis  = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dDis[0],m_tSync.tPreHandle.dDis[1],m_tSync.tPreHandle.dDis[2],m_tSync.tPreHandle.dDis[3],m_tSync.tPreHandle.dDis[4],m_tSync.tPreHandle.dDis[5],\
		m_tSync.tPreHandle.dDis[6],m_tSync.tPreHandle.dDis[7],m_tSync.tPreHandle.dDis[8]);
    LOG_ALGO("Firter :Open = %d,Type = %d,Fre = %.3lf,Grade = %d",\
        tFilterControl.bFilterOpenFlag,tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade);

    double dArcAngle = 0;
    if(m_tSync.tPreHandle.dCircleR > Eps)
        dArcAngle = m_tSync.tPreHandle.dOrigDis[0]/m_tSync.tPreHandle.dCircleR*rad2deg;

	LOG_ALGO("Length= %.3lf,Angle = %.3lf;R = %.3lf,ArcAngle = %.3lf;LSmooth = %.3lf,%.3lf",\
        m_tSync.tPreHandle.dOrigDis[0],m_tSync.tPreHandle.dOrigDis[1],m_tSync.tPreHandle.dCircleR,dArcAngle,m_tSync.tPreHandle.dDisSmooth[PRE],m_tSync.tPreHandle.dDisSmooth[NEX]);
    LOG_ALGO("StopPos = %.3lf, %.3lf;",\
        m_dStopPos[0],m_dStopPos[1]);
	LOG_ALGO("Ratio = %.2lf;KJVel = %.3lf;Cnt = %.3lf",m_dSetRatio,m_tSync.tPreHandle.dRealKJVel,m_tGTrajData[m_iIndex].tMotionData.dCnt);

	//打印规划参数
	LOG_ALGO("RealVel = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tVelPlan.dEVel[0],m_tSync.tVelPlan.dEVel[1],m_tSync.tVelPlan.dEVel[2],m_tSync.tVelPlan.dEVel[3],m_tSync.tVelPlan.dEVel[4]);
	LOG_ALGO("SetKVel = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
		m_tSync.tPreHandle.dSetKVel[0],m_tSync.tPreHandle.dSetKVel[1],m_tSync.tPreHandle.dSetKVel[2],m_tSync.tPreHandle.dSetKVel[3],m_tSync.tPreHandle.dSetKVel[4]);	
    LOG_ALGO("StopAcc = %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dStopAcc[0],m_tSync.tVelPlan.dStopAcc[1],m_tSync.tVelPlan.dStopAcc[2],m_tSync.tVelPlan.dStopAcc[3],m_tSync.tVelPlan.dStopAcc[4]);
    LOG_ALGO("StopJerk= %.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tSync.tVelPlan.dStopJerk[0],m_tSync.tVelPlan.dStopJerk[1],m_tSync.tVelPlan.dStopJerk[2],m_tSync.tVelPlan.dStopJerk[3],m_tSync.tVelPlan.dStopJerk[4]);	
    LOG_ALGO("TAcc = %.3lf,TCon = %.3lf,TDec = %.3lf,TAll = %.3lf,TSmooth = %.3lf---%.3lf",\
        m_tSync.tVelPlan.dTime[TACC],m_tSync.tVelPlan.dTime[TCON],m_tSync.tVelPlan.dTime[TDEC],m_tSync.tVelPlan.dTime[TALL],m_tSync.tVelPlan.dTSmooth[PRE],m_tSync.tVelPlan.dTSmooth[NEX]);
  
    return 0;
}

/************************************************
函数功能：计算速度，生成曲线【执行一次运动插补】
参    数：无
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Circle::Move(int &iErrorId,bool bLastCycle)
{
	return IntMove(iErrorId,bLastCycle);			
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：Busy---运行中
		M_Done--完成运动
*************************************************/
HS_MStatus HS_Int_Circle::execIntMove(IntData &intdata,int &iErrorId)
{
    if(m_bDynSmoothFlag)
    {
        DynSmoothPlan();
    }

	HS_MStatus eMStatus = UpdateIntMove(intdata,iErrorId);	
	return eMStatus;
}

/************************************************
函数功能：点位输出，多段点位
参    数：无
返 回 值：错误码
*************************************************/
int HS_Int_Circle::Stop()
{
	return StopPlan();
}
/************************************************
函数功能：运动调速
参    数：设定的速度倍率
返 回 值：错误码
*************************************************/
int HS_Int_Circle::setRatio(double dRatio)
{
    if(m_bStopFlag)	//执行减速停止，响应调速
        return 0;

	return Ratio(dRatio);
}