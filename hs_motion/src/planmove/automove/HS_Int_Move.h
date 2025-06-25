/*
* Copyright (c) 2017
* All rights reserved.

* 文件名称：HS_Int_Move.h
* 摘    要：插补运动工厂类

* 当前版本：1.0
* 作    者：cyh
* 完成日期：
*			
*/

#ifndef _HS_INT_MOVE_H
#define _HS_INT_MOVE_H

#include "baseautomove.h"
#include "automoveparadef.h"
#include <stdio.h>
#include "AutoControl.h"
#include "HS_BasicPara.h"
#include "HS_VelPlan_Para.h"
#include "HS_GroupKin.h"

class HS_Kinematics;
using namespace hsc3::algo;

#define LIMITACCTIME    0.004              //限制加减速，调速等时间
#define MAXLOOKAHEAD    1                  //前瞻处理段数
#define MAXLOOKAHEADSLINE	20
#define SDLIMIT	        5				   //平滑最小长度约束

enum INTERTYPE                          //迭代优化的策略模式
{
    IT_NONE = 0,                        //无
    IT_MOVEDOWN_S,                      //限制优化---增加平滑时间，启动优化
    IT_MOVEDOWN,                        //二分法求解最优

    IT_MOVEUP_L2S,                      //提速，长运行段至短运行段
    IT_MOVEUP_S2S,                      //短线段之间连接，提速处理
    IT_MOVEUP_S2S_DOWN,                 //提速过程中的约束降速
};

enum RADIUS
{
    PRE = 0,							//实际规划后与上一段的平滑系数
    NEX,								//实际规划后与下一段的平滑系数
    RESTART,							//重启时的平滑系数【与下一段】
    ORIGNEX,							//原始平滑系数【上层给定与下一段】
};

enum RST_STATE
{
    RST_NORMAL = 0,
    RST_SMOOTHPRE,
    RET_SMOOTHNEX,
};

//摆焊添加
enum WEAVESTATE
{
	WEAVE_START = 0,
	WEAVE_DOWN,							//A+运行至A-或0
	WEAVE_UP,							//A-运行至A+或0
	WEAVE_MIDDOWN,						//0运行至A-
	WEAVE_MIDUP,						//0运行至A+
	WEAVE_STARTDOWN,					//A+运行至0
	WEAVE_HALFDOWN,						//0运行至0，负向
	WEAVE_HALFUP,						//0运行至0，正向
	WEAVE_STOP,
	WEAVE_DONE,
	WEAVE_CIRCLESTART,					//圆弧【加速】
	WEAVE_CIRCLEMOVE,					//圆弧【匀速】
	WEAVE_CIRCLEEND,					//圆弧【减速】
};

class HS_Int_Move
{
public:
	HS_Int_Move();
	virtual ~HS_Int_Move();	
	virtual int PreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum) = 0;
	virtual int Plan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos) = 0;

    virtual int RestartPlan(double dRatio,HS_GroupJPos &tRealJPos,RST_STATE eRstState,double &dTAll) = 0;
	virtual int Stop() = 0;
	virtual int setRatio(double ratio) = 0;
	virtual HS_MStatus execIntMove(IntData &intdata,int &iErrorId) = 0;	
	virtual int ResetData() = 0;
	virtual	HS_MStatus Move(int &iErrorId,bool bLastCycle = false) = 0;

    int MixSmoothMove(IntData &intdataPre,IntData &intdata);
    bool GetSmoothNextFlag();
    bool GetSLPlanFlag();
	bool GetHalfSmoothFlag();
	bool GetSmoothAheadFlag();
	int GetMasterCPos(double dMasterCPos[10][MaxAxisNum]);
	int SetMasterCPos(double dMasterCPos[10][MaxAxisNum]);
	double CalcStopTime();
	int StopPlanByTime(double dTStop);
	int GetRatioPara(double dRatio,double dTime[TIMECNT]);
	int SetRatioPara(double dRatio,double dTime[TIMECNT],bool bRatioOKFlag);
public:
    int GetInputParaGE(GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPreHandle,double dRatio);
	int SyncByAcc(SyncPara &tSync,bool bLimitS = false,bool bLimitE = false,bool bGetStopPara = false);
    int SyncByTime(double dTAcc,double dTDec,SyncPara &tSync,bool bRecalAcc = false);
	int GetKAccScaleByTime(double dTime,SyncPara &tSyncPara,double &dKAcc,double &dKVel);
	int GetKVelScaleByTime(double dTime,SyncPara &tSyncPara,double dKVel[MaxAxisNum]);
	int GetPlanPos(SyncPara &tSyncPara,double dTime,double dPlanPos[SpaceAxisNum]);
	int PlanPos2MPos(SyncPara &tSyncPara,double dPlanPos[SpaceAxisNum],double dMPos[5][4]);
    int PlanPos2MPos(SyncPara &tSyncCur,SyncPara &tSyncNex,double dPlanPosCur[SpaceAxisNum],double dPlanPosNex[SpaceAxisNum],double dMPosMix[5][4]);
	int Ratio(double dRatio);
	int StopPlan();
	HS_MStatus IntMove(int &iErrorId,bool bLastCycle);
	HS_MStatus StopMoveHandle(double dMovePos[MaxAxisNum]);
	HS_MStatus UpdateIntMove(IntData &intdata,int &iErrorId);
	int UpdateJPos(int iInterId);

    int SmoothHandle(SyncPara &tSyncCur,SyncPara &tSyncNex);
	int SmoothDisCalc(SyncPara &tSyncCur,bool bPre);
    int DynSmoothHandle(SyncPara &tSyncCur,SyncPara &tSyncNex);
	int CheckTrajDataAvailable(GroupTrajData *trajout,int index);
    int CheckPreHandle(GroupTrajData *tTrajData,int index);
	bool GetPreMovePara(GroupTrajData *trajout,int index,double dEndJPos[MaxAxisNum]);
    int DynSmoothPlan();
    int MoveDoneJPosCorrect();
    int JointReAdjustPara(Para_PreHandle &tPH_Joint);
    int SmoothMaxJParaCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJVel,double &dKJAcc,bool bRealCalc = false);
	int CheckQYPass(Para_PreHandle &tPreHandle);
	SyncPara GetSyncPara();
	int SetSyncParaPlan(SyncPara tSyncPara);
	int GlobalPlan(GroupTrajData *tTrajData,int iIndex,int iGroupNum,double dRatio,HS_GroupJPos &tHS_GroupJPos);
	int GlobalPreHandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex,int iGroupNum,Para_PreHandle &tPreHandle,BaseMoveData *tMoveData);
	int GlobalParaCheck(GroupMotionData &tMotionData);
	int GroupSyncCheck(HS_GroupRel tHS_GroupRel,Para_PreHandle &tPreHandle);
	void GetToolWorkNum(BaseMoveData tMoveData);
	int GroupSyncPosChange(Para_PreHandle &tPreHandle);
	int JPosAutoHandle(Para_PreHandle &tPreHandle,double dEJPos[6],int iAxis);
	bool bCheckRepeatJPos(double dJPosA[MaxAxisNum],double dJPosB[MaxAxisNum]);
	int JPosPrediction(Para_PreHandle &tPreHandle,double dEJPos[6]);
	int MoveAdjust(Para_PreHandle &tPreHandle);
	/************************************************
	摆焊添加（WEAVE）
	*************************************************/
	bool WeaveMoveCheck(GroupTrajData tTrajData);
	int WeaveClose();
	int WeaveMovePlan();
	int WeaveStartHandle(double dTWeaveAhead,double *dInitJPos,double *dSPos = NULL,double *dEPos = NULL);
	int WeavePlan(double dWeaveAmp);
	int WeavePlan(double dWeaveAmp,double dTAll);
	int WeavePlan(double dSVel,double dEVel,double dTAll);
	int WeavePlanCircle(bool bStartFlag);
	int WeaveStop();
	int WeavePosCalc(double *dPosOut,bool bStopFlag);
	int WeldCoordCalc();
	int WeaveMoveHandle(double &dWeavePos,bool bStopFlag);
	int WeaveMove_Circle(double &dWeavePos);
	int WeaveMove_SineAndL(double &dWeavePos);
	int WeaveMove_NormalSine(double &dWeavePos);
	int WeaveMove_StopSine(double &dWeavePos);
	int WeaveMove_L(double &dWeavePos);
	double WeaveMainPlan(double dTSet,bool bStopFlag = false);
	bool CheckStopWeaveType();
	int WeaveCoordHandle(double dWCoord[3][3]);
	int WeaveCoordHandle_Q(double dWCoord[3][3]);
	bool GetWeaveStopMainPos(double dMainJPos[MaxAxisNum]);
	/*************************************************/
private:
    int GetJParaByTime(double dTime,SyncPara &tSyncPara,double dJAcc[MaxAxisNum],double dJVel[MaxAxisNum]);
    int SmoothMaxJVelCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJVel);
    int SmoothMaxJAccCalc(SyncPara tSyncCur,SyncPara tSyncNex,double dTInPre,double dTInNext,double &dKJAcc,double &dKJVel);
    double CalcKJerkSmooth(double dKJAcc,double dTFreProtect,double dTSmooth);
    int GetPlanVel(SyncPara &tSyncPara,double dTime,double dPlanVel[MaxAxisNum]);
    int GetPlanAcc(SyncPara &tSyncPara,double dTime,double dPlanAcc[MaxAxisNum]);
    int GetPlanPara(SyncPara &tSyncPara,double dTime,double dPlanVel[MaxAxisNum],double dPlanAcc[MaxAxisNum]);
    double GetTSmoothByDis(SyncPara &tSync,double dTSmoothMax);
    int RatioHandleSmoothNex();
	
protected:
    HS_BasicPara *m_HS_BasicPara;
	HS_Kinematics *m_HS_Kinematic;
	HS_GroupKin *m_HS_GroupKin;
	double *m_dJVelPara;
	double *m_dJAccPara;
    LimitPara *m_tLimitPara;
    HS_RobotType m_eHS_RobotType;
    bool m_bPlanFlag;

	static double m_dLMPos[10][MAXGROUPNUM][5][4];				//上一段的规划位置，平滑合成使用
	double m_dRMPos[5][4];									//当前齐次坐标位置
	double m_dRealSJPos[MaxAxisNum];						//运动实时起点位置
	static bool m_bSmoothMoveFlag[MAXGROUPNUM];				//当前运动平滑处理，启动段判断与前一段标识，规划完进行置位处理
	static bool m_bWristQYHandleFlag[MAXGROUPNUM];			//当前段是否进行过奇异处理，提供给后面的段
	static bool m_bSmoothSynthFlag[MAXGROUPNUM];			//平滑段合成规划方式，即在平滑段进行空间位置的合成后再逆解处理

	double m_dCVelPara[2];
	double m_dCAccPara[2];
	double m_dCycle;
	double m_dTAcc;											//加速段时间
	double m_dTCon;											//同步段时间
	double m_dTDec;											//减速段时间
	double m_dTAll;											//总时间	
	double m_dTCur;											//当前时间
	bool m_bStopFlag;										//停止标识，停止完成才清除，一段运动停止命令只执行一次
    RST_STATE m_eRstState;
    double m_dTRSTPreAll;
    double m_dPercent;
	double m_dDisStart;
	double m_dDisEnd;
    double m_dKVelMax;                                      //最大速度比例限制，正常是1.0，小线段动态规划可以进行提升                                    

	double m_dTStop;	
	double m_dStopKa[MaxAxisNum];
	double m_dStopKb[MaxAxisNum];
	double m_dStopKc[MaxAxisNum];
	double m_dStopDis[MaxAxisNum];							//停止距离（当前段）
	double m_dStopSPos[MaxAxisNum];
    double m_dMovePosRatio[MaxAxisNum];

	double m_dMovePos[MaxAxisNum];
    double m_dMoveRatioPos[MaxAxisNum];
	double m_dMoveVel[MaxAxisNum];
	double m_dMoveAcc[MaxAxisNum];
	double m_dRJPos[MaxAxisNum];							//当前关节坐标
	double m_dWeaveStopMainJPos[MaxAxisNum];				//摆焊暂停主运动的关节位置
    double m_dStopPos[MaxAxisNum];                          //缓存暂停重启时运动的位移量
    double m_dStopDoneJPos[MaxAxisNum];

	VelPlanPara m_tVelPlanPara[MaxAxisNum];								 //规划参数
	SyncPara m_tSync;													 //当前规划
    static SyncPara m_tSyncAhead[MAXGROUPNUM][MAXLOOKAHEADSLINE+1];      //前瞻规划
    static int m_iLookAhead[MAXGROUPNUM];								 //前瞻段数
    static double m_dTSmoothOff[MAXGROUPNUM];                            //平滑偏移时间
    static bool m_bSLPlanFlag;                              //小线段规划标识
    static int m_iMaxLookAheadLine;                         //前瞻规划的最大数量，正常数量是1，小线段时增加前瞻最大段数
	HS_VelPlan_Para **m_HS_VelPlan_Para;					//速度规划指针	
    static int m_iDynIndex;                                 //规划段索引

	double m_dSaveRatio;									//上一次设置的倍率缓存
	double m_dSetRatio;										//当前规划使用的倍率
    double m_dTFreProtect;                          		//基于频率抑制的时间约束处理方式
	double m_dTAccBase;   

	GroupTrajData *m_tGTrajData;
	int m_iIndex;
	int m_iGroupNum;
	int m_iToolNum;
	int m_iWorkNum;
	unsigned char m_eState;									//形态
	HS_MOVETYPE m_eMoveType;								//当前运动类型，对函数进行兼容处理

    bool m_bSmoothPreFlag;						            //前一段*当前段是否有平滑
    bool m_bSmoothNextFlag;						            //当前段*下一段是否有平滑
	bool m_bTimeInNextSmooth;								//当前处于与下一段的平滑中
	bool m_bTriggerSmoothFlag;								//触发平滑标识
    bool m_bRepeatPosFlag;                                  //是否是重复点位
    bool m_bDynSmoothFlag;                                  //动态平滑标识，动态判断是否有下一段运动
    bool m_bJPosCorrectFlag;                                //点位修正，一段运动执行一次     
    bool m_bSLPlagFlag;                                     //小线段规划标识
    bool m_bWristQYFlag;                                    //腕部过奇异标识

    int m_iInterMultCnt;                                    //插补离散点位个数

	double m_dMasterCPos[10][MaxAxisNum];
	HS_GroupRel m_tHS_GroupRel;

	/************************************************
	摆焊添加（WEAVE）相关参数，静态，存在运动混合
	*************************************************/
	bool m_bStopDoneFlag;									//停止完成标识【摆焊使用】
	static bool m_bWeaveFlag;								//是否开启摆焊功能
	static double m_dWeavePosSave[3];						//缓存摆焊幅值，用来给减速停止使用
	static double m_dWeaveAmp;								//摆焊幅值
	static double m_dWeaveFreq;								//摆焊频率
	static double m_dWeavePeriod;							//摆焊周期
	static double m_dWeaveAccMAX;
	static double m_dWeaveVel;								//摆焊运动过程中的最大速度
	static double m_dWeaveSVel;	
	static bool m_bSinglePlan;								//单段规划
	static double m_dWeaveTOff;								//摆动离散化时间误差

	static double m_dWeaveTCur;								//摆动执行的当前时间	
	static double m_dWeaveKa;
	static double m_dWeaveKb;
	static double m_dWeaveKc;
	static double m_dWeaveStopSVel;
	static double m_dWeaveTStop;
	static bool m_bWeaveStopDoneFlag;
	static double m_dWeaveTAll;
	static double m_dWeavePAll;
	static double m_dWeaveSPos;
	static WEAVESTATE m_eWeaveState;
	static bool m_bWeaveStopFlag;
	static double m_dWeaveStopAmp;
	static HS_WeavePara m_tHS_WeavePara;	
	static bool m_bDStopMode;								//端点停留模式
	static double m_dCircleBaseVel;

	static double m_dAheadTAll;								//当前段前瞻至停止计算求得总时间
	static double m_dMCoord[3][3];							//缓存摆焊变换坐标系

	static int m_WeaveType;									//摆焊方向参数
	static double m_dMainSavePos[2][MaxAxisNum];			//缓存位置值【主运动】
	static bool m_bWeaveStopDir;							//摆焊停止方向，为true则表示Down，需反向
	static bool m_bWeaveWaitFlag;							//摆焊端点停止标识
	static double m_dWeaveWaitTime;							//摆焊端点停留时间

	static double m_dWeaveCicleVel;							//摆焊运动速度【圆弧】
	static double m_dWeaveCicleTAcc;
	static double m_dWeaveCiclePosAcc;
	static double m_dWeaveCicleTSCon;
	static double m_dWeaveEuler[3];							//缓存欧拉角值【滤波使用】
	static bool m_bWeaveFilterFlag;							//滤波启动标识
	static bool m_bStopSineNextFlag;
	static bool m_bStopSineWaitTFlag;						//等待标识
	static bool m_bStopWeaveMainWaitFlag;
	static int m_iWeaveNum;									//摆焊个数

	double m_dTWeaveNex;
	double m_dTLast;
	double m_dWeaveV1[3];
	double m_dWeaveV2[3];

	double m_dWeaveMoveBVel;
	bool m_bWeavePErrorFlag;								//打印摆焊坐标系误差
	bool m_bWeaveChangeYDir;
	/*************************************************/
};
#endif