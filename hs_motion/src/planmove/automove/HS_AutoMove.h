#pragma once

#include "automove.h"
#include "motionpara.h"
#include "HS_FilterHandle.h"
#include "HS_SetPosCheck.h"
#include "HS_BasicPara.h"
#include "HS_GroupKin.h"

class HS_Int_Factory;
class HS_Kinematics;

using namespace hsc3::algo;

//停止状态
enum HS_eStopState
{
    S_NONE = 0,				//没有停止指令
    S_STOP ,				//停止指令的停止
    S_SLSTOP ,				//【小线段】停止指令的停止
    S_HALT,				    //暂停
    S_ABORT,			    //中断产生的停止
    S_ERROR,			    //错误产生的停止
    S_NOACTION,			    //平滑运动未收到Action产生的停止	
    S_TRACKSTOP,            //传送带超下游区的停止
};

#define MaxJointBuffCnt     4


class  HS_AutoMove 
{
public:
    HS_AutoMove(MotionPara *para,double dCycle,int iInterMultCnt);
    ~HS_AutoMove();
    int execPrintKeyInfo();
    void execReset();

	int execPrehandle(GroupMotionData tGroupMotionData, GroupTrajData *tTrajData,int iIndex);
	int execPlanMove(GroupTrajData *tTrajData,int iIndex,double dRatio,HS_GroupJPos &tHS_GroupJPos);
	int execStopPlan();
	int execStopRestartPlan(double dRatio,HS_GroupJPos &tRealJPos);

    int setRatio(double ratio);
	HS_MStatus execIntMove(IntData &intdata,int &iErrorId);
    int execGetJoint(int *joint);
	bool execGetMoveOnFlag();

	bool execGetSmoothAheadFlag();
	int execGetCurMoveId();

	bool HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB);
private:
    void Reset();
    void InitPara();
	void RestartInit();
    int AutoFilterHandle(IntData &intdata,HS_MStatus &eMStatus);
    bool SpaceFilterHandle(IntData &intdata,HS_MStatus &eMStatus);
    int AutoPosCheck(IntData &intdata,HS_MStatus &eMStatus);
    int MoveDoneHandle();
    bool RepeatPosCheck(double *dJPosA,double *dJPosB);
    void PushJoint(int iJointNum);
	HS_MStatus GroupIntMove(IntData &tIntdata,int &iErrorId,bool bPreFlag = false);
	int GroupSyncHandle();
	int GroupIdSort(HS_GroupRel tHS_GroupRel);
	HS_MStatus RestartAddMove(IntData &tIntdata,int &iErrorId);

	int NormalRestart();
	int WeaveRestartAddMove(double dRatio,HS_GroupJPos &tRealJPos);

	int DynLinePlanFilterStart(HS_GroupJPos &tHS_GroupJPos);

private:
    MotionPara *mMotionPara;	
	HS_GroupKin *m_HS_GroupKin;
	int m_iBaseGroup;
    HS_BasicPara  *m_pHS_BasicPara;
	HS_Int_Factory *m_HS_Int_Joint[2][MAXGROUPNUM];
	HS_Int_Factory *m_HS_Int_Line[2][MAXGROUPNUM];
	HS_Int_Factory *m_HS_Int_SLine[2][MAXGROUPNUM];
    HS_Int_Factory *m_HS_Int_Circle[2][MAXGROUPNUM];

    HS_Int_Factory *m_HS_Int_JointPreH;
    HS_Int_Factory *m_HS_Int_LinePreH;
    HS_Int_Factory *m_HS_Int_SLinePreH;
    HS_Int_Factory *m_HS_Int_CirclePreH;

	HS_Int_Factory *m_HS_Int_Factory[MAXGROUPNUM];
	HS_Int_Factory *m_HS_Int_FactoryPre[MAXGROUPNUM];
	bool m_bRestartAddFlag;
	HS_Int_Factory *m_HS_Int_ReStartAdd[MAXGROUPNUM];
	HS_Kinematics *m_HS_Kinematics;
    FilterControl *m_tFilterControl;
    int m_iAutoFilterType;
    bool m_bFilterDoneFlag;
    bool m_bSLFiterFlag;
	int m_iSLGroup;
    bool m_bSLFilterInitFlag;
    double m_dSLCPos[MaxAxisNum];

	HS_GroupRel m_tHS_GroupRel;
	bool m_bGroupPlanFlag;

	double m_dRestartRatio;
	HS_GroupJPos m_tRestartRealJPos;

    bool m_bPlanFlag;
    int m_iIntIndex;                                //插补规划缓存索引
    HS_GroupJPos m_tMoveDoneJPos;   
	HS_GroupJPos m_tWeaveStopJPos;
	bool m_bWeaveStopFlag[MAXGROUPNUM];
    double m_dSFilterRJPos[MaxAxisNum];

    bool m_bSmoothNextFlag;							//平滑标识位
    bool m_bSmoothPreFlag;  
	bool m_bSmoothJointFlag;						//是否需要下一段输出平滑段Joint

    HS_eStopState m_eStopState; 
    HS_FilterHandle *m_HS_FilterHandle;
    HS_FilterHandle *m_HS_SpaceFilter;
    int m_iInterMultCnt;
    HS_SetPosCheck *m_HS_SetPosCheck[MAXGROUPNUM];
    bool m_bSetPosErrorFlag;
	int m_iGroupIndex[MAXGROUPNUM];					//轴组序号，主动轴在前规划

    int m_iJointBuff[MaxJointBuffCnt];
    int m_iJointBuffIndex;

	bool m_bMoveOnFlag;					
	bool m_bSmoothAheadFlag;	

	int m_iGroupId[MAXGROUPNUM];					//主运动的预处理和规划在前

	int m_iCurId;
	GroupTrajData *m_tTrajData;
};
