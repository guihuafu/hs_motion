#include <iostream>
#include "automove.h"
#include "baseautomove.h"

int main()
{
	std::cout << "Hallo World" << std::endl;
	double dRatio = 0.3;
	int mMotionDataNum = 0;
	hsc3::algo::GroupMotionData groupdata = {0};
	hsc3::algo::GroupStaticPara groupstaticpara[4];
	hsc3::algo::GroupTrajData *mGroupTrajout = new hsc3::algo::GroupTrajData[40];
	memset(mGroupTrajout, 0, sizeof(hsc3::algo::GroupTrajData) * 40);

	////////////////////////////////////////////////////////
	groupstaticpara[0].tGroupModelPara.eRobtype = hsc3::algo::HS_RobotType::HSROB_PUMA;
	groupstaticpara[0].tGroupModelPara.eRobtype_sub = hsc3::algo::HS_RobotType_sub::HSROB_SUB_NONE;
	groupstaticpara[0].tGroupVelocityPara.dVtran = 1700.0;
	groupstaticpara[0].tGroupVelocityPara.dVrot = 50.0;
	groupstaticpara[0].tGroupVelocityPara.dVtranacc = 100.0;
	groupstaticpara[0].tGroupVelocityPara.dVrotacc = 100.0;
	groupstaticpara[0].tGroupVelocityPara.dJerkrat = 9.0;
	groupstaticpara[0].tGroupVelocityPara.dTFreMin = 0.05;
	groupstaticpara[0].tGroupVelocityPara.dTFreMax = 0.4;
	groupstaticpara[0].tGroupModelPara.DHPara[0][0] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[0][1] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[0][2] = -90.0; groupstaticpara[0].tGroupModelPara.DHPara[0][3] = 0.0;
	groupstaticpara[0].tGroupModelPara.DHPara[1][0] = 360.0; groupstaticpara[0].tGroupModelPara.DHPara[1][1] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[1][2] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[1][3] = 0.0;
	groupstaticpara[0].tGroupModelPara.DHPara[2][0] = -90.0; groupstaticpara[0].tGroupModelPara.DHPara[2][1] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[2][2] = 90.0; groupstaticpara[0].tGroupModelPara.DHPara[2][3] = 0.0;
	groupstaticpara[0].tGroupModelPara.DHPara[3][0] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[3][1] = 376.5; groupstaticpara[0].tGroupModelPara.DHPara[3][2] = -90.0; groupstaticpara[0].tGroupModelPara.DHPara[3][3] = 0.0;
	groupstaticpara[0].tGroupModelPara.DHPara[4][0] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[4][1] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[4][2] = 90.0; groupstaticpara[0].tGroupModelPara.DHPara[4][3] = 0.0;
	groupstaticpara[0].tGroupModelPara.DHPara[5][0] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[5][1] = 119.0; groupstaticpara[0].tGroupModelPara.DHPara[5][2] = 0.0; groupstaticpara[0].tGroupModelPara.DHPara[5][3] = 0.0;
	memset(groupstaticpara[0].dWorldCoord, 0, sizeof(double) * 6);
	memset(&groupstaticpara[0].dFlTrackCoord, 0, sizeof(hsc3::algo::HS_Coord));
	memset(&groupstaticpara[0].dPoCoord, 0, sizeof(hsc3::algo::HS_Coord));

	for(int i=0; i<MAXCOORDNUM; i++)
	{
		memset(groupstaticpara[0].dToolCoord[i], 0, sizeof(double) * 6);
		memset(groupstaticpara[0].dWorkCoord[i], 0, sizeof(double) * 6);
	}

	for(int i=0; i<MaxAxisNum; i++)
	{
		groupstaticpara[0].tAxisVelocityPara.dVmax[i] = 250.0;
		groupstaticpara[0].tAxisVelocityPara.dVcruise[i] = 200.0;
		groupstaticpara[0].tAxisVelocityPara.dAccelerate[i] = 2000.0;
		groupstaticpara[0].tAxisVelocityPara.dJerkrat[i] = 400.0;
		groupstaticpara[0].tLimitPara.dPmax[i] = 360.0;
		groupstaticpara[0].tLimitPara.dPmin[i] = -360.0;
		groupstaticpara[0].tLimitPara.bOpen[i] = true;
	}
	////////////////////////////////////////////////////////

	groupdata.iLineNum = 0;
	groupdata.tHS_GroupRel.eGroupRelType[0] = hsc3::algo::GroupRelType::GRT_Independent;
	groupdata.tHS_GroupRel.eGroupRelType[1] = hsc3::algo::GroupRelType::GRT_NoUse;
	groupdata.tHS_GroupRel.eGroupRelType[2] = hsc3::algo::GroupRelType::GRT_NoUse;
	groupdata.tHS_GroupRel.eGroupRelType[3] = hsc3::algo::GroupRelType::GRT_NoUse;
	groupdata.dCnt = 0.0;
	groupdata.dCR = 0.0;
	groupdata.iSmooth = 0;
	groupdata.bExtTool = false;
	groupdata.bStartMove = true;
	groupdata.bWristQYFlag = false;
	groupdata.tFilterControl.bFilterOpenFlag = false;
	groupdata.tBaseMoveData[0].eTrajType = hsc3::algo::HS_MOVETYPE::MP_Joint;
	groupdata.tBaseMoveData[0].sCurCoordinate.iCoordinate = hsc3::algo::COORD_SYSTEM::JOINT_COORD_SYSTEM;
	groupdata.tBaseMoveData[0].sCurCoordinate.iToolNum = -1;
	groupdata.tBaseMoveData[0].sCurCoordinate.iWorkNum = -1;
	groupdata.tBaseMoveData[0].sCurCoordinate.bExtTool = -1;
	groupdata.tBaseMoveData[0].sCurCoordinate.bExtCoorper = -1;
	groupdata.tBaseMoveData[0].sStartPos.dPos[0] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[1] = -90; groupdata.tBaseMoveData[0].sStartPos.dPos[2] = 180;
	groupdata.tBaseMoveData[0].sStartPos.dPos[3] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[4] = 90; groupdata.tBaseMoveData[0].sStartPos.dPos[5] = 0;
	groupdata.tBaseMoveData[0].sStartPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sStartPos.dPos[8] = 0;
	groupdata.tBaseMoveData[0].sStartPos.iPose = 0;
	memset(groupdata.tBaseMoveData[0].sMidPos.dPos, 0, sizeof(double) * 9);
	groupdata.tBaseMoveData[0].sEndPos.dPos[0] = 10; groupdata.tBaseMoveData[0].sEndPos.dPos[1] = -90; groupdata.tBaseMoveData[0].sEndPos.dPos[2] = 180;
	groupdata.tBaseMoveData[0].sEndPos.dPos[3] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[4] = 90; groupdata.tBaseMoveData[0].sEndPos.dPos[5] = 0;
	groupdata.tBaseMoveData[0].sEndPos.dPos[6] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[7] = 0; groupdata.tBaseMoveData[0].sEndPos.dPos[8] = 0;
	groupdata.tBaseMoveData[0].sEndPos.iPose = 0;
	groupdata.tBaseMoveData[0].b2mid = false;
	groupdata.tBaseMoveData[0].dVel = 100.0;
	groupdata.tBaseMoveData[0].dVort = 100.0;
	groupdata.tBaseMoveData[0].dAcc = 100.0;
	groupdata.tBaseMoveData[0].dDec = 100.0;
	groupdata.tBaseMoveData[0].iCntType = 0;
	groupdata.tBaseMoveData[0].tRevolve.iTurn = 0;
	groupdata.tBaseMoveData[0].bCoorperMove = false;
	groupdata.tBaseMoveData[0].tWeavePara.bWeaveEn = false;

	////////////////////////////////////////////////////////

	hsc3::algo::MotionPara *mMotionPara = new hsc3::algo::MotionPara();
	mMotionPara->setGroupStaticPara(groupstaticpara);
	hsc3::algo::AutoMove *mAutoMove = new hsc3::algo::BaseAutoMove(mMotionPara, 0.004, 2);
	
	mAutoMove->execPrehandle(groupdata, mGroupTrajout, mMotionDataNum);
	hsc3::algo::HS_GroupJPos groupjpos = {0};
	memcpy(groupjpos.dJPos[0], groupdata.tBaseMoveData[0].sStartPos.dPos, sizeof(double) * MaxAxisNum);

	mAutoMove->execPlanMove(mGroupTrajout, 0, dRatio, groupjpos);
	
	int errorID = 0;
	hsc3::algo::IntData intdata = {0};
	hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
	while(status != hsc3::algo::M_Done)
	{
		status = mAutoMove->execIntMove(intdata, errorID);    // 获取算法周期插补点
		printf("status=%d, outPos: %f %f %f %f %f %f \n", status, intdata.tGJPos[0].dJPos[0][0],intdata.tGJPos[0].dJPos[0][1],intdata.tGJPos[0].dJPos[0][2],intdata.tGJPos[0].dJPos[0][3],intdata.tGJPos[0].dJPos[0][4],intdata.tGJPos[0].dJPos[0][5]);
	}

	int num;
	std::cin >> num;
}