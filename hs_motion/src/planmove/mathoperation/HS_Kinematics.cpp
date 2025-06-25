
#include "HS_Kinematics.h"

HS_Kinematics::HS_Kinematics(int iGroupNum)
{
	m_HS_BasicPara = HS_BasicPara::GetInstance();
	m_dDHPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tGroupModelPara.DHPara;
	m_eRobotType = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tGroupModelPara.eRobtype;
	m_eRobotType_sub = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tGroupModelPara.eRobtype_sub;
	m_tLimitPara = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tLimitPara;
	m_dToolCoord = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].dToolCoord;
	m_dWorkCoord = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].dWorkCoord;
	m_dJVelPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dVcruise;
	m_dJAccPara = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].tAxisVelocityPara.dAccelerate; 
	m_dWorldCoord = m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].dWorldCoord;
	m_dFlTrackCoord = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].dFlTrackCoord;
	m_dPositionerCoord = &m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[iGroupNum].dPoCoord;
	m_dCycle = m_HS_BasicPara->m_dCycle;
	m_iGroupNum = iGroupNum;

	//初始化参数
	m_iExtNum = 6;
	m_iToolNum = -1;
	m_iWorkNum = -1;
	m_QYKbPara = 0;
	Matrix_Eye(4,&m_dTFMatrix[0][0]);
	Matrix_Eye(4,&m_dFTMatrix[0][0]);
	Matrix_Eye(4,&m_dWBMatrix[0][0]);
	Matrix_Eye(4,&m_dBWMatrix[0][0]);
	Matrix_Eye(4,&m_dEBCoord[0][0]);
	Matrix_Eye(4,&m_dBECoord[0][0]);
	memset(m_dExtCoorper,0,sizeof(m_dExtCoorper));		
	m_bTypeBR = false;
	m_bWristQYHandleFlag = false;
	memset(&m_tQYHandle,0,sizeof(m_tQYHandle));
	m_bScaraA360Flag = true;
	m_dA360BaseAngle = 0;
	m_bCobot6Flag = false;

	//Test();
}

//测试
void HS_Kinematics::Test()
{
    double dCabiT[3][3] = {{0.8489979,-0.0986523,-36.750675},{-0.52794546,-0.033144306,-42.30251},{-0.06672759,-0.99373335,26.707212}};
    double dP[3] = {12.84,-44.98,1};

    double dPTool[6] = {560.4215105,-341.2271481,1277.4708592,30.8207672,-51.5619735,-3.2380238};

    double dPOut[3];
    HS_Math::Matrix_Multi(3,3,1,&dCabiT[0][0],dP,dPOut);

    double dMPos[4][4];
    HS_CPosToMPos(dPTool,dMPos);

    double dAtt[3][3];
    for(int i = 0;i < 3;i++)
        for(int j = 0;j < 3;j++)
            dAtt[i][j] = dMPos[i][j];

    double dPNew[3];
    HS_Math::Matrix_Multi(3,3,1,&dAtt[0][0],dPOut,dPNew);

    dPNew[0] += dMPos[0][3];
    dPNew[1] += dMPos[1][3];
    dPNew[2] += dMPos[2][3];

    double dCabiT2[4][4] = {{0,0.8489979,-0.0986523,-36.750675},{0,-0.52794546,-0.033144306,-42.30251},{0,-0.06672759,-0.99373335,26.707212},{0,0,0,1}};
    double dP2[4] = {0,12.84,-44.98,1};

    double dA = sqrt(1 - dCabiT2[0][1]*dCabiT2[0][1]  - dCabiT2[0][2]*dCabiT2[0][2]);
    double dB = sqrt(1 - dCabiT2[1][1]*dCabiT2[1][1]  - dCabiT2[1][2]*dCabiT2[1][2]);
    double dC = sqrt(1 - dCabiT2[2][1]*dCabiT2[2][1]  - dCabiT2[2][2]*dCabiT2[2][2]);

    double dD = dA*dA + dB*dB + dC*dC;

    dCabiT2[0][0] = dA;
    dCabiT2[1][0] = dB;
    dCabiT2[2][0] = dC;

    double dPOut2[4];
    double dPNew2[4];
    HS_Math::Matrix_Multi(4,4,1,&dCabiT2[0][0],dP2,dPOut2);
    HS_Math::Matrix_Multi(4,4,1,&dMPos[0][0],dPOut2,dPNew2);

    double dP3[6] = {0,12.84,-44.98,0,0,0};
    double dPMPos[4][4];
    double dPOut3[4][4];
    double dPNew3[4][4];
    HS_CPosToMPos(dP3,dPMPos);
    HS_Math::Matrix_Multi(4,4,4,&dCabiT2[0][0],&dPMPos[0][0],&dPOut3[0][0]);
    HS_Math::Matrix_Multi(4,4,4,&dMPos[0][0],&dPOut3[0][0],&dPNew3[0][0]);

    double dPOutNew[6];
    HS_MPosToCPos(dPNew3,dPOutNew);

    double dTool[6] = {45.311,11.786,383.59,26.715,-42.279,-23.469};
    double dToolMPos[4][4] = {0};
    HS_CPosToMPos(dTool,dToolMPos);

    HS_Math::Matrix_Multi(4,4,4,&dToolMPos[0][0],&dCabiT2[0][0],&dPNew3[0][0]);

    HS_MPosToCPos(dPNew3,dPOutNew);
}

HS_Kinematics::~HS_Kinematics()
{
}

/******************************************************
函数功能：打印系统内部关键参数
参数：	
******************************************************/
int HS_Kinematics::PrintKeyInfo()
{
    LOG_ALGO("---------------------DH-------------------");
    for(int i = 0;i < 6;i++)
    {
        LOG_ALGO("DHPara[%d][0] = %.3lf,DHPara[%d][1] = %.3lf,DHPara[%d][2] = %.3lf,DHPara[%d][3] = %.3lf",
            i,m_dDHPara[i][0],i,m_dDHPara[i][1],i,m_dDHPara[i][2],i,m_dDHPara[i][3]);
    }

    LOG_ALGO("JAcc--Set:%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf---%.0lf,%.0lf,%.0lf;",\
        m_dJAccPara[0],m_dJAccPara[1],m_dJAccPara[2],m_dJAccPara[3],m_dJAccPara[4],m_dJAccPara[5],
        m_dJAccPara[6],m_dJAccPara[7],m_dJAccPara[8]);

    InitPara();

    LOG_ALGO("eRobotType = %d,RobotTYpeSub = %d,Cobot = %d,Vision = %s",*m_eRobotType,*m_eRobotType_sub,(int)m_bCobot6Flag,m_HS_BasicPara->mMotionPara->m_strVision.c_str());
    LOG_ALGO("PMAX:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tLimitPara->dPmax[0],m_tLimitPara->dPmax[1],m_tLimitPara->dPmax[2],m_tLimitPara->dPmax[3],m_tLimitPara->dPmax[4],m_tLimitPara->dPmax[5],\
        m_tLimitPara->dPmax[6],m_tLimitPara->dPmax[7],m_tLimitPara->dPmax[8]);
    LOG_ALGO("PMIN:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf---%.3lf,%.3lf,%.3lf",\
        m_tLimitPara->dPmin[0],m_tLimitPara->dPmin[1],m_tLimitPara->dPmin[2],m_tLimitPara->dPmin[3],m_tLimitPara->dPmin[4],m_tLimitPara->dPmin[5],\
        m_tLimitPara->dPmin[6],m_tLimitPara->dPmin[7],m_tLimitPara->dPmin[8]);
    LOG_ALGO("POPEN:%d,%d,%d,%d,%d,%d---%d,%d,%d",\
        (int)m_tLimitPara->bOpen[0],(int)m_tLimitPara->bOpen[1],(int)m_tLimitPara->bOpen[2],(int)m_tLimitPara->bOpen[3],(int)m_tLimitPara->bOpen[4],(int)m_tLimitPara->bOpen[5],\
        (int)m_tLimitPara->bOpen[6],(int)m_tLimitPara->bOpen[7],(int)m_tLimitPara->bOpen[8]);
    LOG_ALGO("QYLimit:Inter = %.3lf,Border = %.3lf,Wrist = %.3lf;BorderQYOff = %.3lf",\
            m_tLimitPara->dQYPara_Inter,m_tLimitPara->dQYPara_Border,m_tLimitPara->dQYPara_Wrist,m_QYKbPara);
    LOG_ALGO("JVel:%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf---%.0lf,%.0lf,%.0lf",\
        m_dJVelPara[0],m_dJVelPara[1],m_dJVelPara[2],m_dJVelPara[3],m_dJVelPara[4],m_dJVelPara[5],
        m_dJVelPara[6],m_dJVelPara[7],m_dJVelPara[8]);
	LOG_ALGO("JVelMax:%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf---%.0lf,%.0lf,%.0lf",\
		m_dJVelPara[0]*1.2,m_dJVelPara[1]*1.2,m_dJVelPara[2]*1.2,m_dJVelPara[3]*1.2,m_dJVelPara[4]*1.2,m_dJVelPara[5]*1.2,
		m_dJVelPara[6]*1.2,m_dJVelPara[7]*1.2,m_dJVelPara[8]*1.2);
    LOG_ALGO("JAcc:%.0lf,%.0lf,%.0lf,%.0lf,%.0lf,%.0lf---%.0lf,%.0lf,%.0lf",\
        m_dJAccPara[0],m_dJAccPara[1],m_dJAccPara[2],m_dJAccPara[3],m_dJAccPara[4],m_dJAccPara[5],
        m_dJAccPara[6],m_dJAccPara[7],m_dJAccPara[8]);
    LOG_ALGO("WorldCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",\
        m_dWorldCoord[0],m_dWorldCoord[1],m_dWorldCoord[2],m_dWorldCoord[3],m_dWorldCoord[4],m_dWorldCoord[5]);

	LOG_ALGO("TFreLimit:Min = %.3lf,Max = %.3lf;Cycle = %.3lf,InterCnt = %d",\
		m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[m_iGroupNum].tGroupVelocityPara.dTFreMin,m_HS_BasicPara->mMotionPara->m_tGroupStaticPara[m_iGroupNum].tGroupVelocityPara.dTFreMax,\
		m_HS_BasicPara->m_dCycle,m_HS_BasicPara->m_iInterMultCnt);
	//地轨以及变位机

	LOG_ALGO("FlTrackCoord_0:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dFlTrackCoord->dExtCoord[0][0],m_dFlTrackCoord->dExtCoord[0][1],m_dFlTrackCoord->dExtCoord[0][2],m_dFlTrackCoord->dExtCoord[0][3],m_dFlTrackCoord->dExtCoord[0][4],m_dFlTrackCoord->dExtCoord[0][5],m_dFlTrackCoord->iAxisId[0]);
	LOG_ALGO("FlTrackCoord_1:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dFlTrackCoord->dExtCoord[1][0],m_dFlTrackCoord->dExtCoord[1][1],m_dFlTrackCoord->dExtCoord[1][2],m_dFlTrackCoord->dExtCoord[1][3],m_dFlTrackCoord->dExtCoord[1][4],m_dFlTrackCoord->dExtCoord[1][5],m_dFlTrackCoord->iAxisId[1]);
	LOG_ALGO("FlTrackCoord_2:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dFlTrackCoord->dExtCoord[2][0],m_dFlTrackCoord->dExtCoord[2][1],m_dFlTrackCoord->dExtCoord[2][2],m_dFlTrackCoord->dExtCoord[2][3],m_dFlTrackCoord->dExtCoord[2][4],m_dFlTrackCoord->dExtCoord[2][5],m_dFlTrackCoord->iAxisId[2]);

	LOG_ALGO("CoorperCoord_0:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dPositionerCoord->dExtCoord[0][0],m_dPositionerCoord->dExtCoord[0][1],m_dPositionerCoord->dExtCoord[0][2],m_dPositionerCoord->dExtCoord[0][3],m_dPositionerCoord->dExtCoord[0][4],m_dPositionerCoord->dExtCoord[0][5],m_dPositionerCoord->iAxisId[0]);
	LOG_ALGO("CoorperCoord_1:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dPositionerCoord->dExtCoord[1][0],m_dPositionerCoord->dExtCoord[1][1],m_dPositionerCoord->dExtCoord[1][2],m_dPositionerCoord->dExtCoord[1][3],m_dPositionerCoord->dExtCoord[1][4],m_dPositionerCoord->dExtCoord[1][5],m_dPositionerCoord->iAxisId[1]);
	LOG_ALGO("CoorperCoord_2:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;Axis = %d",\
		m_dPositionerCoord->dExtCoord[2][0],m_dPositionerCoord->dExtCoord[2][1],m_dPositionerCoord->dExtCoord[2][2],m_dPositionerCoord->dExtCoord[2][3],m_dPositionerCoord->dExtCoord[2][4],m_dPositionerCoord->dExtCoord[2][5],m_dPositionerCoord->iAxisId[2]);

    FilterControl tFilterControl = m_HS_BasicPara->m_tSysFilterPara.tHandControl;
	tFilterControl.bFilterOpenFlag = false;
	tFilterControl.tFilterPara.iGrade = 9;
    LOG_ALGO("Firter Hand:Open = %d,Type = %d,Fre = %.3lf,Grade = %d",\
        (int)(tFilterControl.bFilterOpenFlag),tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade);
    tFilterControl = m_HS_BasicPara->m_tSysFilterPara.tAutoControl;
    LOG_ALGO("Firter Auto:Open = %d,Type = %d,Fre = %.3lf,Grade = %d,Type = %d",\
        (int)(tFilterControl.bFilterOpenFlag),tFilterControl.tFilterPara.eFilterType,tFilterControl.tFilterPara.dFre,tFilterControl.tFilterPara.iGrade,\
        m_HS_BasicPara->m_tSysFilterPara.iAutoFilterType);


    return 0;
}
/************************************************
函数功能：打印特定的工具工件值
参    数：
返 回 值：无
*************************************************/
int HS_Kinematics::HS_PrintCoord(int iTool,int iWork)
{
    int iToolNum = iTool + 1;
    int iWorkNum = iWork + 1;

    if(iToolNum >= MAXCOORDNUM||iWorkNum >= MAXCOORDNUM)
    {
        LOG_ALGO("ERROR0 COORNUM");
        return ERROR_COORDNUM;
    }

    LOG_ALGO("-------Print CoorDinate:Tool=%d;iWord=%d--------",iTool,iWork);
    LOG_ALGO("ToolCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dToolCoord[iToolNum][0],m_dToolCoord[iToolNum][1],
        m_dToolCoord[iToolNum][2],m_dToolCoord[iToolNum][3],m_dToolCoord[iToolNum][4],m_dToolCoord[iToolNum][5]);
    LOG_ALGO("WorkCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dWorkCoord[iWorkNum][0],m_dWorkCoord[iWorkNum][1],
        m_dWorkCoord[iWorkNum][2],m_dWorkCoord[iWorkNum][3],m_dWorkCoord[iWorkNum][4],m_dWorkCoord[iWorkNum][5]);
    return 0;
}
/************************************************
函数功能：关联参数的初始化
参    数：
返 回 值：无
*************************************************/
void HS_Kinematics::InitPara()
{
    const double dTAcc = 0.3;
    for(int i = 0;i < MaxAxisNum;i++)
    {
        if(m_dJVelPara[i] < Eps)
            m_dJVelPara[i] = 1.0;
        if(m_dJAccPara[i] < Eps)
        {
            m_dJAccPara[i] = m_dJVelPara[i]/dTAcc*1.5;
        }
    }
    //--------------参数设置-------------------------------------------
    m_bTypeBR = false;
    if(*m_eRobotType == HSROB_PUMA)
    {
		const double MaxBR	= 1.0;		//区别BR系列和JR系列机型
		if(fabs(m_dDHPara[4][1]) > MaxBR||fabs(m_dDHPara[4][0]) > MaxBR)
		{
			m_bTypeBR = true;     
		}
    }
    m_QYKbPara = 0;
    if(*m_eRobotType == HSROB_PUMA)
    {
        if(m_bTypeBR)
            m_QYKbPara = 0;
        else
        {
            //求解边界奇异参数，只与3号轴的位置有关，值越接近0，就越靠近奇异位置
            m_QYKbPara = atan2(fabs(m_dDHPara[2][0]),fabs(m_dDHPara[3][1]))*180/PI;
        }

		if(*m_eRobotType_sub == Cobot6)
		{
			m_bCobot6Flag = true;
			m_QYKbPara = 0;
		}
    }

    if(m_tLimitPara->dQYPara_Inter < Eps||m_tLimitPara->dQYPara_Inter > 1000)
        m_tLimitPara->dQYPara_Inter = 100.0;
    if(m_tLimitPara->dQYPara_Border < Eps||m_tLimitPara->dQYPara_Border > 100)
        m_tLimitPara->dQYPara_Border = 10.0;
    //if(m_tLimitPara->dQYPara_Wrist < Eps||m_tLimitPara->dQYPara_Wrist > 50)
    //    m_tLimitPara->dQYPara_Wrist = 2.0;
	if(m_tLimitPara->dQYPara_Wrist < -Eps||m_tLimitPara->dQYPara_Wrist > 50)
		m_tLimitPara->dQYPara_Wrist = 2.0;
}
/************************************************
函数功能：设置手动腕部过奇异功能开关
参    数：
         bOpenFlag----是否开启腕部过奇异功能
返 回 值：无
*************************************************/
void HS_Kinematics::HS_SetManualWristQY(bool bOpenFlag,double dRealPos[MaxAxisNum])
{
	m_bWristQYHandleFlag = bOpenFlag;

	//增加处理，如果CO协作系列，起点位置远离腕部奇异则不开启，避免影响正常运动的姿态精度
	if(m_bCobot6Flag || m_bTypeBR)
	{
		const double dWristLimit = 5.0;
		if(fabs(dRealPos[4]) > dWristLimit)
			m_bWristQYHandleFlag = false;
	}    
    LOG_ALGO("WristQYHandle = %d",(int)m_bWristQYHandleFlag);
}
/************************************************
函数功能：复位奇异处理参数
参    数：
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_ResetQYHandle()
{
    memset(&m_tQYHandle,0,sizeof(m_tQYHandle));
    return 0;
}
/************************************************
函数功能：设置工具工件坐标系【公共使用】
参    数：iToolNum-----工具坐标系编号(0~16)
		 iWorkNum-----工件坐标系编号
         【参数修改，范围改为-1~15】
         iWorkNum为-2代表基坐标系，其余均在世界坐标系下计算
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_SetCoordinate(int iToolNum,int iWorkNum)
{
    int iTool = iToolNum + 1;
    int iWork = iWorkNum + 1;

	if(iTool < 0||iTool > 16)
		return ERROR_COORDNUM;
	if(iWork < -1||iWork > 16)
		return ERROR_COORDNUM;

	//if(m_iToolNum != iToolNum||m_iWorkNum != iWorkNum)
	{
		LOG_ALGO("-------CoorDinate--------");
		LOG_ALGO("iToolNum = %d,iWorkNum = %d",iToolNum,iWorkNum);	
		LOG_ALGO("ToolCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dToolCoord[iTool][0],m_dToolCoord[iTool][1],
			m_dToolCoord[iTool][2],m_dToolCoord[iTool][3],m_dToolCoord[iTool][4],m_dToolCoord[iTool][5]);
        if(iWork >= 0)
        {
		    LOG_ALGO("WorkCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dWorkCoord[iWork][0],m_dWorkCoord[iWork][1],
			    m_dWorkCoord[iWork][2],m_dWorkCoord[iWork][3],m_dWorkCoord[iWork][4],m_dWorkCoord[iWork][5]);
        }
        else
        {
            LOG_ALGO("Work Set Base!");
        }
	}

	//求工具坐标系
	HS_CPosToMPos(m_dToolCoord[iTool],m_dTFMatrix);
	//工具相对法兰盘变换	
	Matrix_Inverse(4,&m_dTFMatrix[0][0],&m_dFTMatrix[0][0]);
	
    if(iWork == -1)
    {
	    //求工件坐标系
	    HS_CPosToMPos(m_dWorkCoord[0],m_dWBMatrix);
	    //工件相对于基坐标	
	    Matrix_Inverse(4,&m_dWBMatrix[0][0],&m_dBWMatrix[0][0]);	
    }
    else
    {
        double dWorldMatrix[4][4] = {0};
        HS_CPosToMPos(m_dWorldCoord,dWorldMatrix);

        double dWorkWorldMatrix[4][4] = {0};
        HS_CPosToMPos(m_dWorkCoord[iWork],dWorkWorldMatrix);  

        Matrix_Multi(4,4,4,&dWorldMatrix[0][0],&dWorkWorldMatrix[0][0],&m_dWBMatrix[0][0]);

        Matrix_Inverse(4,&m_dWBMatrix[0][0],&m_dBWMatrix[0][0]);
    }

	m_iToolNum = iToolNum;
	m_iWorkNum = iWorkNum;
	return 0;
}

int HS_Kinematics::HS_PrintCoordinate(int iToolNum,int iWorkNum)
{
	int iTool = iToolNum + 1;
	int iWork = iWorkNum + 1;
	LOG_ALGO("-------Print CoorDinate--------");
	LOG_ALGO("iToolNum = %d,iWorkNum = %d",iToolNum,iWorkNum);	
	LOG_ALGO("ToolCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dToolCoord[iTool][0],m_dToolCoord[iTool][1],
		m_dToolCoord[iTool][2],m_dToolCoord[iTool][3],m_dToolCoord[iTool][4],m_dToolCoord[iTool][5]);

	if(iWork >= 0)
	{
		LOG_ALGO("WorkCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dWorkCoord[iWork][0],m_dWorkCoord[iWork][1],
			m_dWorkCoord[iWork][2],m_dWorkCoord[iWork][3],m_dWorkCoord[iWork][4],m_dWorkCoord[iWork][5]);
	}
	else
	{
		LOG_ALGO("Work Set Base!");
	}
	return 0;
}
/************************************************
函数功能：设置工具工件坐标系--工件坐标系为当前工具坐标系
参    数：iToolNum-----工具坐标系编号(0~16)
		 pdJPos-------当前关节位置
         【参数修改，范围改为-1~15】
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_SetTCoordinate(int iToolNum,double dJPos[6])
{
    int iTool = iToolNum + 1;
	if(iTool < 0||iTool > 16)
		return ERROR_COORDNUM;

	LOG_ALGO("-------CoorDinate--------");
	LOG_ALGO("iToolNum = %d",iToolNum);
	LOG_ALGO("ToolCoord:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",m_dToolCoord[iTool][0],m_dToolCoord[iTool][1],
		m_dToolCoord[iTool][2],m_dToolCoord[iTool][3],m_dToolCoord[iTool][4],m_dToolCoord[iTool][5]);
	

    //求工具坐标系
    HS_CPosToMPos(m_dToolCoord[iTool],m_dTFMatrix);
    //工具相对法兰盘变换	
    Matrix_Inverse(4,&m_dTFMatrix[0][0],&m_dFTMatrix[0][0]);

	m_iToolNum = iToolNum;

	double dFBMatrix[5][4];				//法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T
	double dTBMatrix[4][4];				//工具点到基础坐标系的变换矩阵  B/T	
	HS_JPosToMPos(dJPos,dFBMatrix);		//法兰盘相对于基坐标 B/F	
	Matrix_Multi(4,4,4,&dFBMatrix[0][0],&m_dTFMatrix[0][0],&dTBMatrix[0][0]);
	memcpy(m_dWBMatrix,dTBMatrix,sizeof(m_dWBMatrix));
	Matrix_Inverse(4,&m_dWBMatrix[0][0],&m_dBWMatrix[0][0]);
	return 0;
}
/************************************************
函数功能：由三个空间点位计算出对应圆心
参    数：p1---第一个点	 
		 p2---第二个点
		 p3---第三个点
		 p4---圆心所在空间位置
返 回 值：圆心计算是否成功
*************************************************/
bool HS_Kinematics::CalCenter(double *p1,double *p2,double *p3,double *center)
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
	if(!Matrix_Inverse(3,&m[0][0],&m_inv[0][0]))
		return false;
	double C[3];
	Matrix_Multi(3,3,1,&m_inv[0][0],D,C);
	center[0] = -C[0];
	center[1] = -C[1];
	center[2] = -C[2];
	return true;
}
/************************************************
函数功能：关节坐标转换空间坐标【空间坐标值】	
参    数：pdJPos---关节角度
		 eCPType--转换的坐标系类型
		 pdCPos---空间坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToCPos(double dJPos[MaxAxisNum],CPType eCPType,double dCPos[MaxAxisNum])
{
	int iErrorId = 0;
	double dMPos[5][4] = {0};
	iErrorId = HS_JPosToMPos(dJPos,eCPType,dMPos);
	iErrorId = HS_MPosToCPos(dMPos,dCPos);
	dCPos[6] = dJPos[6];
	dCPos[7] = dJPos[7];
	dCPos[8] = dJPos[8];
	return iErrorId;	
}
/************************************************
函数功能：关节坐标转换空间坐标【空间坐标值】指定工具工件	
参    数：pdJPos---关节角度
		 iToolNum--工具编号
		 iWorkNum--工件编号
		 dMPos---矩阵坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToMPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dMPos[5][4])
{
	double dFBMPos[5][4] = {0};

	if(*m_eRobotType == HSROB_COORPER)
	{
		HS_JPosToPoMPos(dJPos,dMPos);
		return 0;
	}

	int iErrorId = HS_JPosToMPos(dJPos,dFBMPos);
	HS_FBMPosToTWMPos(dFBMPos,iToolNum,iWorkNum,dMPos);

	return iErrorId;
}
/************************************************
函数功能：关节坐标转换空间坐标【空间坐标值】指定工具工件	
参    数：pdJPos---关节角度
		 eCPType--转换的坐标系类型
		 pdCPos---空间坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToCPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dCPos[MaxAxisNum])
{
	double dMPos[5][4] = {0};
	int iErrorId = HS_JPosToMPos(dJPos,iToolNum,iWorkNum,dMPos);
	iErrorId = HS_MPosToCPos(dMPos,dCPos);	

	dCPos[6] = dJPos[6];
	dCPos[7] = dJPos[7];
	dCPos[8] = dJPos[8];
	return iErrorId;
}
/************************************************
函数功能：关节坐标转换空间坐标（提供给上层显示使用）	
参    数：pdJPos---关节角度
		 pdLJPos--上个周期的关节角度
		 iToolNum--工具号
		 iWorkNum--工件号
		 dFCPos---空间坐标，法兰盘在基坐标位置	
		 dTWCPos---空间坐标，工具点在工件中位置
		 bExtCoorper---外部地轨坐标系标识
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToCPos(double dJPos[MaxAxisNum],int iToolNum,int iWorkNum,double dFBCPos[6],double dTWCPos[6],bool bExtCoorper)
{
	int iErrorId = 0;
    
    iErrorId = HS_JPosToCPos(dJPos,-1,-1,dFBCPos);
    if(iErrorId != 0)   return iErrorId;

    iErrorId = HS_JPosToCPos(dJPos,iToolNum,iWorkNum,dTWCPos);

	return iErrorId;
}

/************************************************
函数功能：根据当前关节角度求解位姿类型
参    数：pdCurJPos---当前关节角度	
返 回 值：姿态信息	 
*************************************************/
unsigned char HS_Kinematics::HS_JPosToAState(double dJPos[6])
{
    unsigned char nATState = 0;
	double dJTemp[6] = {0};

	memcpy(dJTemp,dJPos,sizeof(double)*6);

    //对于六轴机型形态的判别
    if(*m_eRobotType == HSROB_PUMA)
    {
		double dNewDH[6][4];				//DH参数表 
		memcpy(dNewDH,m_dDHPara,sizeof(dNewDH));

		if(m_bCobot6Flag)
		{
			dJTemp[2] += 90;
			dNewDH[0][0] = 0;               dNewDH[0][1] = m_dDHPara[0][1];	    dNewDH[0][2] = -90;
			dNewDH[1][0] = m_dDHPara[1][0]; dNewDH[1][1] = m_dDHPara[2][1];		dNewDH[1][2] = 0;
			dNewDH[2][0] = 0;               dNewDH[2][1] = 0;		            dNewDH[2][2] = 90;
			dNewDH[3][0] = 0;	            dNewDH[3][1] = (m_dDHPara[2][0]-m_dDHPara[4][1]);	    dNewDH[3][2] = -90;
			dNewDH[4][0] = 0;	            dNewDH[4][1] = 0;	                dNewDH[4][2] = 90;
			dNewDH[5][0] = 0;	            dNewDH[5][1] = m_dDHPara[5][1];	    dNewDH[5][2] = 0;
		}
        //1、内部奇异位置判断臂的形态
        //以臂面（此时2、3轴顺时针旋转角度减小）的方向看过去，腕部中心点在1轴轴线的左边为反手
        //在右边为正手
        //齐次变换矩阵
        //不考虑此时1轴的位置，求解出腕部中心点对应的X轴坐标，为正代表正手，为负代表反手
        double A1[16],A2[16],A3[16],A4[16];
        HS_TransformPlus(A1,0,dNewDH[0][0],dNewDH[0][1],dNewDH[0][2],dNewDH[0][3]);
        HS_TransformPlus(A2,dJTemp[1],dNewDH[1][0],dNewDH[1][1],dNewDH[1][2],dNewDH[0][3]);
        HS_TransformPlus(A3,dJTemp[2],dNewDH[2][0],dNewDH[2][1],dNewDH[2][2],dNewDH[0][3]);
        HS_TransformPlus(A4,dJTemp[3],dNewDH[3][0],dNewDH[3][1],dNewDH[3][2],dNewDH[0][3]);
        double J2[16],J3[16],J4[4][4];
        Matrix_Multi(4,4,4,A1,A2,J2);	
        Matrix_Multi(4,4,4,J2,A3,J3);
        Matrix_Multi(4,4,4,J3,A4,&J4[0][0]);

        if(J4[0][3] > 0)	//正手
            nATState |= AT_Front;
        else				//反手
            nATState |= AT_Back;

        //2、边界奇异位置判断肘的形态
        if(dJTemp[2] >= 90 + m_QYKbPara &&dJTemp[2] <= (90 + 180 + m_QYKbPara))
            nATState |= AT_Above;
        else
            nATState |= AT_Below;

        //3、腕部奇异位置判断腕的形态
		//增加处理对大于180和小于-180进行处理
        if((dJTemp[4] <= 0&&dJTemp[4] > -180)||dJTemp[4] >= 180)
            nATState |= AT_Flip;
        else
            nATState |= AT_NonFlip;
    }  
    //四轴机型形态的判别
    else if(*m_eRobotType == HSROB_SCARA || *m_eRobotType == HSROB_SCARA_3)
    {
        //2号轴的位置判断肘的形态
        if(dJTemp[1] >= 0)
            nATState |= AT_Right;
        else
            nATState |= AT_Left;
    }
    else if(*m_eRobotType == HSROB_LINK_3)
    {
        if(dJTemp[1] >= 0)
            nATState |= AT_Right;
        else
            nATState |= AT_Left;
    }

    return nATState;
}
/************************************************
函数功能：关节坐标转换空间齐次坐标【M矩阵】	
参    数：dJPos---关节角度
		 eCPType--转换的坐标系类型
		 dMPos---空间齐次坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToMPos(double dJPos[MaxAxisNum],CPType eCPType,double dMPos[5][4])
{
	int iErrorId = 0;	
	double dFBMPos[5][4] = {0};
	iErrorId = HS_JPosToMPos(dJPos,dFBMPos);
	HS_FBMPosToTWMPos(dFBMPos,eCPType,dMPos);
	return 0;
}
/************************************************
函数功能：将法兰盘基坐标系下的点位坐标转换为工具工件的坐标值	
参    数：dFBMPos---法兰盘基坐标系【输入】
		 eCPType---转换的坐标系类型
		 dTWMPos---工具工件坐标系【输出】	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_FBMPosToTWMPos(double dFBMPos[4][4],CPType eCPType,double dTWMPos[4][4])
{
	switch(eCPType)
	{
	case CP_ToolWork:	//工具点在工件坐标系中的位置
		{
			//法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T			
			double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
			//工具点到工件坐标系的变换矩阵  W/T			
			Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dTFMatrix[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&m_dBWMatrix[0][0],&dTBMatrix[0][0],&dTWMPos[0][0]);

            if(*m_eRobotType == HSROB_SCARA)
            {
                double dToolAOffset = m_dToolCoord[m_iToolNum][3];
                double dWorkAOffset = m_dWorldCoord[3] + m_dWorkCoord[m_iWorkNum][3];

                //ScaraA360 A角角度的求解，工具的方向一致，工件的方向相反
                m_dA360BaseAngle = m_dA360BaseAngle + dToolAOffset - dWorkAOffset;
            }

			/*m_HS_Printer->outDebugInfo("Motion_P","Line","HS_JPosToMPos",0,AllDeb,"TFMatrix[X-Z]:%.6lf,%.6lf,%.6lf",
				m_dTFMatrix[0][3],m_dTFMatrix[1][3],m_dTFMatrix[2][3]);
			m_HS_Printer->outDebugInfo("Motion_P","Line","HS_JPosToMPos",0,AllDeb,"BWMatrix[X-Z]:%.6lf,%.6lf,%.6lf",
				m_dBWMatrix[0][3],m_dBWMatrix[1][3],m_dBWMatrix[2][3]);*/
			break;
		}
	case CP_WorkTool:	//工件点在工具坐标系中的位置
		{
			//法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T			
			double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
			double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T
			//double dWTMatrix[4][4];	//工件点到工具坐标系的变换矩阵  T/W			
			Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dWBMatrix[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&m_dFTMatrix[0][0],&dTBMatrix[0][0],&dTWMatrix[0][0]);
			Matrix_Inverse(4,&dTWMatrix[0][0],&dTWMPos[0][0]);			
			break;
		}
	case CP_Flange:		//法兰盘在基础坐标系中位置
		memcpy(dTWMPos,dFBMPos,sizeof(double)*16);
		break;
	case CP_ToolCoord:
		{
			//double dFBMatrix[4][4];	//法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T			
			double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
			//double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T			
			Matrix_Multi(4, 4, 4, &dFBMPos[0][0], &m_dTFMatrix[0][0], &dTBMatrix[0][0]);
			double dToolCoord_inv[4][4];
			//Matrix_Inverse(4, &m_dMOuterWork[0][0], &dToolCoord_inv[0][0]);
			Matrix_Multi(4, 4, 4, &dToolCoord_inv[0][0], &dTBMatrix[0][0], &dTWMPos[0][0]);			
			break;
		}
	case CP_EterToolWork:
		{
			double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
			Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dWBMatrix[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&m_dFTMatrix[0][0],&dTBMatrix[0][0],&dTWMPos[0][0]);//这个值真正意义是外部工件相对于外部工具的值
			break;
		}
    case CP_ExtCoorper:
        {
            //外部轴默认7号轴，单位为mm，对应外部协同坐标系的X轴
            double dBECoord[4][4] = {0};
            memcpy(dBECoord,m_dBECoord,sizeof(dBECoord));

            dBECoord[0][3] = 0;         //设置附加轴为0，用来做空间点动

            //法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T		
            double dTEMatrix[4][4];	            

            double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
            //工具点到工件坐标系的变换矩阵  W/T			
            Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dTFMatrix[0][0],&dTBMatrix[0][0]);

            Matrix_Multi(4,4,4,&dBECoord[0][0],&dTBMatrix[0][0],&dTEMatrix[0][0]);          // E/B * B/T = E/T   

            Matrix_Multi(4,4,4,&m_dBWMatrix[0][0],&dTEMatrix[0][0],&dTWMPos[0][0]);
        }
        break;
	default:
		break;
	}
	return 0;
}
/************************************************
函数功能：将工具工件的坐标值	转换为法兰盘基坐标系下的点位坐标
参    数：dTWMPos---工具工件坐标系【输入】
		 iToolNum---工具号
		 iWorkNum---工件号
		 dFBMPos---法兰盘基坐标系【输出】	 
		 dTBMPos---工具基坐标系【输出】
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_TWMPosToFBMPos(double dTWMPos[4][4],int iToolNum,int iWorkNum,double dFBMPos[4][4])
{
	double dTBMPos[4][4] = {0};

	HS_TWMPosToFBMPos(dTWMPos,iToolNum,iWorkNum,dFBMPos,dTBMPos);
	return 0;
}
int HS_Kinematics::HS_TWMPosToFBMPos(double dTWMPos[4][4],int iToolNum,int iWorkNum,double dFBMPos[4][4],double dTBMPos[4][4])
{
	int iTool = iToolNum + 1;
	int iWork = iWorkNum + 1;

	double dTFMatrix[4][4] = {0};
	double dFTMatrix[4][4] = {0};
	double dWBMatrix[4][4] = {0};
	double dBWMatrix[4][4] = {0};

	HS_CPosToMPos(m_dToolCoord[iTool],dTFMatrix);
	Matrix_Inverse(4,&dTFMatrix[0][0],&dFTMatrix[0][0]);

	memcpy(m_dTFMatrix_T,dTFMatrix,sizeof(double)*16);

	double dWorldMatrix[4][4] = {0};
	HS_CPosToMPos(m_dWorldCoord,dWorldMatrix);
	double dWorkWorldMatrix[4][4] = {0};
	HS_CPosToMPos(m_dWorkCoord[iWork],dWorkWorldMatrix);  
	Matrix_Multi(4,4,4,&dWorldMatrix[0][0],&dWorkWorldMatrix[0][0],&dWBMatrix[0][0]);

	Matrix_Inverse(4,&dWBMatrix[0][0],&dBWMatrix[0][0]);

	//TWPosA--->FBPosA
	Matrix_Multi(4,4,4,&dWBMatrix[0][0],&dTWMPos[0][0],&dTBMPos[0][0]);
	Matrix_Multi(4,4,4,&dTBMPos[0][0],&dFTMatrix[0][0],&dFBMPos[0][0]);

	if(*m_eRobotType == HSROB_SCARA)
	{
		double dToolAOffset = m_dToolCoord[iTool][3];
		double dWorkAOffset = m_dWorldCoord[3] + m_dWorkCoord[iWork][3];

		//ScaraA360 A角角度的求解，工具的方向一致，工件的方向相反
		m_dA360TWOffset = dToolAOffset - dWorkAOffset;
	}
	return 0;
}
/************************************************
函数功能：将法兰盘基坐标系下的点位坐标转换为工具工件的坐标值	
参    数：dFBMPos---法兰盘基坐标系【输入】
		 iToolNum---工具号
		 iWorkNum---工件号  -1代表世界坐标系
		 dTWMPos---工具工件坐标系【输出】	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_FBMPosToTWMPos(double dFBMPos[4][4],int iToolNum,int iWorkNum,double dTWMPos[4][4])
{
    int iTool = iToolNum + 1;
    int iWork = iWorkNum + 1;

	double dTFMatrix[4][4] = {0};
	double dFTMatrix[4][4] = {0};
	double dWBMatrix[4][4] = {0};
	double dBWMatrix[4][4] = {0};

	HS_CPosToMPos(m_dToolCoord[iTool],dTFMatrix);
	Matrix_Inverse(4,&dTFMatrix[0][0],&dFTMatrix[0][0]);

    double dWorldMatrix[4][4] = {0};
    HS_CPosToMPos(m_dWorldCoord,dWorldMatrix);
    double dWorkWorldMatrix[4][4] = {0};
    HS_CPosToMPos(m_dWorkCoord[iWork],dWorkWorldMatrix);  
    Matrix_Multi(4,4,4,&dWorldMatrix[0][0],&dWorkWorldMatrix[0][0],&dWBMatrix[0][0]);

	Matrix_Inverse(4,&dWBMatrix[0][0],&dBWMatrix[0][0]);

    //法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T			
    double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
    //工具点到工件坐标系的变换矩阵  W/T			
    Matrix_Multi(4,4,4,&dFBMPos[0][0],&dTFMatrix[0][0],&dTBMatrix[0][0]);
    Matrix_Multi(4,4,4,&dBWMatrix[0][0],&dTBMatrix[0][0],&dTWMPos[0][0]);

    if(*m_eRobotType == HSROB_SCARA)
    {
        double dToolAOffset = m_dToolCoord[iTool][3];
        double dWorkAOffset = m_dWorldCoord[3] + m_dWorkCoord[iWork][3];

        //ScaraA360 A角角度的求解，工具的方向一致，工件的方向相反
        m_dA360BaseAngle = m_dA360BaseAngle + dToolAOffset - dWorkAOffset;
    }
	return 0;
}
/************************************************
函数功能：将法兰盘基坐标系下的点位坐标转换为工具工件的坐标值【地轨坐标系下】	
参    数：dFBMPos---法兰盘基坐标系【输入】
		 iToolNum---工具号
		 iWorkNum---工件号
		 dTWMPos---工具工件坐标系【输出】	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_FBMPosToTWMPos_ExtCoorper(double dFBMPos[4][4],int iToolNum,int iWorkNum,double dTWMPos[4][4])
{
    int iTool = iToolNum + 1;
    int iWork = iWorkNum + 1;

	double dTFMatrix[4][4] = {0};
	double dFTMatrix[4][4] = {0};
	double dWBMatrix[4][4] = {0};
	double dBWMatrix[4][4] = {0};

	HS_CPosToMPos(m_dToolCoord[iTool],dTFMatrix);
	Matrix_Inverse(4,&dTFMatrix[0][0],&dFTMatrix[0][0]);

    double dWorldMatrix[4][4] = {0};
    HS_CPosToMPos(m_dWorldCoord,dWorldMatrix);
    double dWorkWorldMatrix[4][4] = {0};
    HS_CPosToMPos(m_dWorkCoord[iWork],dWorkWorldMatrix);  
    Matrix_Multi(4,4,4,&dWorldMatrix[0][0],&dWorkWorldMatrix[0][0],&dWBMatrix[0][0]);

	Matrix_Inverse(4,&dWBMatrix[0][0],&dBWMatrix[0][0]);

	//法兰盘到基础坐标系的变换矩阵  B/F*F/T = B/T			
	double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  W/B*B/T = W/T	
	//工具点到工件坐标系的变换矩阵  W/T			
	Matrix_Multi(4,4,4,&dFBMPos[0][0],&dTFMatrix[0][0],&dTBMatrix[0][0]);
	Matrix_Multi(4,4,4,&dBWMatrix[0][0],&dTBMatrix[0][0],&dTWMPos[0][0]);
	return 0;
}
/************************************************
函数功能：关节坐标计算变位机坐标矩阵
参    数：dJPos------关节坐标
		iToolNum----工具坐标
		 dTCMPos----工具在变位机坐标系的坐标值
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToTCMPos(double dJPos[MaxAxisNum],int iToolNum,double dTCMPos[5][4])
{
	int iErrorId = 0;

	double dTWMPos[5][4] = {0};
	HS_JPosToMPos(dJPos,m_iToolNum,-1,dTWMPos);

	double dCWMPos[4][4] = {0};
	HS_JPosToPoMPos(&dJPos[6],dCWMPos);

	double dWCMPos[4][4] = {0};
	Matrix_Inverse(4,&dCWMPos[0][0],&dWCMPos[0][0]);						//C/W

	Matrix_Multi(4,4,&dWCMPos[0][0],&dTWMPos[0][0],&dTCMPos[0][0]);			//C/W*W/T = C/T

	dTCMPos[4][0] = dJPos[6];
	dTCMPos[4][1] = dJPos[7];
	dTCMPos[4][2] = dJPos[8];

	return 0;
}
/************************************************
函数功能：地轨坐标系计算
参    数：dJPos------地轨对应的关节坐标【附加轴】
		 dFlBMPos---求解得到的地轨相对基坐标矩阵
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToFlMPos(double dJPos[3],double dFlBMPos[4][4])
{
	int iErrorId = 0;

	m_dFlTrackCoord->iAxisId[0] = 0;

	return iErrorId;
}

/************************************************
函数功能：变位机坐标系计算
参    数：dJPos------变位机对应的关节坐标【附加轴】
		 dPoBMPos---求解得到的变位机相对基坐标矩阵
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToPoMPos(double dJPos[MaxAxisNum],double dPoBMPos[4][4])
{
	int iErrorId = 0;

	double dMRotz1[4][4] = {0};
	double dMRotz2[4][4] = {0};
	double dMRotz3[4][4] = {0};

	Matrix_Eye(4,&dMRotz1[0][0]);
	Matrix_Eye(4,&dMRotz2[0][0]);
	Matrix_Eye(4,&dMRotz3[0][0]);

	int iAxis1 = m_dPositionerCoord->iAxisId[0];
	int iAxis2 = m_dPositionerCoord->iAxisId[1];
	int iAxis3 = m_dPositionerCoord->iAxisId[2];

	if(*m_eRobotType == HSROB_COORPER)
	{
		if(iAxis1 >= 0&&iAxis1 <= 8)
			HS_RotZMPos(dJPos[iAxis1],dMRotz1);	

		if(iAxis2 >= 0&&iAxis2 <= 8)
			HS_RotZMPos(dJPos[iAxis2],dMRotz2);	

		if(iAxis3 >= 0&&iAxis3 <= 8)
			HS_RotZMPos(dJPos[iAxis3],dMRotz3);
	}
	else
	{
		if(iAxis1 >= 6&&iAxis1 <= 8)
			HS_RotZMPos(dJPos[iAxis1-6],dMRotz1);	

		if(iAxis2 >= 6&&iAxis2 <= 8)
			HS_RotZMPos(dJPos[iAxis2-6],dMRotz2);	

		if(iAxis3 >= 6&&iAxis3 <= 8)
			HS_RotZMPos(dJPos[iAxis3-6],dMRotz3);
	}

	double dMBJ1[4][4] = {0};
	double dMJ1J2[4][4] = {0};
	double dMJ2J3[4][4] = {0};

	HS_CPosToMPos(m_dPositionerCoord->dExtCoord[0], dMBJ1);
	HS_CPosToMPos(m_dPositionerCoord->dExtCoord[1], dMJ1J2);
	HS_CPosToMPos(m_dPositionerCoord->dExtCoord[2], dMJ2J3);

	double dMT1[4][4],dMT2[4][4],dMT3[4][4];
	Matrix_Multi(4,4,4,&dMBJ1[0][0],&dMRotz1[0][0],&dMT1[0][0]);
	Matrix_Multi(4,4,4,&dMJ1J2[0][0],&dMRotz2[0][0],&dMT2[0][0]);
	Matrix_Multi(4,4,4,&dMJ2J3[0][0],&dMRotz3[0][0],&dMT3[0][0]);

	double M1M2[4][4];
	Matrix_Multi(4,4,4,&dMT1[0][0],&dMT2[0][0],&M1M2[0][0]);
	Matrix_Multi(4,4,4,&M1M2[0][0],&dMT3[0][0],&dPoBMPos[0][0]);

	return iErrorId;
}

/************************************************
函数功能：将工具工件的坐标值	的点位坐标转换为法兰盘基坐标系下
参    数：dTWMPos---工具工件坐标系【输入】	
		 eCPType---转换的坐标系类型
		 dFBMPos---法兰盘基坐标系【输出】
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_TWMPosToFBMPos(double dTWMPos[5][4],CPType eCPType,double dFBMPos[4][4])
{
	double dTBMatrix[4][4] = {0};
	HS_TWMPosToFBMPos(dTWMPos,eCPType,dFBMPos,dTBMatrix);
	return 0;
}

int HS_Kinematics::HS_TWMPosToFBMPos(double dTWMPos[5][4],CPType eCPType,double dFBMPos[4][4],double dTBMatrix[4][4])
{
	switch(eCPType)
	{
	case CP_ToolWork:	//工具点在工件坐标系中的位置
		{
			//double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T
			//double dFBMatrix[4][4];	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F			
			Matrix_Multi(4,4,4,&m_dWBMatrix[0][0],&dTWMPos[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&dTBMatrix[0][0],&m_dFTMatrix[0][0],&dFBMPos[0][0]);			
			break;
		}
	case CP_WorkTool:	//工件点在工具坐标系中的位置
		{
			double dWTMatrix[4][4];	//工件点在工具坐标系的变换矩阵  T/W
			double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T
			//double dFBMatrix[4][4];	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F
			memcpy(dWTMatrix,dTWMPos,sizeof(dWTMatrix));
			Matrix_Inverse(4,&dWTMatrix[0][0],&dTWMatrix[0][0]);
			Matrix_Multi(4,4,4,&m_dTFMatrix[0][0],&dTWMatrix[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&dTBMatrix[0][0],&m_dBWMatrix[0][0],&dFBMPos[0][0]);			
			break;
		}
	case CP_Flange:		//法兰盘在基础坐标系中位置
		break;	
	case CP_EterBaseSpin://外部TCP点动逆解（MPos是外部工件相对于外部工具的值）
		{
			Matrix_Multi(4,4,4,&m_dTFMatrix[0][0],&dTWMPos[0][0],&dTBMatrix[0][0]);
			Matrix_Multi(4,4,4,&dTBMatrix[0][0],&m_dBWMatrix[0][0],&dFBMPos[0][0]);
			break;
		}
	case CP_ExtCoorper:     //地轨坐标系处理
		{
			double dTEMatrix[4][4];	//工具点到基础坐标系的变换矩阵  B/W*W/T = B/T	
			Matrix_Multi(4,4,4,&m_dWBMatrix[0][0],&dTWMPos[0][0],&dTEMatrix[0][0]);

			//点位为 W/T的点位 其中W为相对E的变换关系
			double dBECoord[4][4] = {0};
			memcpy(dBECoord,m_dBECoord,sizeof(dBECoord));

			dBECoord[0][3] = dTWMPos[4][m_iExtNum-6];

			double dEBMatrix[4][4] = {0};
			Matrix_Inverse(4,&dBECoord[0][0],&dEBMatrix[0][0]);
			double dTBMatrix[4][4];	//工具点到基础坐标系的变换矩阵  B/W*W/T = B/T
			Matrix_Multi(4,4,4,&dEBMatrix[0][0],&dTEMatrix[0][0],&dTBMatrix[0][0]);  // B/E*E/T = B/T
			Matrix_Multi(4,4,4,&dTBMatrix[0][0],&m_dFTMatrix[0][0],&dFBMPos[0][0]);		
		}

		break;
	default:
		break;
	}
	return 0;
}
/************************************************
函数功能：关节角度求解齐次矩阵（法兰盘相对于基坐标）	
参    数：dJPos---关节角度
		 dMPos---齐次矩阵坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToMPos(double dJPos[MaxAxisNum],double dMPos[5][4])
{
	int iErrorId = 0;

    switch(*m_eRobotType)
    {
        case HSROB_PUMA:
            HS_JPosToMPos_Puma(dJPos,dMPos);
            break;
        case HSROB_SCARA:
            HS_JPosToMPos_Scara(dJPos,dMPos);
            break;
        default:
        break;
    }       	
	//附加轴坐标
	dMPos[4][0] = dJPos[6];
	dMPos[4][1] = dJPos[7];
	dMPos[4][2] = dJPos[8];
	return iErrorId;
}
/************************************************
函数功能：关节角度求解齐次矩阵---通用六轴	
参    数：dJPos---关节角度
		 dMPos---齐次矩阵坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToMPos_Puma(double dJPos[6],double dMPos[4][4])
{
    int iErrorId = 0;

    double A1[16],A2[16],A3[16],A4[16],A5[16],A6[16];
    HS_TransformPlus(A1,dJPos[0],m_dDHPara[0][0],m_dDHPara[0][1],m_dDHPara[0][2],m_dDHPara[0][3]);
    HS_TransformPlus(A2,dJPos[1],m_dDHPara[1][0],m_dDHPara[1][1],m_dDHPara[1][2],m_dDHPara[1][3]);
    HS_TransformPlus(A3,dJPos[2],m_dDHPara[2][0],m_dDHPara[2][1],m_dDHPara[2][2],m_dDHPara[2][3]);
    HS_TransformPlus(A4,dJPos[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]);
    HS_TransformPlus(A5,dJPos[4],m_dDHPara[4][0],m_dDHPara[4][1],m_dDHPara[4][2],m_dDHPara[4][3]);
    HS_TransformPlus(A6,dJPos[5],m_dDHPara[5][0],m_dDHPara[5][1],m_dDHPara[5][2],m_dDHPara[5][3]);

    double J2[16],J3[16],J4[16],J5[16];
    Matrix_Multi(4,4,4,A1,A2,J2);	
    Matrix_Multi(4,4,4,J2,A3,J3);
    Matrix_Multi(4,4,4,J3,A4,J4);
    Matrix_Multi(4,4,4,J4,A5,J5);
    Matrix_Multi(4,4,4,J5,A6,&dMPos[0][0]);
    return iErrorId;
}
/************************************************
函数功能：关节角度求解齐次矩阵---通用Scara	
参    数：dJPos---关节角度
		 dMPos---齐次矩阵坐标	 
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JPosToMPos_Scara(double dJPos[6],double dMPos[4][4])
{
    int iErrorId = 0;

	double A1[16],A2[16],A3[16],A4[16];

    //β角更改，添加独立的参数
    HS_TransformPlus(A1,dJPos[0],m_dDHPara[0][0],0,m_dDHPara[0][2],m_dDHPara[0][3]);
    HS_TransformPlus(A2,dJPos[1],m_dDHPara[1][0],0,m_dDHPara[1][2],m_dDHPara[1][3]);
    HS_TransformPlus(A3,       0,m_dDHPara[2][0], dJPos[2],m_dDHPara[2][2],m_dDHPara[2][3]);
	if(*m_eRobotType == HSROB_SCARA_3)
		HS_TransformPlus(A4,0,m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]);
	else
		HS_TransformPlus(A4,dJPos[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]); 

	double J2[16],J3[16];
	Matrix_Multi(4,4,4,A1,A2,J2);	
	Matrix_Multi(4,4,4,J2,A3,J3);
	Matrix_Multi(4,4,4,J3,A4,&dMPos[0][0]);	

    m_dA360BaseAngle = dJPos[0] + dJPos[1] + dJPos[3];
    return iErrorId;
}

/************************************************
函数功能：计算齐次变换矩阵【添加β角度】
参    数：dJPos---关节角度，角度
		 dA,dD,dC---DH参数
返 回 值：pdMatrix--齐次变换矩阵
*************************************************/
void HS_Kinematics::HS_TransformPlus(double *pdMatrix,double dJPos,double dA,double dD,double dC,double dB)
{
    double dcosAngle,dsinAngle,dcosC,dsinC,dsinB,dcosB;
    dcosAngle = cos(dJPos*deg2rad);
    dsinAngle = sin(dJPos*deg2rad);
    dcosC = cos(dC*deg2rad);
    dsinC = sin(dC*deg2rad);
    dcosB = cos(dB*deg2rad);
    dsinB = sin(dB*deg2rad);

    pdMatrix[0] = dcosAngle*dcosB - dsinAngle*dsinC*dsinB; 
    pdMatrix[1] = -dsinAngle*dcosC;
    pdMatrix[2] = dcosAngle*dsinB + dsinAngle*dsinC*dcosB;
    pdMatrix[3] = dA*dcosAngle;
    pdMatrix[4] = dsinAngle*dcosB + dcosAngle*dsinC*dsinB; 
    pdMatrix[5] = dcosAngle*dcosC;	
    pdMatrix[6] = dsinAngle*dsinB - dcosAngle*dsinC*dcosB; 
    pdMatrix[7]	= dA*dsinAngle;
    pdMatrix[8] = -dcosC*dsinB;		 
    pdMatrix[9] = dsinC;			
    pdMatrix[10] = dcosC*dcosB;		   
    pdMatrix[11] = dD;
    pdMatrix[12] = 0;
    pdMatrix[13] = 0;
    pdMatrix[14] = 0;
    pdMatrix[15] = 1;
}

/************************************************
函数功能：逆解计算【精确求解】
参    数：dMPos-------空间齐次矩阵
		  iToolNum---点位工具号
          iWorkNum---点位工件号
          eState-----点位形态
          eCPType----点位对应的坐标类型
          dJPos------求解得到的关节坐标
		  bWristQyFlag---奇异处理求解
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_MPosToJPos_JXJ(double dMPos[4][4],int iToolNum,int iWorkNum,unsigned char eState,CPType eCPType,double dJPos[6],bool bWristQyFlag)
{
	double dTBMatrix[4][4] = {0};	//工具点到基础坐标系的变换矩阵  B/W*W/T = B/T
	double dFBMatrix[4][4] = {0};	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F

	HS_TWMPosToFBMPos(dMPos,iToolNum,iWorkNum,dFBMatrix,dTBMatrix);

	//////////////////////
	double dWTMatrix[4][4];	//工件点在工具坐标系的变换矩阵  T/W
	double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T
	//double dFBMatrix[4][4];	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F
	//外部TCP模式
	// if(eCPType == CP_WorkTool)
	// {
	// 	memcpy(dWTMatrix,dMPos,sizeof(dWTMatrix));
	// 	Matrix_Inverse(4,&dWTMatrix[0][0],&dTWMatrix[0][0]);
	// 	Matrix_Multi(4,4,4,&dTFMatrix[0][0],&dTWMatrix[0][0],&dTBMatrix[0][0]);
	// 	Matrix_Multi(4,4,4,&dTBMatrix[0][0],&dBWMatrix[0][0],&dFBMatrix[0][0]);	
	// }
	//////////////////////

	int iError = 0;
    double dJPosCalc[6] = {0};
	memcpy(dJPosCalc,dJPos,sizeof(double)*6);

	switch(*m_eRobotType)
	{
	case HSROB_PUMA_5:
	case HSROB_MD410:
		break;
	case HSROB_SCARA:
        iError = HS_MPosToJPos_Scara(dFBMatrix,eState,dJPosCalc);

        //alph和beta角度的补偿，通过雅可比矩阵进行临近点位的迭代求解
        HS_MPosToJPos(dTBMatrix,dJPosCalc,dJPosCalc,true,iToolNum);
		break;
	case HSROB_SCARA_3:
		break;
    case HSROB_LINK_3:
        break;
    case HSROB_PUMA:
		if(bWristQyFlag)
		{
			iError = HS_FBMPosToJPos_Puma(dFBMatrix,eState,dJPosCalc,bWristQyFlag);
			
			//4轴不变
			dJPosCalc[3] = dJPos[3];
			const int iMAXCNT = 8;
			for(int i = 0;i < iMAXCNT;i++)
			{
				HS_MPosToJPos(dTBMatrix,dJPosCalc,dJPosCalc,true,iToolNum+1,bWristQyFlag);
			}
		}
		else
			iError = HS_FBMPosToJPos_Puma(dFBMatrix,eState,dJPosCalc);
        break;
	default:	
		break;
	}

    memcpy(dJPos,dJPosCalc,sizeof(double)*6);
	return iError;
}

/************************************************
函数功能：逆解计算关节点位【6轴】
参    数： dFBMPos-------空间齐次矩阵
		  eState------形态位
          dJPos------求解得到的关节坐标
		  bWristQyFlag---奇异处理【限制4轴】
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_FBMPosToJPos_Puma(double dFBMPos[4][4],unsigned char eState,double dJPos[6],bool bWristQyFlag)
{
	int iErrorId = 0;
	double dJPosCalc[6] = {0};
	const double MaxBR	= 1.0;		//区别BR系列和JR系列机型
	memcpy(dJPosCalc,dJPos,sizeof(double)*6);

	if(m_bCobot6Flag)
	{
		if(bWristQyFlag)
		{
			iErrorId = HS_MPosToJPos_PieperCobot(dFBMPos,eState,dJPosCalc,true);
		}
		else
		{
			iErrorId = HS_MPosToJPos_PieperCobot(dFBMPos,eState,dJPosCalc);
			if(iErrorId != 0)
			{
				LOG_ALGO("Cobot Pieper Error!");
				return iErrorId;
			}
			iErrorId = HS_MPosToJPos_Iter_Cobot(dFBMPos,eState,dJPosCalc,dJPosCalc);
		}
	}
	else if(m_bTypeBR)
	{			
		if(bWristQyFlag)
		{
			iErrorId = HS_MPosToJPos_Pieper(dFBMPos,eState,dJPosCalc,true);
		}
		else
		{
			iErrorId = HS_MPosToJPos_Pieper(dFBMPos,eState,dJPosCalc);
			if(iErrorId != 0)
			{
				LOG_ALGO("BR Pieper Error!");
				return iErrorId;
			}
			iErrorId = HS_MPosToJPos_Iter(dFBMPos,eState,dJPosCalc,dJPosCalc);	
		}
	}
	else
	{
		if(bWristQyFlag)
		{
			iErrorId = HS_MPosToJPos_Pieper(dFBMPos,eState,dJPosCalc,true);	
		}
		else
		{
			iErrorId = HS_MPosToJPos_Pieper(dFBMPos,eState,dJPosCalc);		
			HS_MPosToJPos(dFBMPos,dJPosCalc,dJPosCalc);
		}
	}
	memcpy(dJPos,dJPosCalc,sizeof(double)*6);
	return iErrorId;
}

/************************************************
函数功能：逆解计算【临近点求解】空间点位
参    数：dMPos-------空间齐次矩阵
		  iToolNum---点位工具号
          iWorkNum---点位工件号
          eCPType----点位对应的坐标类型
		  dLJPos-----临近关节位置
          dCJPos------求解得到的关节坐标
		  bWristQyFlag--腕部奇异处理
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_CPosToJPos_LJ(double dCPos[6],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9],bool bWristQyFlag)
{
    double dMPos[5][4] = {0};
    HS_CPosToMPos(dCPos,dMPos);
    int iErrorId = HS_MPosToJPos_LJ(dMPos,iToolNum,iWorkNum,eCPType,dLJPos,dCJPos,bWristQyFlag);
    return iErrorId;
}

/************************************************
函数功能：逆解计算【临近点求解】
参    数：dMPos-------空间齐次矩阵
		  iToolNum---点位工具号
          iWorkNum---点位工件号
          eCPType----点位对应的坐标类型
		  dLJPos-----临近关节位置
          dCJPos------求解得到的关节坐标
		  bWristQyFlag--腕部奇异处理
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_MPosToJPos_LJ(double dMPos[5][4],int iToolNum,int iWorkNum,CPType eCPType,double dLJPos[6],double dCJPos[9],bool bWristQyFlag)
{
    int iTool = iToolNum + 1;
    int iWork = iWorkNum + 1;

	double dTBMatrix[4][4] = {0};	//工具点到基础坐标系的变换矩阵  B/W*W/T = B/T
	double dFBMatrix[4][4] = {0};	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F
	double dFTMatrix[4][4] = {0},dTFMatrix[4][4] = {0};
	double dWBMatrix[4][4] = {0},dBWMatrix[4][4] = {0};
	
	HS_CPosToMPos(m_dToolCoord[iTool],dTFMatrix);
	Matrix_Inverse(4,&dTFMatrix[0][0],&dFTMatrix[0][0]);
	HS_CPosToMPos(m_dWorkCoord[iWork],dWBMatrix);
	Matrix_Inverse(4,&dWBMatrix[0][0],&dBWMatrix[0][0]);

    if(eCPType == CP_ExtCoorper)
    {
        //点位为 E/T的点位
        double dBECoord[4][4] = {0};
        memcpy(dBECoord,m_dBECoord,sizeof(dBECoord));

        dBECoord[0][3] = dMPos[4][m_iExtNum-6];

        double dEBMatrix[4][4] = {0};
        Matrix_Inverse(4,&dBECoord[0][0],&dEBMatrix[0][0]);
        double dWBMartix[4][4] = {0};
        Matrix_Multi(4,4,4,&dEBMatrix[0][0],&dWBMatrix[0][0],&dWBMartix[0][0]);     // B/E*E/W = B/W
        Matrix_Multi(4,4,4,&dWBMartix[0][0],&dMPos[0][0],&dTBMatrix[0][0]);         // B/W*W/T = B/T

        dCJPos[6] = dMPos[4][0];
        dCJPos[7] = dMPos[4][1];
        dCJPos[8] = dMPos[4][2];
    }
    else
    {
        Matrix_Multi(4,4,4,&dWBMatrix[0][0],&dMPos[0][0],&dTBMatrix[0][0]);
    }

	Matrix_Multi(4,4,4,&dTBMatrix[0][0],&dFTMatrix[0][0],&dFBMatrix[0][0]);

	//////////////////////
	double dWTMatrix[4][4];	//工件点在工具坐标系的变换矩阵  T/W
	double dTWMatrix[4][4];	//工具点到工件坐标系的变换矩阵  W/T
	//double dFBMatrix[4][4];	//法兰盘到基础坐标系的变换矩阵  B/T*T/F = B/F
	//外部TCP模式
	if(eCPType == CP_WorkTool)
	{
		memcpy(dWTMatrix,dMPos,sizeof(dWTMatrix));
		Matrix_Inverse(4,&dWTMatrix[0][0],&dTWMatrix[0][0]);
		Matrix_Multi(4,4,4,&dTFMatrix[0][0],&dTWMatrix[0][0],&dTBMatrix[0][0]);
		Matrix_Multi(4,4,4,&dTBMatrix[0][0],&dBWMatrix[0][0],&dFBMatrix[0][0]);	
	}
	//////////////////////
    int iErrorId = 0;
	double dLLJPos[6] = {0};
	memcpy(dLLJPos,dLJPos,sizeof(double)*6);
	switch(*m_eRobotType)
	{
		case HSROB_PUMA:
			//iErrorId = HS_MPosToJPos(dFBMatrix,dLJPos,dCJPos);
			{
				iErrorId = HS_MPosToJPos(dTBMatrix,dLLJPos,dCJPos,true,iTool,bWristQyFlag);
				if(iErrorId != 0)
					return iErrorId;

				int iLoopCnt = 1;
				if(bWristQyFlag)
					iLoopCnt = 3;
				
				for(int i = 0;i < iLoopCnt;i++)
				{
					memcpy(dLLJPos,dCJPos,sizeof(double)*6);
					iErrorId = HS_MPosToJPos(dTBMatrix,dLLJPos,dCJPos,true,iTool,bWristQyFlag);	
				}
			}
			break;
        case HSROB_SCARA:
			{
				iErrorId = HS_MPosToJPos(dTBMatrix,dLLJPos,dCJPos,true,iTool,bWristQyFlag);
				if(iErrorId != 0)
					return iErrorId;
				memcpy(dLLJPos,dCJPos,sizeof(double)*6);
				iErrorId = HS_MPosToJPos(dTBMatrix,dLLJPos,dCJPos,true,iTool,bWristQyFlag);	
			}
		    break;            
        
        default:
		break;
	}
	return iErrorId;
}
/************************************************
函数功能：空间位置逆解关节坐标
参    数：dCPos-----空间位置【输入】
		 eCPType----空间位置的类型
		 dLJPos-----上一个关节位置【输入】
		 dCJPos-----当前关节位置【输出】
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPosToJPos(double dCPos[6],CPType eCPType,double dLJPos[6],double dCJPos[6])
{
	int iErrorId = 0;
	double dTWMPos[5][4] = {0};	
	HS_CPosToMPos(dCPos,dTWMPos);
	iErrorId = HS_MPosToJPos(dTWMPos,eCPType,dLJPos,dCJPos);
	return iErrorId;
}
/************************************************
函数功能：齐次变换矩阵逆解求关节坐标
参    数：pMPos--齐次矩阵
		 dLJPos--上一个关节角
		 dCJPos---当前关节角度		 
返 回 值：错误Id 
*************************************************/
int HS_Kinematics::HS_MPosToJPos(double dMPos[5][4],CPType eCPType,double dLJPos[6],double dCJPos[6],bool bWristQyFlag)
{
	double dTBMPos[4][4] = {0};
	double dFBMPos[4][4] = {0};
	int iErrorId = 0;
	HS_TWMPosToFBMPos(dMPos,eCPType,dFBMPos,dTBMPos);

	double dLLJPos[6] = {0};
	memcpy(dLLJPos,dLJPos,sizeof(double)*6);

	switch(*m_eRobotType)
	{
		case HSROB_PUMA:
			{
				iErrorId = HS_MPosToJPos(dTBMPos,dLLJPos,dCJPos,true,m_iToolNum+1,bWristQyFlag);
				if(iErrorId != 0)
					return iErrorId;
				memcpy(dLLJPos,dCJPos,sizeof(double)*6);
				iErrorId = HS_MPosToJPos(dTBMPos,dLLJPos,dCJPos,true,m_iToolNum+1,bWristQyFlag);	
			}

			break;
        case HSROB_SCARA:
			{
				iErrorId = HS_MPosToJPos(dFBMPos,dLLJPos,dCJPos);
				if(iErrorId != 0)
					return iErrorId;
				memcpy(dLLJPos,dCJPos,sizeof(double)*6);
				iErrorId = HS_MPosToJPos(dFBMPos,dLLJPos,dCJPos);				
			}
		break;
		default:
		break;
	}   
	
	if(iErrorId != 0)
		return iErrorId;

	//逆解得到关节位置求解精度确认
	double dMPosOut[5][4] = {0};
	HS_JPosToMPos(dCJPos,dMPosOut);
    if(*m_eRobotType == HSROB_PUMA&&!m_bWristQYHandleFlag)
    {
	    double dError = HS_MatrixDis(dMPosOut,dFBMPos);
        double ERRORMAX = 5.0;      
	    if(dError > ERRORMAX)//&&m_bPrintFlag)
	    {
		    //m_bPrintFlag = false;
		    LOG_ALGO("TWMPos[X-Z]:%.6lf,%.6lf,%.6lf",
			    dMPos[0][3],dMPos[1][3],dMPos[2][3]);
		    LOG_ALGO("FBMPos[X-Z]:%.6lf,%.6lf,%.6lf",
			    dFBMPos[0][3],dFBMPos[1][3],dFBMPos[2][3]);
		    LOG_ALGO("MPosOut[X-Z]:%.6lf,%.6lf,%.6lf",
			    dMPosOut[0][3],dMPosOut[1][3],dMPosOut[2][3]);
		    LOG_ALGO("CJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
			    dCJPos[0],dCJPos[1],dCJPos[2],dCJPos[3],dCJPos[4],dCJPos[5]);
		    LOG_ALGO("LJPos:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
			    dLJPos[0],dLJPos[1],dLJPos[2],dLJPos[3],dLJPos[4],dLJPos[5]);		
            LOG_ALGO("Error = %.3lf",dError);
		    return ERROR_C2JACCURACY;
	    }
    }
	return iErrorId;
}
/************************************************
函数功能：求解两个齐次变换矩阵的差值
参    数：dMPosA----齐次变换矩阵A
		 dMPosB----齐次变化矩阵B
返 回 值： 差值
*************************************************/
double HS_Kinematics::HS_MatrixDis(double dMPosA[4][4],double dMPosB[4][4])
{
	double dLError = (dMPosA[0][3] - dMPosB[0][3])*(dMPosA[0][3] - dMPosB[0][3]) + 
		(dMPosA[1][3] - dMPosB[1][3])*(dMPosA[1][3] - dMPosB[1][3]) + 
		(dMPosA[2][3] - dMPosB[2][3])*(dMPosA[2][3] - dMPosB[2][3]);

	//姿态的差值求解，通过矩阵求解
	double dRTransp[4][4] = {0};		//转置矩阵
	double dRotate[4][4] = {0};			//旋转矩阵
	double dRA[4] = {0};				//旋转轴角

	HS_Math::Matrix_Transpose(4,4,&dMPosB[0][0],&dRTransp[0][0]);	
	HS_Math::Matrix_Multi(4,4,4,&dRTransp[0][0],&dMPosA[0][0],&dRotate[0][0]);

	double dRotateT[3][3] = {0};

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			dRotateT[i][j] = dRotate[i][j];

	HS_Math::Matrix_MToA(&dRotateT[0][0],dRA);

	double dError = sqrt(dLError) + dRA[0]*rad2deg;
	return dError;
}
/************************************************
函数功能：空间点位逆解关节位置求解
参    数：dCPos-----空间位置
		iToolNum---点位的工具号
		iWorkNum---点位的工件号
		eState-----点位的形态【机器人形态有效】
		dJPos------输出关节位置
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_CPosToJPos_JXJ(double *dCPos,int iToolNum,int iWorkNum,unsigned char eState,double dJPos[MaxAxisNum],bool bWristQyFlag)
{
	double dMPos[4][4] = {0};
	HS_CPosToMPos(dCPos,dMPos);
    m_dA360TWOffset = 0;
	int iErrorId = HS_MPosToJPos_JXJ(dMPos,iToolNum,iWorkNum,eState,CP_ToolWork,dJPos,bWristQyFlag);

    if(*m_eRobotType == HSROB_SCARA&&m_bScaraA360Flag)
    {
        double dJ4Ref = dCPos[3] - dJPos[0] - dJPos[1] - m_dA360TWOffset;
        HS_NearestPoint(dJPos[3],dJ4Ref,-1);
    }

	if(iErrorId == 0)
	{
		double dCalcCPos[9] = {0};
		HS_JPosToCPos(dJPos,iToolNum,iWorkNum,dCalcCPos);
		double dPErrorLimit = 0.1;
		for(int i = 0;i < 3;i++)
		{
			if(fabs(dCPos[i] - dCalcCPos[i]) > dPErrorLimit)
			{
				iErrorId = E_C2J_PRECISION;
				return iErrorId;
			}
		}

		double dAErrorLimit = 0.1;

		bool bQYHandleFlag = false;
		if(*m_eRobotType == HSROB_PUMA)//&&(m_bWristQYHandleFlag||fabs(dJPos[4]) < 5.0))
		{
			bQYHandleFlag = true;
		}

		// Scara机型暂不进行检测
		if(!bQYHandleFlag&&*m_eRobotType != HSROB_SCARA)
		{			
			for(int i = 3;i < 6;i++)
			{
				if(fabs(dCPos[i] - dCalcCPos[i]) > dAErrorLimit&&fabs(dCPos[i] - dCalcCPos[i]) < 360 - dAErrorLimit)
				{
					iErrorId = E_C2J_ATTITUDE;
					return iErrorId;
				}
			}
		}	
	}

	dJPos[6] = dCPos[6];
	dJPos[7] = dCPos[7];
	dJPos[8] = dCPos[8];	
	return iErrorId;
}
/************************************************
函数功能：空间点位逆解关节位置求解【考虑多转的临近点位求解】
参    数：dCPos-----空间位置
		iToolNum---点位的工具号
		iWorkNum---点位的工件号
		eState-----点位的形态【机器人形态有效】
        dSJPos-----起点位置
		dJPos------输出关节位置
		bWristQyFlag--腕部奇异处理【6轴】
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_CPosToJPos_JXJ(double *dCPos,int iToolNum,int iWorkNum,unsigned char eState,double dSJPos[MaxAxisNum],double dJPos[MaxAxisNum],bool bWristQyFlag)
{
	if(bWristQyFlag)
	{
		//传入起点坐标
		memcpy(dJPos,dSJPos,sizeof(double)*MaxAxisNum);
	}

    int iErrorId = HS_CPosToJPos_JXJ(dCPos,iToolNum,iWorkNum,eState,dJPos,bWristQyFlag);

    HS_JPosNearestHandle(dJPos,dSJPos);
    return iErrorId;
}
/************************************************
函数功能：空间位置A的工具工件点位切换
参    数：dCPosA-------输入空间位置
		 iToolNumA----当前工具坐标系
		 iWorkNumA----当前工作坐标系
		 iToolNumB----待转换工具坐标系
		 iWorkNumB----待转换工件坐标系
		 dCPosB----输出空间位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPosChangeCoord(double dCPosA[6],int iToolNumA,int iWorkNumA,int iToolNumB,int iWorkNumB,double dCPosB[6])
{
	//工具工件号与系统一致
	if(iToolNumA == iToolNumB&&iWorkNumA == iWorkNumB)
	{
		memcpy(dCPosB,dCPosA,sizeof(double)*6);
		return 0;
	}
	double dTWMatrixA[4][4] = {0};
	double dFBMatrixA[4][4] = {0};
	HS_CPosToMPos(dCPosA,dTWMatrixA);

	//TWPosA--->FBPosA
	HS_TWMPosToFBMPos(dTWMatrixA,iToolNumA,iWorkNumA,dFBMatrixA);

	double dTWMatrixB[4][4] = {0};
	//FBPosA--->TMPosB
	HS_FBMPosToTWMPos(dFBMatrixA,iToolNumB,iWorkNumB,dTWMatrixB);

	HS_MPosToCPos(dTWMatrixB,dCPosB);
	return 0;
}
/************************************************
函数功能：计算两个ZYX方式欧拉角的四元数差值
参   数：dEuler1----输入欧拉角1，SPos
		dEuler2----输入欧拉角2，EPos
		dDis-------输出四元数
返 回 值：无
*************************************************/
int HS_Kinematics::EulerZYX_CalcQ(double dCPosA[6],double dCPosB[6],double dQ[4])
{
	double dMStart[4][4] = {0};
	double dMEnd[4][4] = {0};

	double dMatrixStart[3][3] = {0};			
	double dMatrixEnd[3][3] = {0};	

	HS_CPosToMPos(dCPosA,dMStart);
	HS_CPosToMPos(dCPosB,dMEnd);

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			dMatrixStart[i][j] = dMStart[i][j];
			dMatrixEnd[i][j] = dMEnd[i][j];
		}

	double dMatrixStartR[3][3] = {0};

	Matrix_Inverse(3,&dMatrixStart[0][0],&dMatrixStart[0][0]);

	double dMatrixQ[3][3] = {0};
	Matrix_Multi(3,3,3,&dMatrixStart[0][0],&dMatrixEnd[0][0],&dMatrixQ[0][0]);

	Matrix_MToA(&dMatrixQ[0][0],dQ);
	return 0;
}
/************************************************
函数功能：计算两个ZYX方式欧拉角的角度差值
参   数：dEuler1----输入欧拉角1，SPos
		dEuler2----输入欧拉角2，EPos
		dDis-------输出欧拉角差值
返 回 值：无
*************************************************/
int HS_Kinematics::EulerZYX_CalcDis(double *dEuler1,double *dEuler2,double *dDis,bool bEulerQY)
{
	if(bEulerQY)
	{
		double dCosE2 = fabs(cos(dEuler2[1]*deg2rad));
		double dCosE1 = fabs(cos(dEuler1[1]*deg2rad));
		double dLimit = cos(89.99*deg2rad);
		//对欧拉角奇异进行处理
		if(dCosE2 < dLimit) //结束角度为欧拉奇异
		{
			//判断A-C值
			if(fabs(dEuler2[1] - 90) < 0.01)		//B角为90度，此时A-C为固定值
			{
				double dAPC = dEuler2[0] - dEuler2[2];

				//此时使得A角与起点的A角保持一致
				dEuler2[0] = dEuler1[0];
				dEuler2[2] = dEuler2[0] - dAPC;
				if(dEuler2[2] > 180)
					dEuler2[2] = dEuler2[2] - 360;
				else if(dEuler2[2] < -180)
					dEuler2[2] = dEuler2[2] + 360;
			}	
			//判断A+C值
			else if(fabs(dEuler2[1] + 90) < 0.01)	//B角为-90度，此时A+C为固定值
			{
				double dAPC = dEuler2[0] + dEuler2[2];

				//此时使得A角与起点的A角保持一致
				dEuler2[0] = dEuler1[0];
				dEuler2[2] = -dEuler2[0] + dAPC;
				if(dEuler2[2] > 180)
					dEuler2[2] = dEuler2[2] - 360;
				else if(dEuler2[2] < -180)
					dEuler2[2] = dEuler2[2] + 360;
			}
		}

		if(dCosE1 < dLimit) //起点角度为欧拉奇异
		{
			//判断A-C值
			if(fabs(dEuler1[1] - 90) < 0.01)		//B角为90度，此时A-C为固定值
			{
				double dAPC = dEuler1[0] - dEuler1[2];

				//此时使得A角与起点的A角保持一致
				dEuler1[0] = dEuler2[0];
				dEuler1[2] = dEuler1[0] - dAPC;
				if(dEuler1[2] > 180)
					dEuler1[2] = dEuler1[2] - 360;
				else if(dEuler1[2] < -180)
					dEuler1[2] = dEuler1[2] + 360;
			}	
			//判断A+C值
			else if(fabs(dEuler1[1] + 90) < 0.01)	//B角为-90度，此时A+C为固定值
			{
				double dAPC = dEuler1[0] + dEuler1[2];

				//此时使得A角与起点的A角保持一致
				dEuler1[0] = dEuler2[0];
				dEuler1[2] = -dEuler1[0] + dAPC;
				if(dEuler1[2] > 180)
					dEuler1[2] = dEuler1[2] - 360;
				else if(dEuler1[2] < -180)
					dEuler1[2] = dEuler1[2] + 360;
			}
		}

	}

	//通过欧拉角的两种表达方式求解出较临近的差值
	//求出当前ABC的总位移
	double dDis1 = Dis_ZYX(dEuler1,dEuler2);		
	//将末端点的ABC切换
	double dZYXNew[3] = {0};
	HS_ZYXChange(dEuler2,dZYXNew);
	double dDis2 = Dis_ZYX(dZYXNew,dEuler1);

	if(dDis2 < dDis1)
		memcpy(dEuler2,dZYXNew,sizeof(double)*3);

	//求解欧拉角差值
	for(int i = 0;i < 3;i++)
	{		
		dDis[i] = dEuler2[i] - dEuler1[i];

		///////////////////////////////////////
		//欧拉角360修改—（不做姿态差值变换）
		bool ScaraA360=false;
		// if(GetScaraA360())
		// {
		// 	if(m_eRobotType==HSROB_SCARA)
		// 		ScaraA360 = true;
		// }
		//////////////////////////////////////
		if(!ScaraA360)
		{
			if(dDis[i] > 180)
				dDis[i] = dDis[i] - 360;
			else if(dDis[i] < -180)
				dDis[i] = dDis[i] + 360;
			else if(dDis[i] > 360)
				dDis[i] = dDis[i] - 360;
			else if(dDis[i] < -360)
				dDis[i] = dDis[i] + 360;
			else if(dDis[i] > 720)
				dDis[i] = dDis[i] - 720;
			else if(dDis[i] < -720)
				dDis[i] = dDis[i] + 720;
		}
	}
	return 0;
}
/************************************************
函数功能：空间点位偏移计算【姿态左乘】【位移相加】
参    数：
         dCPosIn--------输入空间位置
         dCPosOffset----偏移空间位置
         dCPosOut-------输出空间位置
返 回 值：无
*************************************************/
void HS_Kinematics::HS_CPosOffset_RotLeft(double dCPosIn[6],double dCPosOffset[6],double dCPosOut[6])
{
    double dMPosIn[4][4] = {0};
    HS_CPosToMPos(dCPosIn,dMPosIn);

    double dMPosOffset[4][4] = {0};
    HS_CPosToMPos(dCPosOffset,dMPosOffset);

    double dMPosOut[4][4] = {0};
    Matrix_Multi(4,4,4,&dMPosOffset[0][0],&dMPosIn[0][0],&dMPosOut[0][0]);

    for(int i = 0;i < 3;i++)
        dMPosOut[i][3] = dCPosIn[i] + dCPosOffset[i];

    HS_MPosToCPos(dMPosOut,dCPosOut);
}
/************************************************
函数功能：欧拉角临近点位求解，防止点位突变
参    数：
         dCPosRef---参考空间位置
         dCPosIn----待求空间位置
返 回 值：错误码
*************************************************/
int HS_Kinematics::EnlerNearstHandle(double *dCPosRef,double *dCPosIn)
{
    double dEnlerRef[3] = {0};
    memcpy(dEnlerRef,&dCPosRef[3],sizeof(double)*3);

    double dEnlerIn[3] = {0};
    memcpy(dEnlerIn,&dCPosIn[3],sizeof(double)*3);

    double dDis1 = Dis_ZYX(dEnlerRef,dEnlerIn);		
    //将末端点的ABC切换
    double dZYXNew[3] = {0};
    HS_ZYXChange(dEnlerIn,dZYXNew);
    double dDis2 = Dis_ZYX(dEnlerRef,dZYXNew);

    if(dDis2 < dDis1)
        memcpy(dEnlerIn,dZYXNew,sizeof(double)*3);

    //临近点优化
    for(int i = 0;i < 3;i++)
    {
        double dDis = dEnlerIn[i] - dEnlerRef[i];
        if(dDis > 540)
            dEnlerIn[i] = dEnlerIn[i] - 720;
        else if(dDis > 180)
            dEnlerIn[i] = dEnlerIn[i] - 360;
        else if(dDis < -540)
            dEnlerIn[i] = dEnlerIn[i] + 720;
        else if(dDis < -180)
            dEnlerIn[i] = dEnlerIn[i] + 360;
    }
    memcpy(&dCPosIn[3],dEnlerIn,sizeof(double)*3);
    return 0;
}
/************************************************
函数功能：求解两个ZYX角度之间的差值
参   数：无
返 回 值：角度差值
*************************************************/
double HS_Kinematics::Dis_ZYX(double *dZYX1,double *dZYX2)
{
	double dDisAll = 0;
	double dDis = 0;
	for(int i = 0;i < 3;i++)
	{
		dDis = dZYX1[i] - dZYX2[i];

		///////////////////////////////////////
		//欧拉角360修改—（不做姿态差值变换）
		bool ScaraA360=false;
		// if(GetScaraA360())
		// {
		// 	if(m_eRobotType==HSROB_SCARA)
		// 		ScaraA360 = true;
		// }
		//////////////////////////////////////
		if(!ScaraA360)
		{
			if(dDis > 180)
				dDis = dDis - 360;
			else if(dDis < -180)
				dDis = dDis + 360;
		}
		dDisAll += fabs(dDis);
	}
	return dDisAll;
}
/************************************************
函数功能：ZYX角度值切换，由一种表示方式，切换到另外一种表示方式
参    数：pZYXIn---输入ZYX值		
		 pZYXOut--输出ZYX值
返 回 值：错误Id
*************************************************/
int HS_Kinematics::HS_ZYXChange(double *pZYXIn,double *pZYXOut)
{
	//对A进行处理
	if(pZYXIn[0] > 0)
		pZYXOut[0] = pZYXIn[0] - 180;
	else
		pZYXOut[0] = pZYXIn[0] + 180;

	//对C进行处理
	if(pZYXIn[2] > 0)
		pZYXOut[2] = pZYXIn[2] - 180;
	else
		pZYXOut[2] = pZYXIn[2] + 180;

	//对B进行处理
	if(pZYXIn[1] > 0)
		pZYXOut[1] = 180 - pZYXIn[1];
	else
		pZYXOut[1] = -180 - pZYXIn[1];

	return 0;
}
/************************************************
函数功能：空间位置逆解关节坐标--Scara机型		
参    数：pCPos-------空间位置
		 eCPType----空间位置的类型
		 eState------机械臂形态信息
		 pdCurJPos---当前关节角度	
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_MPosToJPos_Scara(double dFBMatrix[][4],unsigned char eState,double dJPos[6])
{
	double dSJPos[6] = {0};
    const double dBaseJPos = 6.0;          //临近点参考角度值

	//判断肩的姿态，伸直状态，小于奇异位置
	if((eState&0x01) == 0x01)
	{
		dSJPos[1] = dBaseJPos;
	}
	else
	{
		dSJPos[1] = -dBaseJPos;
	}
	//使用解析解求解
	int iErrorid = HS_MPosToJPos_Scara(dFBMatrix,dSJPos,dJPos);
	//unsigned char nState = HS_JPosToAState(dJPos);
	return iErrorid;
}
/************************************************
函数功能：齐次变换矩阵逆解求关节坐标--scara模型
参    数：dMPos--齐次矩阵
		 dLJPos---临近关节角
		 dCJPos---求解关节角度		 
返 回 值：错误Id 
*************************************************/
int HS_Kinematics::HS_MPosToJPos_Scara(double dMPos[4][4],double dLJPos[6],double dCJPos[6])
{		
	//求解Z轴位置
	dCJPos[2] = dMPos[2][3];	

	//由PX，PY求解0和1轴的角度值
	double dX = dMPos[0][3];
	double dY = dMPos[1][3];
	double dL1 = m_dDHPara[0][0];
	double dL2 = m_dDHPara[1][0];
	double dCos2 = (dX*dX + dY*dY - dL1*dL1 - dL2*dL2)/2/dL1/dL2;
	//需要在正常范围内
	if(dCos2 > 1.0||dCos2 < -1.0)
    {
        //区分由于靠近奇异，空间点位异常无法求逆，还是正常不可达
        if(fabs(dLJPos[1]) < 5.0 )
            return ERROR_SCARA_QYC2J;
        else
		    return ERROR_UNREACHABLE;
    }
	double dSin2_1 = sqrt(1 - dCos2*dCos2);
	double dSin2_2 = -sqrt(1 - dCos2*dCos2);

	double dAngle2_1 = atan2(dSin2_1,dCos2);
	double dAngle2_2 = atan2(dSin2_2,dCos2);

	double dSin2 = 0;
	//选取1轴的角度
	if(fabs(dAngle2_1 - dLJPos[1]*deg2rad) < fabs(dAngle2_2 - dLJPos[1]*deg2rad))
	{
		dCJPos[1] = dAngle2_1;
		dSin2 = dSin2_1;
	}
	else
	{	
		dCJPos[1] = dAngle2_2;
		dSin2 = dSin2_2;
	}

	double dK1 = dL1 + dL2*dCos2;
	double dK2 = dL2*dSin2;

	//求解0轴的角度
	dCJPos[0] = atan2(dY,dX) - atan2(dK2,dK1);

	//求解0+1+3的角度值
	double dAngle_All = atan2(dMPos[1][0],dMPos[0][0]);
	
	//从多个角度中求解3轴的角度值
	double dJPos[3] = {0};

	dJPos[0] = dAngle_All - dCJPos[0] - dCJPos[1];
	dJPos[1] = dJPos[0] + 2*PI;
	dJPos[2] = dJPos[0] - 2*PI;
	
	//欧拉角360修改：
	double LJ3 = dLJPos[3];
	//////////////////////////////
	

	//求解3轴角度最近点
	if(fabs(dJPos[0] - dLJPos[3]*deg2rad) < fabs(dJPos[1] - dLJPos[3]*deg2rad))
	{
		if(fabs(dJPos[0] - dLJPos[3]*deg2rad) < fabs(dJPos[2] - dLJPos[3]*deg2rad))
			dCJPos[3] = dJPos[0];
		else
			dCJPos[3] = dJPos[2];
	}
	else
	{
		if(fabs(dJPos[1] - dLJPos[3]*deg2rad) < fabs(dJPos[2] - dLJPos[3]*deg2rad))
			dCJPos[3] = dJPos[1];
		else
			dCJPos[3] = dJPos[2];
	}

	//3轴角度不发生突变，范围为+-360

	//转化为角度制
	dCJPos[0] = dCJPos[0]*rad2deg;
	dCJPos[1] = dCJPos[1]*rad2deg;
	dCJPos[3] = dCJPos[3]*rad2deg;

	//if(GetScaraA360())
	//{
	//	//欧拉角360修改：由于计算A角方式变化，会导致最终计算的4轴角度可能与上一点相差较大
	//	HS_NearestPoint(dCJPos[3],LJ3,-1);
	//	//////////////////////////////////////
	//}
	return 0;
}
/************************************************
函数功能：对于满足Pieper准则的逆解求解
参    数：
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_MPosToJPos_Pieper(double dFBMatrix[][4],unsigned char eState,double *dJPos,bool bWristQyFlag)
{
	double dRefJPos[6] = {0};
	memcpy(dRefJPos,dJPos,sizeof(double)*6);

    const double dLimitAdd = 10;
	//4轴末端点对应的位置
	double dGBTrans[4][4] = {0};
	dGBTrans[0][0] = 1;
	dGBTrans[1][1] = 1;
	dGBTrans[2][2] = 1;
	dGBTrans[3][3] = 1;
	dGBTrans[2][3] = -m_dDHPara[5][1];			//Z轴偏移
	double dGBMatrix[4][4] = {0};
	Matrix_Multi(4,4,4,&dFBMatrix[0][0],&dGBTrans[0][0],&dGBMatrix[0][0]);
	
	//提取位置值
	double dPx = dGBMatrix[0][3];
	double dPy = dGBMatrix[1][3];
	double dPz = dGBMatrix[2][3];
	//相应的DH参数的提取
	double dA0 = m_dDHPara[0][0];
	double dD0 = m_dDHPara[0][1];
	double dD1 = m_dDHPara[1][1];
	double dA1 = m_dDHPara[1][0];
	double dA2 = m_dDHPara[2][0];
	double dD3 = m_dDHPara[3][1];

	//先求前三个关节值
	//1、求解关节1的角度值
	double dLPXY = sqrt(dPx*dPx + dPy*dPy);
	double dAngleXY = atan2(dPy,dPx);
	if(fabs(dLPXY) < Eps)
		return ERROR_UNREACHABLE;
	double dSinAgXYAndA1 = dD1/dLPXY;
	if(fabs(dSinAgXYAndA1) > 1.0)
		return ERROR_UNREACHABLE;
	double dCosAgXYAndA1 = sqrt(1 - dSinAgXYAndA1*dSinAgXYAndA1);
	double dAgXYAndA1_1 = atan2(dSinAgXYAndA1,dCosAgXYAndA1);
	double dAgXYAndA1_2 = atan2(dSinAgXYAndA1,-dCosAgXYAndA1);
	double dAgA1[2] = {0};
	dAgA1[0] = (dAngleXY - dAgXYAndA1_1)*rad2deg;
	dAgA1[1] = (dAngleXY - dAgXYAndA1_2)*rad2deg;

	double dAgA1V,dTemp,dK3;
    double dK1,dK2,dTempL;
    double dTempAg,dSinTemp,dCosTemp;
    double dTAgA2_1,dTAgA2_2;
	double dAgA2[4] = {0};

    double ERRORLIMIT = 0.5;       //由于DH参数有偏差，所以误差需要放大【DH参数的误差应该不会影响求解的误差，后续的求解会处理误差，此处方法误差可能会导致求解的值不正确，故修改】

	int iErrorCnt = 0;
	for(int i = 0;i < 2;i++)
	{		
		//2、求解关节2的角度值
		//转化为K1*C2 - K2*S2 = K3的方式		
		dAgA1V = dAgA1[i]*deg2rad;
		dTemp = dPx*cos(dAgA1V) + dPy*sin(dAgA1V) - dA0;		
		if(fabs(dA1) < Eps)
		{
			return ERROR_UNREACHABLE;
		}
		dK3 = (dTemp*dTemp + dA1*dA1 + (dD0 - dPz)*(dD0 - dPz) - dD3*dD3 - dA2*dA2)/2/dA1;	
		dK1 = dTemp;
		dK2 = dPz - dD0;
		
		dTempL = sqrt(dK1*dK1 + dK2*dK2);
		dTempAg = atan2(dK1,dK2);		
		if(fabs(dTempL) < Eps)
		{			
			iErrorCnt++;
			continue;
		}
		dSinTemp = dK3/dTempL;		
		if(fabs(dSinTemp) > 1.0)
		{			
			iErrorCnt++;
			continue;
		}		
		dCosTemp = sqrt(1 - dSinTemp*dSinTemp);
		dTAgA2_1 = atan2(dSinTemp,dCosTemp);
		dTAgA2_2 = atan2(dSinTemp,-dCosTemp);
		dAgA2[2*i] = (dTempAg - dTAgA2_1)*rad2deg;
		dAgA2[2*i+1] = (dTempAg - dTAgA2_2)*rad2deg;
		
	}
	if(iErrorCnt == 2)
	{
        //if(m_bPrintFlag)
		LOG_ALGO("UnReachable XYZ!");
		return ERROR_UNREACHABLE;
	}
	
	double dAgA2V = 0;
	double dTAgA23_1,dTAgA23_2,dAgA23_1,dAgA23_2,dAgA3;
	double dJPosCalc[6] = {0};

	bool bCalOK = false;	//是否计算成功
    //矩阵求解
    //double A1[16],A2[16],A3[16],A4[16];
    //double J2[16],J3[4][4],J4[4][4];
    double dInvT03[3][3] = {0};
	for(int i = 0;i < 4;i++)
	{		
		dAgA2V = dAgA2[i]*deg2rad;

		//3、求解关节3的角度值
		//先求解2+3的角度值 同样转化为K1*C2 - K2*S2 = K3的方式
		dK3 = dA1*sin(dAgA2V) + dPz - dD0;
		dK1 = dD3;
		dK2 = dA2;
		dTempL = sqrt(dK1*dK1 + dK2*dK2);
		dTempAg = atan2(dK1,dK2);
		if(fabs(dTempL) < Eps)
			continue;
		dSinTemp = dK3/dTempL;
		if(fabs(dSinTemp) > 1.0)
			continue;
		dCosTemp = sqrt(1 - dSinTemp*dSinTemp);

		dTAgA23_1 = atan2(dSinTemp,dCosTemp);
		dTAgA23_2 = atan2(dSinTemp,-dCosTemp);
		dAgA23_1 = dTempAg - dTAgA23_1;
		dAgA23_2 = dTempAg - dTAgA23_2;

        //进一步选解，只有一组有效，通过PY值进行判断
        double dSinA1 = sin(dAgA1[i/2]*deg2rad);
        double dCosA1 = cos(dAgA1[i/2]*deg2rad);
        double dCosA2 = cos(dAgA2V);
        double dSin23 = sin(dAgA23_1);
        double dCos23 = cos(dAgA23_1);

        double dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
        if(fabs(dPyTemp) < ERRORLIMIT)
        {
            dAgA3 = (dAgA23_1 - dAgA2V)*rad2deg;
            double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
            if(fabs(dPxTemp) > ERRORLIMIT)
            {
                dSin23 = sin(dAgA23_2);
                dCos23 = cos(dAgA23_2);
                dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
                if(fabs(dPyTemp) < ERRORLIMIT)
                {
                    dAgA3 = (dAgA23_2 - dAgA2V)*rad2deg;
                    double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
                    if(fabs(dPxTemp) > ERRORLIMIT)
                        continue;
                }
                else 
                    continue;               
            }
         }
        else
        {
            dSin23 = sin(dAgA23_2);
            dCos23 = cos(dAgA23_2);
            dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
            if(fabs(dPyTemp) < ERRORLIMIT)
            {
                dAgA3 = (dAgA23_2 - dAgA2V)*rad2deg;
                double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
                if(fabs(dPxTemp) > ERRORLIMIT)
                    continue;
            }
            else 
                continue;
        }
        
		{			
			dJPosCalc[0] = dAgA1[i/2]; dJPosCalc[1] = dAgA2[i]; dJPosCalc[2] = dAgA3;	

            //1/2/3轴调整到合适的范围内，1轴-180~180,2轴-270到90,3轴0到360度【改为80至440，兼容BR系列】
            if(dJPosCalc[0] > 180)  dJPosCalc[0] -=360;
            else if(dJPosCalc[0] < -180)  dJPosCalc[0] +=360;

            if(dJPosCalc[1] > 90)  dJPosCalc[1] -=360;
            else if(dJPosCalc[1] < -270)  dJPosCalc[1] +=360;

            if(dJPosCalc[2] > 440)  dJPosCalc[2] -=360;
            else if(dJPosCalc[2] < 80)  dJPosCalc[2] +=360;

            /*unsigned char ucState = HS_JPosToAState(dJPosCalc);

            HS_Transform(A1,dJPosCalc[0],m_dDHPara[0][0],m_dDHPara[0][1],m_dDHPara[0][2]);
            HS_Transform(A2,dJPosCalc[1],m_dDHPara[1][0],m_dDHPara[1][1],m_dDHPara[1][2]);
            HS_Transform(A3,dJPosCalc[2],m_dDHPara[2][0],m_dDHPara[2][1],m_dDHPara[2][2]);
            HS_Transform(A4,dJPosCalc[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2]);

            Matrix_Multi(4,4,4,A1,A2,J2);	
            Matrix_Multi(4,4,4,J2,A3,&J3[0][0]);
            Matrix_Multi(4,4,4,&J3[0][0],A4,&J4[0][0]);
            double dError = fabs(dPx - J4[0][3]) + fabs(dPy - J4[1][3]) + fabs(dPz - J4[2][3]);
            //double dError = 0;
           */
            
            //【220905】优化逆解算法，优化选解，默认计算误差满足要求，形态的计算采用简便求解方法优化
            double dWristPx = dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23;
            unsigned char ucState = 0;
            if(dWristPx > 0)	//正手
                ucState |= AT_Front;
            else				//反手
                ucState |= AT_Back;

            //2、边界奇异位置判断肘的形态
            if(dJPosCalc[2] >= 90 + m_QYKbPara &&dJPosCalc[2] <= (90 + 180 + m_QYKbPara))
                ucState |= AT_Above;
            else
                ucState |= AT_Below;

            const double ErrorMax = 2.0;
			if((ucState&0x06) == (eState&0x06))//&& dError < ErrorMax)
			{
                //T03矩阵，直接求解，优化耗时，其逆矩阵为转置矩阵
                dInvT03[0][0] = dCosA1*dCos23;
                dInvT03[0][1] = dSinA1*dCos23;
                dInvT03[0][2] = -dSin23;
                dInvT03[1][0] = -dSinA1;
                dInvT03[1][1] = dCosA1;
                dInvT03[1][2] = 0;
                dInvT03[2][0] = dCosA1*dSin23;
                dInvT03[2][1] = dSinA1*dSin23;
                dInvT03[2][2] = dCos23;
				bCalOK = true;
				break;
			}			
		}	
		if(bCalOK)
			break;
	}
	//再求后三个值，解析解求解
	if(bCalOK)
	{
		//J3代表T03，取得姿态矩阵，然后求逆
		//double dT03[3][3] = {0};
		double dT06[3][3] = {0};

		for(int i = 0;i < 3;i++)
			for(int j = 0;j < 3;j++)
			{
				//dT03[i][j] = J3[i][j];
				dT06[i][j] = dFBMatrix[i][j];
			}
			//Matrix_Inverse(3,&dT03[0][0],&dInvT03[0][0]);
			//求解T36
			double dT36[3][3] = {0};
			Matrix_Multi(3,3,3,&dInvT03[0][0],&dT06[0][0],&dT36[0][0]);
			//求解第5轴角度值
			double dCosAg5 = dT36[2][2];
			double dSinAg5 = sqrt(1 - dCosAg5*dCosAg5);

			if(!bWristQyFlag)
			{
				//根据5轴的形态进行判断
				if((eState&0x08) == 0x08)	//5轴为负
				{
					dJPosCalc[4] = atan2(-dSinAg5,dCosAg5)*rad2deg;
					dSinAg5 = -dSinAg5;
				}
				else
				{
					dJPosCalc[4] = atan2(dSinAg5,dCosAg5)*rad2deg;
				}

				if(fabs(dJPosCalc[4]) > Eps)	//5轴为0，奇异位置无法求解
				{
					dJPosCalc[3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
					dJPosCalc[5] = atan2(dT36[2][1]/dSinAg5,-dT36[2][0]/dSinAg5)*rad2deg;	
					//不考虑限位处理
					memcpy(dJPos,dJPosCalc,sizeof(double)*6);
					return 0;
				}
				else
					return ERROR_UNREACHABLE;
			}
			else
			{
				//两组解，按照4/6轴的偏移小获取
				double dJPosOut[2][6] = {0};

				dJPosOut[0][4] = atan2(dSinAg5,dCosAg5)*rad2deg;
				dJPosOut[0][3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
				dJPosOut[0][5] = atan2(dT36[2][1]/dSinAg5,-dT36[2][0]/dSinAg5)*rad2deg;


				dJPosOut[1][4] = atan2(-dSinAg5,dCosAg5)*rad2deg;
				dSinAg5 = -dSinAg5;
				dJPosOut[1][3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
				dJPosOut[1][5] = atan2(dT36[2][1]/dSinAg5,-dT36[2][0]/dSinAg5)*rad2deg;

				double dDisA = fabs(dJPosOut[0][3] - dRefJPos[3]) + fabs(dJPosOut[0][5] - dRefJPos[5]);
				double dDisB = fabs(dJPosOut[1][3] - dRefJPos[3]) + fabs(dJPosOut[1][5] - dRefJPos[5]);

				if(dDisA < dDisB)
				{
					dJPosCalc[3] = dJPosOut[0][3];
					dJPosCalc[4] = dJPosOut[0][4];
					dJPosCalc[5] = dJPosOut[0][5];
				}
				else
				{
					dJPosCalc[3] = dJPosOut[1][3];
					dJPosCalc[4] = dJPosOut[1][4];
					dJPosCalc[5] = dJPosOut[1][5];
				}
				memcpy(dJPos,dJPosCalc,sizeof(double)*6);
				return 0;
			}			
	}
	return ERROR_UNREACHABLE;
}

/************************************************
函数功能：对于满足Pieper准则的逆解求解【协作系列机器人】
参    数：
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_MPosToJPos_PieperCobot(double dFBMatrix[][4],unsigned char eState,double *dJPos,bool bWristQyFlag)
{
	double dRefJPos[6] = {0};
	memcpy(dRefJPos,dJPos,sizeof(double)*6);
	
	double dBaseJPos[6] = {0,-90,180,0,90,0};
	for(int i = 0;i < 6;i++)				
	{
		dBaseJPos[i] = (m_tLimitPara->dPmax[i] + m_tLimitPara->dPmin[i])/2;		
	}

    //构建新的DH参数，满足Pieper准则的机器人，求解临近点位
    double dNewDH[6][3] = {0};
    //dNewDH[0][0] = 0;               dNewDH[0][1] = m_dDHPara[0][1];				dNewDH[0][2] = -90;
    //dNewDH[1][0] = m_dDHPara[1][0]; dNewDH[1][1] = m_dDHPara[2][1];				dNewDH[1][2] = 0;
    //dNewDH[2][0] = 0;               dNewDH[2][1] = 0;							dNewDH[2][2] = 90;
    //dNewDH[3][0] = 0;	            dNewDH[3][1] = (m_dDHPara[2][0]-m_dDHPara[4][1]);	    dNewDH[3][2] = -90;
    //dNewDH[4][0] = 0;	            dNewDH[4][1] = 0;							dNewDH[4][2] = 90;
    //dNewDH[5][0] = 0;	            dNewDH[5][1] = m_dDHPara[5][1];				dNewDH[5][2] = 0;

	dNewDH[0][0] = 0;               dNewDH[0][1] = m_dDHPara[0][1];				dNewDH[0][2] = -90;
	dNewDH[1][0] = m_dDHPara[1][0]; dNewDH[1][1] = m_dDHPara[2][1];				dNewDH[1][2] = 0;
	dNewDH[2][0] = 0;               dNewDH[2][1] = 0;							dNewDH[2][2] = 90;
	dNewDH[3][0] = 0;	            dNewDH[3][1] = m_dDHPara[2][0];				dNewDH[3][2] = -90;
	dNewDH[4][0] = 0;	            dNewDH[4][1] = 0;							dNewDH[4][2] = 90;
	dNewDH[5][0] = 0;	            dNewDH[5][1] = m_dDHPara[5][1];				dNewDH[5][2] = 0;

	bool bChange52 = false;
	if(fabs(m_dDHPara[5][2]) > 20)		//180度
		bChange52 = true;

    const double dLimitAdd = 10;
	//4轴末端点对应的位置
	double dGBTrans[4][4] = {0};
	dGBTrans[0][0] = 1;
	dGBTrans[1][1] = 1;
	dGBTrans[2][2] = 1;
	dGBTrans[3][3] = 1;
	if(!bChange52)
		dGBTrans[2][3] = -dNewDH[5][1];
	else
		dGBTrans[2][3] = dNewDH[5][1];			//Z轴偏移修改，DH参数变化了180
	double dGBMatrix[4][4] = {0};
	Matrix_Multi(4,4,4,&dFBMatrix[0][0],&dGBTrans[0][0],&dGBMatrix[0][0]);
	
	//提取位置值
	double dPx = dGBMatrix[0][3];
	double dPy = dGBMatrix[1][3];
	double dPz = dGBMatrix[2][3];
	//相应的DH参数的提取
	double dA0 = dNewDH[0][0];
	double dD0 = dNewDH[0][1];
	double dD1 = dNewDH[1][1];
	double dA1 = dNewDH[1][0];
	double dA2 = dNewDH[2][0];
	double dD3 = dNewDH[3][1];

	double dAgA1[2] = {0};
	double dAgA1V,dTemp,dK3;
	double dK1,dK2,dTempL;
	double dTempAg,dSinTemp,dCosTemp;
	double dTAgA2_1,dTAgA2_2;
	double dAgA2[4] = {0};

	double ERRORLIMIT = 0.5;       //由于DH参数有偏差，所以误差需要放大【DH参数的误差应该不会影响求解的误差，后续的求解会处理误差，此处方法误差可能会导致求解的值不正确，故修改】

	for(int iUnreached = 0;iUnreached < 2;iUnreached++)
	{
		//BR以及Co系列，非精准求解，允许有一定的超出
		if(iUnreached == 1)
		{
			double dLOff = fabs(m_dDHPara[4][1]);
			if(fabs(dPx) > fabs(dPy)&&fabs(dPx) > fabs(dPz))
			{
				if(dPx > 0)
					dPx -= dLOff;
				else 
					dPx += dLOff;
			}
			else if(fabs(dPy) > fabs(dPx)&&fabs(dPy) > fabs(dPz))
			{
				if(dPy > 0)
					dPy -= dLOff;
				else 
					dPy += dLOff;
			}
			else
			{
				if(dPz > 0)
					dPz -= dLOff;
				else 
					dPz += dLOff;
			}
		}

		//先求前三个关节值
		//1、求解关节1的角度值
		//m_HS_Printer->outDebugInfo("Motion_P","Kinematic","Pieper",0,AllDeb,"1!");
		double dLPXY = sqrt(dPx*dPx + dPy*dPy);
		double dAngleXY = atan2(dPy,dPx);
		if(fabs(dLPXY) < Eps)
			return ERROR_UNREACHABLE;
		double dSinAgXYAndA1 = dD1/dLPXY;
		if(fabs(dSinAgXYAndA1) > 1.0)
			return ERROR_UNREACHABLE;
		double dCosAgXYAndA1 = sqrt(1 - dSinAgXYAndA1*dSinAgXYAndA1);
		double dAgXYAndA1_1 = atan2(dSinAgXYAndA1,dCosAgXYAndA1);
		double dAgXYAndA1_2 = atan2(dSinAgXYAndA1,-dCosAgXYAndA1);

		dAgA1[0] = (dAngleXY - dAgXYAndA1_1)*rad2deg;
		dAgA1[1] = (dAngleXY - dAgXYAndA1_2)*rad2deg;


		int iErrorCnt = 0;
		for(int i = 0;i < 2;i++)
		{		
		   // m_HS_Printer->outDebugInfo("Motion_P","Kinematic","Pieper",0,AllDeb,"3!");
			//2、求解关节2的角度值
			//转化为K1*C2 - K2*S2 = K3的方式		
			dAgA1V = dAgA1[i]*deg2rad;
			dTemp = dPx*cos(dAgA1V) + dPy*sin(dAgA1V) - dA0;		
			if(fabs(dA1) < Eps)
			{
				LOG_ALGO("dA1Error:%lf",dA1);
				return ERROR_UNREACHABLE;
			}
			dK3 = (dTemp*dTemp + dA1*dA1 + (dD0 - dPz)*(dD0 - dPz) - dD3*dD3 - dA2*dA2)/2/dA1;	
			dK1 = dTemp;
			dK2 = dPz - dD0;
		
			dTempL = sqrt(dK1*dK1 + dK2*dK2);
			dTempAg = atan2(dK1,dK2);		
			if(fabs(dTempL) < Eps)
			{			
				iErrorCnt++;
				continue;
			}
			dSinTemp = dK3/dTempL;		
			if(fabs(dSinTemp) > 1.0)
			{			
				iErrorCnt++;
				continue;				
			}		
			dCosTemp = sqrt(1 - dSinTemp*dSinTemp);
			dTAgA2_1 = atan2(dSinTemp,dCosTemp);
			dTAgA2_2 = atan2(dSinTemp,-dCosTemp);
			dAgA2[2*i] = (dTempAg - dTAgA2_1)*rad2deg;
			dAgA2[2*i+1] = (dTempAg - dTAgA2_2)*rad2deg;
		
		}
		if(iErrorCnt == 2&&iUnreached == 1)
		{
			LOG_ALGO("UnReachable XYZ!");
			return -1;
		}
	}
	
	double dAgA2V = 0;
	double dTAgA23_1,dTAgA23_2,dAgA23_1,dAgA23_2,dAgA3;
	double dJPosCalc[6] = {0};

	bool bCalOK = false;	//是否计算成功
    //矩阵求解
    //double A1[16],A2[16],A3[16],A4[16];
    //double J2[16],J3[4][4],J4[4][4];
    double dInvT03[3][3] = {0};
	for(int i = 0;i < 4;i++)
	{		
		dAgA2V = dAgA2[i]*deg2rad;

		//3、求解关节3的角度值
		//先求解2+3的角度值 同样转化为K1*C2 - K2*S2 = K3的方式
		dK3 = dA1*sin(dAgA2V) + dPz - dD0;
		dK1 = dD3;
		dK2 = dA2;
		dTempL = sqrt(dK1*dK1 + dK2*dK2);
		dTempAg = atan2(dK1,dK2);
		if(fabs(dTempL) < Eps)
			continue;
		dSinTemp = dK3/dTempL;
		if(fabs(dSinTemp) > 1.0)
			continue;
		dCosTemp = sqrt(1 - dSinTemp*dSinTemp);

		dTAgA23_1 = atan2(dSinTemp,dCosTemp);
		dTAgA23_2 = atan2(dSinTemp,-dCosTemp);
		dAgA23_1 = dTempAg - dTAgA23_1;
		dAgA23_2 = dTempAg - dTAgA23_2;

        //进一步选解，只有一组有效，通过PY值进行判断
        double dSinA1 = sin(dAgA1[i/2]*deg2rad);
        double dCosA1 = cos(dAgA1[i/2]*deg2rad);
        double dCosA2 = cos(dAgA2V);
        double dSin23 = sin(dAgA23_1);
        double dCos23 = cos(dAgA23_1);

        double dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
        if(fabs(dPyTemp) < ERRORLIMIT)
        {
            dAgA3 = (dAgA23_1 - dAgA2V)*rad2deg;
            double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
            if(fabs(dPxTemp) > ERRORLIMIT)
            {
                dSin23 = sin(dAgA23_2);
                dCos23 = cos(dAgA23_2);
                dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
                if(fabs(dPyTemp) < ERRORLIMIT)
                {
                    dAgA3 = (dAgA23_2 - dAgA2V)*rad2deg;
                    double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
                    if(fabs(dPxTemp) > ERRORLIMIT)
                        continue;
                }
                else 
                    continue;               
            }
         }
        else
        {
            dSin23 = sin(dAgA23_2);
            dCos23 = cos(dAgA23_2);
            dPyTemp = dD1*dCosA1 + (dA0 + dA1*dCosA2)*dSinA1 + dSinA1*(dD3*dSin23 + dA2*dCos23) - dPy;
            if(fabs(dPyTemp) < ERRORLIMIT)
            {
                dAgA3 = (dAgA23_2 - dAgA2V)*rad2deg;
                double dPxTemp = -dD1*dSinA1 + dCosA1*(dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23) - dPx;
                if(fabs(dPxTemp) > ERRORLIMIT)
                    continue;
            }
            else 
                continue;
        }
        
		{			
			dJPosCalc[0] = dAgA1[i/2]; dJPosCalc[1] = dAgA2[i]; dJPosCalc[2] = dAgA3;	

			double dBaseNJPos[3] = {0};

			memcpy(dBaseNJPos,dBaseJPos,sizeof(dBaseNJPos));
			dBaseNJPos[2] += 90;

			for(int i = 0;i < 3;i++)				
			{				
				HS_NearestPoint(dJPosCalc[i],dBaseNJPos[i],-1);
			}
            
            //【220905】优化逆解算法，优化选解，默认计算误差满足要求，形态的计算采用简便求解方法优化
            double dWristPx = dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23;
            unsigned char ucState = 0;
			if(dWristPx > 0)	//正手
				ucState |= AT_Front;
			else				//反手
				ucState |= AT_Back;

            //2、边界奇异位置判断肘的形态
            if(dJPosCalc[2] >= 90 + m_QYKbPara &&dJPosCalc[2] <= (90 + 180 + m_QYKbPara))
                ucState |= AT_Above;
            else
                ucState |= AT_Below;

            const double ErrorMax = 2.0;
			if((ucState&0x06) == (eState&0x06))//&& dError < ErrorMax)
			{
                //T03矩阵，直接求解，优化耗时，其逆矩阵为转置矩阵
                dInvT03[0][0] = dCosA1*dCos23;
                dInvT03[0][1] = dSinA1*dCos23;
                dInvT03[0][2] = -dSin23;
                dInvT03[1][0] = -dSinA1;
                dInvT03[1][1] = dCosA1;
                dInvT03[1][2] = 0;
                dInvT03[2][0] = dCosA1*dSin23;
                dInvT03[2][1] = dSinA1*dSin23;
                dInvT03[2][2] = dCos23;
				bCalOK = true;
				break;
			}			
		}	
		if(bCalOK)
			break;
	}
	//再求后三个值，解析解求解
	if(bCalOK)
	{
		//J3代表T03，取得姿态矩阵，然后求逆
		double dT03[3][3] = {0};
		double dT06[3][3] = {0};

		double dJPosNew[6] = {0};
		dJPosNew[0] = dJPosCalc[0];
		dJPosNew[1] = dJPosCalc[1];
		dJPosNew[2] = dJPosCalc[2] - 90;

		double A1[16],A2[16],A3[16],A4[16],A5[16],A6[16];
        HS_TransformPlus(A1,dJPosNew[0],m_dDHPara[0][0],m_dDHPara[0][1],m_dDHPara[0][2],m_dDHPara[0][3]);
        HS_TransformPlus(A2,dJPosNew[1],m_dDHPara[1][0],m_dDHPara[1][1],m_dDHPara[1][2],m_dDHPara[1][3]);
        HS_TransformPlus(A3,dJPosNew[2],m_dDHPara[2][0],m_dDHPara[2][1],m_dDHPara[2][2],m_dDHPara[2][3]);

		double J2[16],J3[16],J4[16],J5[16];
		Matrix_Multi(4,4,4,A1,A2,J2);	
		Matrix_Multi(4,4,4,J2,A3,J3);

		for(int i = 0;i < 3;i++)
			for(int j = 0;j < 3;j++)
			{
				dT03[i][j] = J3[i*4 + j];
				dT06[i][j] = dFBMatrix[i][j];
			}

		//修改末端三轴求解方式,4,5,6轴按照正常求解
		Matrix_Inverse(3,&dT03[0][0],&dInvT03[0][0]);
		//求解T36
		double dT36[3][3] = {0};
		Matrix_Multi(3,3,3,&dInvT03[0][0],&dT06[0][0],&dT36[0][0]);

		//求解第5轴角度值
		double dCosAg5 = -dT36[2][2];
		if(bChange52)
			dCosAg5 = dT36[2][2];
		double dSinAg5 = sqrt(1 - dCosAg5*dCosAg5);

		if(!bWristQyFlag)
		{
			//根据5轴的形态进行判断
			if((eState&0x08) == 0x08)	//5轴为负
			{
				dJPosNew[4] = atan2(-dSinAg5,dCosAg5)*rad2deg;
				dSinAg5 = -dSinAg5;
			}
			else
			{
				dJPosNew[4] = atan2(dSinAg5,dCosAg5)*rad2deg;
			}

			if(fabs(dJPosNew[4]) > Eps)	//5轴为0，奇异位置无法求解
			{
				if(bChange52)
				{
					dJPosNew[3] = atan2(-dT36[1][2]/dSinAg5,-dT36[0][2]/dSinAg5)*rad2deg;
					dJPosNew[5] = atan2(dT36[2][1]/dSinAg5,dT36[2][0]/dSinAg5)*rad2deg;
				}
				else
				{
					dJPosNew[3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
					dJPosNew[5] = atan2(-dT36[2][1]/dSinAg5,dT36[2][0]/dSinAg5)*rad2deg;
				}
		
				//不考虑限位处理
				memcpy(dJPos,dJPosNew,sizeof(double)*6);

				for(int i = 0;i < 3;i++)				
				{				
					HS_NearestPoint(dJPos[i],dBaseJPos[i],-1);
				}
				return 0;
			}
			else
				return ERROR_UNREACHABLE;
		}
		else
		{
			//两组解，按照4/6轴的偏移小获取
			double dJPosOut[2][6] = {0};

			dJPosOut[0][4] = atan2(dSinAg5,dCosAg5)*rad2deg;
			dJPosOut[0][3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
			dJPosOut[0][5] = atan2(dT36[2][1]/dSinAg5,-dT36[2][0]/dSinAg5)*rad2deg;


			dJPosOut[1][4] = atan2(-dSinAg5,dCosAg5)*rad2deg;
			dSinAg5 = -dSinAg5;
			dJPosOut[1][3] = atan2(dT36[1][2]/dSinAg5,dT36[0][2]/dSinAg5)*rad2deg;
			dJPosOut[1][5] = atan2(dT36[2][1]/dSinAg5,-dT36[2][0]/dSinAg5)*rad2deg;

			double dDisA = fabs(dJPosOut[0][3] - dRefJPos[3]) + fabs(dJPosOut[0][5] - dRefJPos[5]);
			double dDisB = fabs(dJPosOut[1][3] - dRefJPos[3]) + fabs(dJPosOut[1][5] - dRefJPos[5]);

			if(dDisA < dDisB)
			{
				dJPosNew[3] = dJPosOut[0][3];
				dJPosNew[4] = dJPosOut[0][4];
				dJPosNew[5] = dJPosOut[0][5];
			}
			else
			{
				dJPosNew[3] = dJPosOut[1][3];
				dJPosNew[4] = dJPosOut[1][4];
				dJPosNew[5] = dJPosOut[1][5];
			}
			memcpy(dJPos,dJPosNew,sizeof(double)*6);
			return 0;
		}			
	}
	return ERROR_UNREACHABLE;
}
/************************************************
函数功能：通过数值迭代的方法进行求解
参    数：dMPos----待求空间齐次坐标
		 eState---形态
		 dSJPos---临近关节坐标
		 dJPos----输出关节坐标
返 回 值： 错误码
*************************************************/
int HS_Kinematics::HS_MPosToJPos_Iter(double dMPos[4][4],unsigned char eState,double dSJPos[6],double dJPos[6])
{	
	//先用初始值临近求解，应当适用于大部分点位，靠近奇异位置才容易求解不出
	//后续进行5轴奇异附近选解
	double dInitJPos[6] = {0};
	memcpy(dInitJPos,dSJPos,sizeof(double)*6);
	const int MAXINTER = 15;
	double MAXERROR = 1e-5;
	const double LIMITERROR = 800;
	int iCnt = 0;
	double dLJPos[6] = {0};
	bool bDisFlag = true;
	int iJacobCnt = 0;
	bool bQYHandleFlag = false;

	double dVelLimit = 50.0;
	double dWristLimit = 10.0;

	//初值状态保护处理，如果初值靠近奇异位置，会容易发散导致异常
	//处理策略：A：如果靠近边界奇异，则处理3轴坐标，对边界进行原理
	double dBorderLimit = 30;
	if(dInitJPos[2] < 90.0 + dBorderLimit&&dInitJPos[2] > 90.0)
		dInitJPos[2] = 90.0 + dBorderLimit;
	else if(dInitJPos[2] > 90.0 - dBorderLimit&&dInitJPos[2] < 90.0)
		dInitJPos[2] = 90.0 - dBorderLimit;
	else if(dInitJPos[2] < 270.0 + dBorderLimit&&dInitJPos[2] > 270.0)
		dInitJPos[2] = 270.0 + dBorderLimit;
	else if(dInitJPos[2] > 270.0 - dBorderLimit&&dInitJPos[2] < 270.0)
		dInitJPos[2] = 270.0 - dBorderLimit;

	memcpy(dLJPos,dInitJPos,sizeof(double)*6);

	double dBaseJPos[6] = {0,-90,180,0,0,0};
	for(int i = 0;i < 6;i++)				
	{
		if(i < 3)
			dBaseJPos[i] = (m_tLimitPara->dPmax[i] + m_tLimitPara->dPmin[i])/2;
		HS_NearestPoint(dLJPos[i],dBaseJPos[i],-1);
	}

	//m_HS_Printer->outDebugInfo("Motion_P","HS_Kinematic","MPosToJPos_JXJ",0,AllDeb,"dLJPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf!",\
	dLJPos[0],dLJPos[1],dLJPos[2],dLJPos[3],dLJPos[4],dLJPos[5]);

	double dJPosCalc[6] = {0};
	while(iCnt < MAXINTER)
	{	
		// HS_MPosToJPos(dMPos,dLJPos,dJPosCalc);

		//直接使用运算过程中的数据，并优化求解
		double dCVel[6] = {0};				//空间速度
		double dLMPos[4][4] = {0};			//上个周期的齐次变换矩阵	
		double dJVel[6] = {0};				//关节速度
		HS_JPosToMPos(dLJPos,dLMPos);
		HS_MPosToCVel(dMPos,dLMPos,dCVel);

		if(!bQYHandleFlag)
		{
			HS_CVToJV(dLJPos,dCVel,dJVel);	

			if(fabs(dLJPos[4]) < dWristLimit)
			{
				for(int i = 0;i < 6;i++)				
				{
					if(fabs(dJVel[i]) > dVelLimit)
					{
						bQYHandleFlag = true;
						memcpy(dLJPos,dInitJPos,sizeof(double)*6);
						HS_CVToJV_BR(dLJPos,dCVel,dJVel);
						LOG_ALGO("Wrist BR QY Handle!");
						break;
					}
				}
			}			
		}
		else
		{
			HS_CVToJV_BR(dLJPos,dCVel,dJVel);
		}
		iJacobCnt++;			

		double dMaxJVel = 0;
		for(int i = 0;i < 6;i++)				
		{
			dJPosCalc[i] = dLJPos[i] + dJVel[i];
			dMaxJVel = Max(dMaxJVel,fabs(dJVel[i]));
		}

		//m_HS_Printer->outDebugInfo("Motion_P","HS_Kinematic","MPosToJPos_JXJ",0,AllDeb,"dJPosCalc = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf!",\
		dJPosCalc[0],dJPosCalc[1],dJPosCalc[2],dJPosCalc[3],dJPosCalc[4],dJPosCalc[5]);

		for(int i = 0;i < 6;i++)				
		{
			HS_NearestPoint(dJPosCalc[i],dLJPos[i],-1);
		}

		//误差求解
		double dMPosT[4][4] = {0};
		//对误差进行求解判断
		HS_JPosToMPos(dJPosCalc,dMPosT);

		//先仅进行位置误差的处理
		double dError = fabs(dMPosT[0][3] - dMPos[0][3]) + fabs(dMPosT[1][3] - dMPos[1][3]) + fabs(dMPosT[2][3] - dMPos[2][3]);

		//增加腕部形态的判断
		bool bWristOKFlag = true;

		int iWristBit = (eState&0x08) >> 3;

		int iCurWrist = 0;
		if((dJPosCalc[4] <= 0&&dJPosCalc[4] > -180)||dJPosCalc[4] >= 180)
			iCurWrist = 1;

		if(iCurWrist != iWristBit)
			bWristOKFlag = false;

		if(bQYHandleFlag)
			MAXERROR = 0.01;

		if(dError < MAXERROR&&(bWristOKFlag||fabs(dJPosCalc[4]) < 5.0))
		{
			for(int i = 0;i < 6;i++)				
			{				
				HS_NearestPoint(dJPosCalc[i],dBaseJPos[i],-1);
			}
			memcpy(dJPos,dJPosCalc,sizeof(double)*6);
			return 0;
		}
		else if(dError > LIMITERROR&&iCnt > 3)
		{
			//求解失败
			break;
		}
		else
		{
			//增加判断是否靠近边界奇异和腕部奇异
			double dDisBorder = fabs(dLJPos[2] - 90.0);
			if(dDisBorder < fabs(dLJPos[2] - 270.0))
				dDisBorder = fabs(dLJPos[2] - 270.0);

			double dDisWrist = fabs(dLJPos[4]);

			bool bQYFlag = false;

			if(dDisBorder < 20||dDisWrist < 15)
				bQYFlag = true;

			//如果求解发散，则减小位移间隔，用临近点的方式逐渐逼近求解
			double dOrignDis = fabs(dCVel[0]) + fabs(dCVel[1]) + fabs(dCVel[2]);
			if(dMaxJVel > dVelLimit&&bDisFlag&&bQYFlag)
			{
				LOG_ALGO("LJDis JXJ = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf!",\
					dLJPos[0],dLJPos[1],dLJPos[2],dLJPos[3],dLJPos[4],dLJPos[5]);

				bDisFlag = false;
				double dOMPos[4][4] = {0};
				memcpy(dOMPos,dLMPos,sizeof(dOMPos));
				//memcpy(dLJPos,dInitJPos,sizeof(double)*6);      
				//增加求解发散的保护
				int N = 20;
				double dNJPos[6] = {0};

				for(int i = 1;i < N;i++)
				{
					dOMPos[0][3] = dLMPos[0][3] + dCVel[0]/N*i;
					dOMPos[1][3] = dLMPos[1][3] + dCVel[1]/N*i;
					dOMPos[2][3] = dLMPos[2][3] + dCVel[2]/N*i;
					HS_MPosToJPos(dOMPos,dLJPos,dNJPos);
					iJacobCnt++;
					for(int id = 0;id < 6;id++)
					{
						double dJVelT = dNJPos[id] - dLJPos[id];
						if(fabs(dJVelT) > dVelLimit)
						{
							bQYHandleFlag = true;
							memcpy(dLJPos,dInitJPos,sizeof(double)*6);
							LOG_ALGO("BR QY Handle!");
							break;
						}
					}
					if(bQYHandleFlag)
						break;
					memcpy(dLJPos,dNJPos,sizeof(double)*6);
				}
				iCnt ++;
			}
			else
			{
				iCnt ++;
				memcpy(dLJPos,dJPosCalc,sizeof(double)*6);
			}
		}
	}    

	if(bQYHandleFlag)
		return 0;

	if(iJacobCnt > 40)
	{
		LOG_ALGO("Jacob Cnt All = %d!",iJacobCnt);
		return -1;
	}    
	return -1;
}

/************************************************
函数功能：通过迭代的方法进行求解
参    数：
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_MPosToJPos_Iter_Cobot(double dMPos[4][4],unsigned char eState,double dSJPos[6],double dJPos[6])
{	
    //先用初始值临近求解，应当适用于大部分点位，靠近奇异位置才容易求解不出
    //后续进行5轴奇异附近选解
    double dInitJPos[6] = {0};
    memcpy(dInitJPos,dSJPos,sizeof(double)*6);
    const int MAXINTER = 15;
    double MAXERROR = 1e-5;
    const double LIMITERROR = 800;
    int iCnt = 0;
    double dLJPos[6] = {0};
    bool bDisFlag = true;
    int iJacobCnt = 0;
	bool bQYHandleFlag = false;

	double dBaseJPos[6] = {0,-90,90,-90,0,0};	
	double dVelLimit = 50.0;
	double dWristLimit = 10.0;

	//初值状态保护处理，如果初值靠近奇异位置，会容易发散导致异常
	//处理策略：A：如果靠近边界奇异，则处理3轴坐标，对边界进行原理
	double dBorderLimit = 20;
	if(dInitJPos[2] < 0.0 + dBorderLimit&&dInitJPos[2] > 0.0)
		dInitJPos[2] = 0.0 + dBorderLimit;
	else if(dInitJPos[2] > 0.0 - dBorderLimit&&dInitJPos[2] < 0.0)
		dInitJPos[2] = 0.0 - dBorderLimit;
	else if(dInitJPos[2] < 180.0 + dBorderLimit&&dInitJPos[2] > 180.0)
		dInitJPos[2] = 180.0 + dBorderLimit;
	else if(dInitJPos[2] > 180.0 - dBorderLimit&&dInitJPos[2] < 180.0)
		dInitJPos[2] = 180.0 - dBorderLimit;

    memcpy(dLJPos,dInitJPos,sizeof(double)*6);

	for(int i = 0;i < 6;i++)				
	{
		if(i < 3)
			dBaseJPos[i] = (m_tLimitPara->dPmax[i] + m_tLimitPara->dPmin[i])/2;
		HS_NearestPoint(dLJPos[i],dBaseJPos[i],-1);
	}
		
    double dJPosCalc[6] = {0};
    while(iCnt < MAXINTER)
    {	
        //直接使用运算过程中的数据，并优化求解
        double dCVel[6] = {0};				//空间速度
	    double dLMPos[4][4] = {0};			//上个周期的齐次变换矩阵	
	    double dJVel[6] = {0};				//关节速度
	    HS_JPosToMPos(dLJPos,dLMPos);
	    HS_MPosToCVel(dMPos,dLMPos,dCVel);

		if(!bQYHandleFlag)
		{
			HS_CVToJV(dLJPos,dCVel,dJVel);	

			if(fabs(dLJPos[4]) < dWristLimit)
			{
				for(int i = 0;i < 6;i++)				
				{
					if(fabs(dJVel[i]) > dVelLimit)
					{
						bQYHandleFlag = true;
						memcpy(dLJPos,dInitJPos,sizeof(double)*6);
						HS_CVToJV_BR(dLJPos,dCVel,dJVel);
						//LOG_ALGO("Cobot QY Handle!");
						break;
					}
				}
			}
		}
		else
		{
			HS_CVToJV_BR(dLJPos,dCVel,dJVel);
		}
        iJacobCnt++;			

        double dMaxJVel = 0;
 	    for(int i = 0;i < 6;i++)				
        {
		    dJPosCalc[i] = dLJPos[i] + dJVel[i];
            dMaxJVel = Max(dMaxJVel,fabs(dJVel[i]));
        }

		for(int i = 0;i < 6;i++)				
		{
			HS_NearestPoint(dJPosCalc[i],dLJPos[i],-1);
		}

        //误差求解
        double dMPosT[4][4] = {0};
        //对误差进行求解判断
        HS_JPosToMPos(dJPosCalc,dMPosT);

        //先仅进行位置误差的处理
        double dError = fabs(dMPosT[0][3] - dMPos[0][3]) + fabs(dMPosT[1][3] - dMPos[1][3]) + fabs(dMPosT[2][3] - dMPos[2][3]);

		//增加腕部形态的判断
		bool bWristOKFlag = true;

		int iWristBit = (eState&0x08) >> 3;

		//if((dJPosCalc[4] > Eps&&dJPosCalc[4] < 180)&&iWristBit == 1)
		//	bWristOKFlag = false;
		//else if((dJPosCalc[4] < -Eps||dJPosCalc[4] > 180)&&iWristBit == 0)
		//	bWristOKFlag = false;

		int iCurWrist = 0;
		if((dJPosCalc[4] <= 0&&dJPosCalc[4] > -180)||dJPosCalc[4] >= 180)
			iCurWrist = 1;
		
		if(iCurWrist != iWristBit)
			bWristOKFlag = false;
			
		if(bQYHandleFlag)
			MAXERROR = 0.01;

        if(dError < MAXERROR&&(bWristOKFlag||fabs(dJPosCalc[4]) < 5.0))
        {
			for(int i = 0;i < 6;i++)				
			{
				HS_NearestPoint(dJPosCalc[i],dBaseJPos[i],-1);
			}
            memcpy(dJPos,dJPosCalc,sizeof(double)*6);
            return 0;
        }
        else if(dError > LIMITERROR&&iCnt > 3)
        {
            //求解失败
            break;
        }
        else
        {
            //如果求解发散，则减小位移间隔，用临近点的方式逐渐逼近求解
            double dOrignDis = fabs(dCVel[0]) + fabs(dCVel[1]) + fabs(dCVel[2]);
            if(dMaxJVel > 30.0&&bDisFlag)
            {
				//m_HS_Printer->outDebugInfo("Motion_P","HS_Kinematic","MPosToJPos_JXJ",0,AllDeb,"LJDis JXJ = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf!",\
					dLJPos[0],dLJPos[1],dLJPos[2],dLJPos[3],dLJPos[4],dLJPos[5]);
                bDisFlag = false;

				//memcpy(dLJPos,dInitJPos,sizeof(double)*6);    
				//HS_JPosToMPos(dLJPos,dLMPos);
				//HS_MPosToCVel(dMPos,dLMPos,dCVel);

                double dOMPos[4][4] = {0};
                memcpy(dOMPos,dLMPos,sizeof(dOMPos));

				//增加求解发散的保护
                int N = 20;
				double dNJPos[6] = {0};
					
                for(int i = 1;i < N;i++)
                {
                    dOMPos[0][3] = dLMPos[0][3] + dCVel[0]/N*i;
                    dOMPos[1][3] = dLMPos[1][3] + dCVel[1]/N*i;
                    dOMPos[2][3] = dLMPos[2][3] + dCVel[2]/N*i;
                    HS_MPosToJPos(dOMPos,dLJPos,dNJPos);
                    iJacobCnt++;
					for(int id = 0;id < 6;id++)
					{
						double dJVelT = dNJPos[id] - dLJPos[id];
						if(fabs(dJVelT) > dVelLimit)
						{
							bQYHandleFlag = true;
							memcpy(dLJPos,dInitJPos,sizeof(double)*6);
							//LOG_ALGO("Cobot QY Handle DisNear!");
							break;
						}
					}
					if(bQYHandleFlag)
						break;
					memcpy(dLJPos,dNJPos,sizeof(double)*6);
                }
                iCnt ++;
            }
            else
            {
                iCnt ++;
                memcpy(dLJPos,dJPosCalc,sizeof(double)*6);
            }

        }
    }
    
	memcpy(dJPos,dJPosCalc,sizeof(double)*6);
	for(int i = 0;i < 6;i++)				
	{
		HS_NearestPoint(dJPos[i],dBaseJPos[i],-1);
	}

	if(bQYHandleFlag)
		return 0;

    if(iJacobCnt > 40)
    {
        LOG_ALGO("Jacob Cnt All = %d!",iJacobCnt);
        return -1;
    }    
	return -1;
}

/************************************************
函数功能：由当前齐次矩阵和上个周期的齐次矩阵求解空间速度
参    数：dpMPos--指令齐次矩阵
		 dpLMPos--上个周期的空间矩阵
		 dCVel---输出空间速度		 
返 回 值：无
*************************************************/
void HS_Kinematics::HS_MPosToCVel(double dCurMPos[4][4],double dLastMPos[4][4],double dCVel[6])
{
	double dRTransp[4][4] = {0};		//转置矩阵
	double dRotate[4][4] = {0};			//旋转矩阵
	double dRA[4] = {0};				//旋转轴角

	dCVel[0] = dCurMPos[0][3] - dLastMPos[0][3];
	dCVel[1] = dCurMPos[1][3] - dLastMPos[1][3];
	dCVel[2] = dCurMPos[2][3] - dLastMPos[2][3];

	Matrix_Transpose(4,4,&dLastMPos[0][0],&dRTransp[0][0]);	
	Matrix_Multi(4,4,4,&dRTransp[0][0],&dCurMPos[0][0],&dRotate[0][0]);

	double dRotateT[3][3] = {0};

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			dRotateT[i][j] = dRotate[i][j];

	Matrix_MToA(&dRotateT[0][0],dRA);

	double dTemp[3] = {0};
	dTemp[0] = dRA[1]*dRA[0];	//角速度
	dTemp[1] = dRA[2]*dRA[0];
	dTemp[2] = dRA[3]*dRA[0];

	double dCurMPosT[3][3] = {0};

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			dCurMPosT[i][j] = dCurMPos[i][j];

	//将角速度转化为当前
	Matrix_Multi(3,3,1,&dCurMPosT[0][0],dTemp,&dCVel[3]);
}

/************************************************
函数功能：由空间速度求解关节速度
参    数：pdJPos---关节角度		 
		 pdCVel--空间速度
		 pdJVel--关节速度
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_CVToJV_BR(double dJPos[6],double dCVel[6],double dJVel[6])
{
	double dJacobian[6][5];
	HS_JacobianQYWOpt(dJPos,dJacobian);
	double dJacobianT[5][6]; //转置矩阵
	Matrix_Transpose(6,5,&dJacobian[0][0],&dJacobianT[0][0]);
	double dMulti[5][5];
	Matrix_Multi(5,6,5,&dJacobianT[0][0],&dJacobian[0][0],&dMulti[0][0]);
	double dInv_ATA[5][5];
	if(!Matrix_Inverse(5,&dMulti[0][0],&dInv_ATA[0][0]))
		return ERROR_JOCAB_INV;
	double dInv_LeftA[5][6];
	Matrix_Multi(5,5,6,&dInv_ATA[0][0],&dJacobianT[0][0],&dInv_LeftA[0][0]);
	double dJVelTemp[5] = {0};
	Matrix_Multi(5,6,1,&dInv_LeftA[0][0],dCVel,dJVelTemp);
	dJVel[0] = dJVelTemp[0]*rad2deg;
	dJVel[1] = dJVelTemp[1]*rad2deg;
	dJVel[2] = dJVelTemp[2]*rad2deg;
	dJVel[4] = dJVelTemp[3]*rad2deg;
	dJVel[3] = 0;
	dJVel[5] = dJVelTemp[4]*rad2deg;
	return 0;
}
/************************************************
函数功能：由空间速度求解关节速度
参    数：dJPos---关节角度		 
		 dCVel--空间速度
		 dJVel--关节速度
         bTool--雅克比计算包含工具坐标系
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_CVToJV(double dJPos[6],double dCVel[6],double dJVel[6],bool bTool,int iToolNum,bool bWristQyFlag)
{
	int iErrorId = 0;
    if(*m_eRobotType == HSROB_SCARA || *m_eRobotType == HSROB_SCARA_3)
    {
        double dJacobian[6][4];
        iErrorId = HS_JacobianScara(dJPos,dJacobian,bTool,iToolNum);
        double dJacobianT[4][6]; //转置矩阵
        Matrix_Transpose(6,4,&dJacobian[0][0],&dJacobianT[0][0]);
        double dMulti[4][4];
        Matrix_Multi(4,6,4,&dJacobianT[0][0],&dJacobian[0][0],&dMulti[0][0]);
        double dInv_ATA[4][4];
        if(!Matrix_Inverse(4,&dMulti[0][0],&dInv_ATA[0][0]))
            return ERROR_JOCAB_INV;
        double dInv_LeftA[4][6];
        Matrix_Multi(4,4,6,&dInv_ATA[0][0],&dJacobianT[0][0],&dInv_LeftA[0][0]);
        //强制将ABC的速度为0
        //pdCVel[3] = 0;pdCVel[4] = 0;pdCVel[5] = 0;
        Matrix_Multi(4,6,1,&dInv_LeftA[0][0],dCVel,dJVel);
        dJVel[0] *= rad2deg;
        dJVel[1] *= rad2deg;
        dJVel[3] *= rad2deg;
    }
    else
    {
	    double Jacobian[6][6];        
        iErrorId = HS_Jacobian(dJPos,Jacobian,bTool,iToolNum);
	    if(iErrorId != 0)
		    return iErrorId;

	    if(*m_eRobotType == HSROB_PUMA_5||*m_eRobotType == HSROB_MD410||
			(bWristQyFlag&&*m_eRobotType == HSROB_PUMA))	
	    {
		    double dJacobian[6][5] = {0};

			if(*m_eRobotType == HSROB_PUMA)
			{ 
				for(int i = 0;i < 6;i++)
				{
					for(int j = 0;j < 5;j++)
					{
						if(j < 3)
							dJacobian[i][j] = Jacobian[i][j];
						else
							dJacobian[i][j] = Jacobian[i][j+1];
					}
				}
			}
			else
			{
				for(int i=0;i<6;i++)
				{
					for(int j=0;j<5;j++)
					{
						dJacobian[i][j] = Jacobian[i][j];
					}
				}
			}
		    
		    double dJacobianT[5][6]; //转置矩阵
		    Matrix_Transpose(6,5,&dJacobian[0][0],&dJacobianT[0][0]);
		    double dMulti[5][5];
		    Matrix_Multi(5,6,5,&dJacobianT[0][0],&dJacobian[0][0],&dMulti[0][0]);
		    double dInv_ATA[5][5];
		    if(!Matrix_Inverse(5,&dMulti[0][0],&dInv_ATA[0][0]))
			    return ERROR_JOCAB_INV;
		    double dInv_LeftA[5][6];
		    Matrix_Multi(5,5,6,&dInv_ATA[0][0],&dJacobianT[0][0],&dInv_LeftA[0][0]);
		    //强制将ABC的速度为0
		    //pdCVel[3] = 0;pdCVel[4] = 0;pdCVel[5] = 0;
		    Matrix_Multi(5,6,1,&dInv_LeftA[0][0],dCVel,dJVel);
		    dJVel[0] *= rad2deg;
		    dJVel[1] *= rad2deg;
		    dJVel[2] *= rad2deg;
		    dJVel[3] *= rad2deg;
		    dJVel[4] *= rad2deg;

			if(*m_eRobotType == HSROB_PUMA)
			{
				dJVel[5] = dJVel[4];
				dJVel[4] = dJVel[3];
				dJVel[3] = 0;
			}
	    }
	    else
	    {
		    double Inv_J[6][6];
		    if(!Matrix_Inverse(6,&Jacobian[0][0],&Inv_J[0][0]))
			    return ERROR_JOCAB_INV;

            double Inv_All[6][6];
            Matrix_Multi(6,6,6,&Jacobian[0][0],&Inv_J[0][0],&Inv_All[0][0]);

		    Matrix_Multi(6,6,1,&Inv_J[0][0],dCVel,dJVel);
		    dJVel[0] *= rad2deg;
		    dJVel[1] *= rad2deg;
		    dJVel[2] *= rad2deg;
		    dJVel[3] *= rad2deg;
		    dJVel[4] *= rad2deg;
		    dJVel[5] *= rad2deg; 

	    }
    }
	return iErrorId;
}
/************************************************
函数功能：计算雅克比矩阵【带工具坐标系】
参    数：pdJPos---关节角度[6]
		 dJacobian---雅克比矩阵[6][6]
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JacobianQYWOpt(double dJPos[4],double dJacobian[6][5])
{
	int iErrorId = 0;
	//添加β角
	double A1[16],A2[16],A3[16],A4[16],A5[16],A6Temp[16];
	HS_TransformPlus(A1,dJPos[0],m_dDHPara[0][0],m_dDHPara[0][1],m_dDHPara[0][2],m_dDHPara[0][3]);
	HS_TransformPlus(A2,dJPos[1],m_dDHPara[1][0],m_dDHPara[1][1],m_dDHPara[1][2],m_dDHPara[1][3]);
	HS_TransformPlus(A3,dJPos[2],m_dDHPara[2][0],m_dDHPara[2][1],m_dDHPara[2][2],m_dDHPara[2][3]);
	HS_TransformPlus(A4,dJPos[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]);
	HS_TransformPlus(A5,dJPos[4],m_dDHPara[4][0],m_dDHPara[4][1],m_dDHPara[4][2],m_dDHPara[4][3]);
	HS_TransformPlus(A6Temp,dJPos[5],m_dDHPara[5][0],m_dDHPara[5][1],m_dDHPara[5][2],m_dDHPara[5][3]); 

	double A6[16];
	//增加工具坐标系
	Matrix_Multi(4,4,4,A6Temp,&m_dTFMatrix[0][0],A6);	

	double J2[16],J3[16],J4[16],J5[16],J6[16];
	Matrix_Multi(4,4,4,A1,A2,J2);	
	Matrix_Multi(4,4,4,J2,A3,J3);
	Matrix_Multi(4,4,4,J3,A4,J4);
	Matrix_Multi(4,4,4,J4,A5,J5);
	Matrix_Multi(4,4,4,J5,A6,J6);

	double Z0[3],Z1[3],Z2[3],Z3[3],Z4[3],Z5[3];
	Z0[0] = 0;	  Z0[1] = 0;	Z0[2] = 1;
	Z1[0] = A1[2];Z1[1] = A1[6];Z1[2] = A1[10];
	Z2[0] = J2[2];Z2[1] = J2[6];Z2[2] = J2[10];
	Z3[0] = J3[2];Z3[1] = J3[6];Z3[2] = J3[10];
	Z4[0] = J4[2];Z4[1] = J4[6];Z4[2] = J4[10];
	Z5[0] = J5[2];Z5[1] = J5[6];Z5[2] = J5[10];
	
	double P0[3],P1[3],P2[3],P3[3],P4[3],P5[3],Pe[3];
	Pe[0] = J6[3];		  Pe[1] = J6[7];		Pe[2] = J6[11];
	P0[0] = Pe[0] - 0;    P0[1] = Pe[1] - 0;	P0[2] = Pe[2] - 0;
	P1[0] = Pe[0] - A1[3];P1[1] = Pe[1] - A1[7];P1[2] = Pe[2] - A1[11];
	P2[0] = Pe[0] - J2[3];P2[1] = Pe[1] - J2[7];P2[2] = Pe[2] - J2[11];
	P3[0] = Pe[0] - J3[3];P3[1] = Pe[1] - J3[7];P3[2] = Pe[2] - J3[11];
	P4[0] = Pe[0] - J4[3];P4[1] = Pe[1] - J4[7];P4[2] = Pe[2] - J4[11];
	P5[0] = Pe[0] - J5[3];P5[1] = Pe[1] - J5[7];P5[2] = Pe[2] - J5[11];	

	double J11[3],J12[3],J13[3],J14[3],J15[3],J16[3];
	Matrix_VecCross(Z0,P0,J11);
	Matrix_VecCross(Z1,P1,J12);
	Matrix_VecCross(Z2,P2,J13);
	Matrix_VecCross(Z3,P3,J14);
	Matrix_VecCross(Z4,P4,J15);
	Matrix_VecCross(Z5,P5,J16);

	dJacobian[0][0] = J11[0]; dJacobian[0][1] = J12[0];dJacobian[0][2] = J13[0];dJacobian[0][3] = J15[0];dJacobian[0][4] = J16[0];
	dJacobian[1][0] = J11[1]; dJacobian[1][1] = J12[1];dJacobian[1][2] = J13[1];dJacobian[1][3] = J15[1];dJacobian[1][4] = J16[1];
	dJacobian[2][0] = J11[2]; dJacobian[2][1] = J12[2];dJacobian[2][2] = J13[2];dJacobian[2][3] = J15[2];dJacobian[2][4] = J16[2];
	dJacobian[3][0] =  Z0[0]; dJacobian[3][1] =  Z1[0];dJacobian[3][2] =  Z2[0];dJacobian[3][3] =  Z4[0];dJacobian[3][4] =  Z5[0];
	dJacobian[4][0] =  Z0[1]; dJacobian[4][1] =  Z1[1];dJacobian[4][2] =  Z2[1];dJacobian[4][3] =  Z4[1];dJacobian[4][4] =  Z5[1];
	dJacobian[5][0] =  Z0[2]; dJacobian[5][1] =  Z1[2];dJacobian[5][2] =  Z2[2];dJacobian[5][3] =  Z4[2];dJacobian[5][4] =  Z5[2];
	return iErrorId;
}
/************************************************
函数功能：计算雅克比矩阵---Scara机型
参    数：pdJPos---关节角度[4]
		 dJacobian---雅克比矩阵[6][4]
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_JacobianScara(double dJPos[4],double dJacobian[6][4],bool bTool,int iToolNum)
{
	int iErrorId = 0;
	//齐次变换矩阵
	double A1[16],A2[16],A3[16],A4[16];

    //β角更改，添加独立的参数
    HS_TransformPlus(A1,dJPos[0],m_dDHPara[0][0],0,m_dDHPara[0][2],m_dDHPara[0][3]);
    HS_TransformPlus(A2,dJPos[1],m_dDHPara[1][0],0,m_dDHPara[1][2],m_dDHPara[1][3]);
    HS_TransformPlus(A3,        0,m_dDHPara[2][0],      dJPos[2],m_dDHPara[2][2],m_dDHPara[2][3]);
    HS_TransformPlus(A4,dJPos[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]);

	if(bTool)
	{
		double A4Temp[16];
		//增加工具坐标系
		double dTFMatrix[4][4] = {0};
		HS_CPosToMPos(m_dToolCoord[iToolNum],dTFMatrix);
		Matrix_Multi(4,4,4,A4,&dTFMatrix[0][0],A4Temp);

		memcpy(A4,A4Temp,sizeof(A4Temp));
	}

	double J2[16],J3[16],J4[16];
	Matrix_Multi(4,4,4,A1,A2,J2);	
	Matrix_Multi(4,4,4,J2,A3,J3);
	Matrix_Multi(4,4,4,J3,A4,J4);

	double Z0[3],Z1[3],Z2[3],Z3[3];
	Z0[0] = 0;	  Z0[1] = 0;	Z0[2] = 1;
	Z1[0] = A1[2];Z1[1] = A1[6];Z1[2] = A1[10];
	Z2[0] = J2[2];Z2[1] = J2[6];Z2[2] = J2[10];
	Z3[0] = J3[2];Z3[1] = J3[6];Z3[2] = J3[10];

	
	double P0[3],P1[3],P2[3],P3[3],Pe[3];
	Pe[0] = J4[3];		  Pe[1] = J4[7];		Pe[2] = J4[11];
	P0[0] = Pe[0] - 0;    P0[1] = Pe[1] - 0;	P0[2] = Pe[2] - 0;
	P1[0] = Pe[0] - A1[3];P1[1] = Pe[1] - A1[7];P1[2] = Pe[2] - A1[11];
	P2[0] = Pe[0] - J2[3];P2[1] = Pe[1] - J2[7];P2[2] = Pe[2] - J2[11];
	P3[0] = Pe[0] - J3[3];P3[1] = Pe[1] - J3[7];P3[2] = Pe[2] - J3[11];


	double J11[3],J12[3],J13[3],J14[3],J15[3],J16[3];
	Matrix_VecCross(Z0,P0,J11);
	Matrix_VecCross(Z1,P1,J12);
	Matrix_VecCross(Z2,P2,J13);
	Matrix_VecCross(Z3,P3,J14);
    
	dJacobian[0][0] = J11[0]; dJacobian[0][1] = J12[0];dJacobian[0][2] =  Z2[0];dJacobian[0][3] = J14[0];
	dJacobian[1][0] = J11[1]; dJacobian[1][1] = J12[1];dJacobian[1][2] =  Z2[1];dJacobian[1][3] = J14[1];
	dJacobian[2][0] = J11[2]; dJacobian[2][1] = J12[2];dJacobian[2][2] =  Z2[2];dJacobian[2][3] = J14[2];
	dJacobian[3][0] =  Z0[0]; dJacobian[3][1] =  Z1[0];dJacobian[3][2] =      0;dJacobian[3][3] =  Z3[0];
	dJacobian[4][0] =  Z0[1]; dJacobian[4][1] =  Z1[1];dJacobian[4][2] =      0;dJacobian[4][3] =  Z3[1];
	dJacobian[5][0] =  Z0[2]; dJacobian[5][1] =  Z1[2];dJacobian[5][2] =      0;dJacobian[5][3] =  Z3[2];
	return iErrorId;
}
/************************************************
函数功能：计算雅克比矩阵
参    数：dJPos---关节角度[6]
		 dJacobian---雅克比矩阵 六轴为[6][6] 四轴为[6][4]
         bTool-------是否附带工具【默认工具】【六轴有效】
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_Jacobian(double dJPos[6],double *dJacobian,bool bTool)
{
    int iErrorId = 0;
    if(*m_eRobotType == HSROB_PUMA)
    {
        double dJ[6][6] = {0};
        iErrorId = HS_Jacobian(dJPos,dJ,bTool);

        memcpy(dJacobian,dJ,sizeof(double)*36);
    }
    else if(*m_eRobotType == HSROB_SCARA)
    {
        double dJ[6][4] = {0};
        
        iErrorId = HS_JacobianScara(dJPos,dJ);

        memcpy(dJacobian,dJ,sizeof(double)*24);
    }

    return iErrorId;
}
/************************************************
函数功能：计算雅克比矩阵
参    数：pdJPos---关节角度[6]
		 dJacobian---雅克比矩阵[6][6]
		 bTool------是否使用工具雅可比
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_Jacobian(double dJPos[6],double dJacobian[6][6],bool bTool,int iToolNum)
{
	int iErrorId = 0;
    //添加β角
    double A1[16],A2[16],A3[16],A4[16],A5[16],A6[16];
    HS_TransformPlus(A1,dJPos[0],m_dDHPara[0][0],m_dDHPara[0][1],m_dDHPara[0][2],m_dDHPara[0][3]);
    HS_TransformPlus(A2,dJPos[1],m_dDHPara[1][0],m_dDHPara[1][1],m_dDHPara[1][2],m_dDHPara[1][3]);
    HS_TransformPlus(A3,dJPos[2],m_dDHPara[2][0],m_dDHPara[2][1],m_dDHPara[2][2],m_dDHPara[2][3]);
    HS_TransformPlus(A4,dJPos[3],m_dDHPara[3][0],m_dDHPara[3][1],m_dDHPara[3][2],m_dDHPara[3][3]);
    HS_TransformPlus(A5,dJPos[4],m_dDHPara[4][0],m_dDHPara[4][1],m_dDHPara[4][2],m_dDHPara[4][3]);
    HS_TransformPlus(A6,dJPos[5],m_dDHPara[5][0],m_dDHPara[5][1],m_dDHPara[5][2],m_dDHPara[5][3]); 

	if(bTool)
	{
		double A6Temp[16];
		//增加工具坐标系
		double dTFMatrix[4][4] = {0};
		HS_CPosToMPos(m_dToolCoord[iToolNum],dTFMatrix);
		Matrix_Multi(4,4,4,A6,&dTFMatrix[0][0],A6Temp);

		memcpy(A6,A6Temp,sizeof(A6Temp));
	}

	double J2[16],J3[16],J4[16],J5[16],J6[16];
	Matrix_Multi(4,4,4,A1,A2,J2);	
	Matrix_Multi(4,4,4,J2,A3,J3);
	Matrix_Multi(4,4,4,J3,A4,J4);
	Matrix_Multi(4,4,4,J4,A5,J5);
	Matrix_Multi(4,4,4,J5,A6,J6);

	double Z0[3],Z1[3],Z2[3],Z3[3],Z4[3],Z5[3];
	Z0[0] = 0;	  Z0[1] = 0;	Z0[2] = 1;
	Z1[0] = A1[2];Z1[1] = A1[6];Z1[2] = A1[10];
	Z2[0] = J2[2];Z2[1] = J2[6];Z2[2] = J2[10];
	Z3[0] = J3[2];Z3[1] = J3[6];Z3[2] = J3[10];
	Z4[0] = J4[2];Z4[1] = J4[6];Z4[2] = J4[10];
	Z5[0] = J5[2];Z5[1] = J5[6];Z5[2] = J5[10];
	
	double P0[3],P1[3],P2[3],P3[3],P4[3],P5[3],Pe[3];
	Pe[0] = J6[3];		  Pe[1] = J6[7];		Pe[2] = J6[11];
	P0[0] = Pe[0] - 0;    P0[1] = Pe[1] - 0;	P0[2] = Pe[2] - 0;
	P1[0] = Pe[0] - A1[3];P1[1] = Pe[1] - A1[7];P1[2] = Pe[2] - A1[11];
	P2[0] = Pe[0] - J2[3];P2[1] = Pe[1] - J2[7];P2[2] = Pe[2] - J2[11];
	P3[0] = Pe[0] - J3[3];P3[1] = Pe[1] - J3[7];P3[2] = Pe[2] - J3[11];
	P4[0] = Pe[0] - J4[3];P4[1] = Pe[1] - J4[7];P4[2] = Pe[2] - J4[11];
	P5[0] = Pe[0] - J5[3];P5[1] = Pe[1] - J5[7];P5[2] = Pe[2] - J5[11];	

	double J11[3],J12[3],J13[3],J14[3],J15[3],J16[3];
	Matrix_VecCross(Z0,P0,J11);
	Matrix_VecCross(Z1,P1,J12);
	Matrix_VecCross(Z2,P2,J13);
	Matrix_VecCross(Z3,P3,J14);
	Matrix_VecCross(Z4,P4,J15);
	Matrix_VecCross(Z5,P5,J16);

	dJacobian[0][0] = J11[0]; dJacobian[0][1] = J12[0];dJacobian[0][2] = J13[0];dJacobian[0][3] = J14[0];dJacobian[0][4] = J15[0];dJacobian[0][5] = J16[0];
	dJacobian[1][0] = J11[1]; dJacobian[1][1] = J12[1];dJacobian[1][2] = J13[1];dJacobian[1][3] = J14[1];dJacobian[1][4] = J15[1];dJacobian[1][5] = J16[1];
	dJacobian[2][0] = J11[2]; dJacobian[2][1] = J12[2];dJacobian[2][2] = J13[2];dJacobian[2][3] = J14[2];dJacobian[2][4] = J15[2];dJacobian[2][5] = J16[2];
	dJacobian[3][0] =  Z0[0]; dJacobian[3][1] =  Z1[0];dJacobian[3][2] =  Z2[0];dJacobian[3][3] =  Z3[0];dJacobian[3][4] =  Z4[0];dJacobian[3][5] =  Z5[0];
	dJacobian[4][0] =  Z0[1]; dJacobian[4][1] =  Z1[1];dJacobian[4][2] =  Z2[1];dJacobian[4][3] =  Z3[1];dJacobian[4][4] =  Z4[1];dJacobian[4][5] =  Z5[1];
	dJacobian[5][0] =  Z0[2]; dJacobian[5][1] =  Z1[2];dJacobian[5][2] =  Z2[2];dJacobian[5][3] =  Z3[2];dJacobian[5][4] =  Z4[2];dJacobian[5][5] =  Z5[2];
	return iErrorId;
}
/************************************************
函数功能：齐次变换矩阵逆解求关节坐标【奇异处理】【BR系列机型】
参    数：dMPos----齐次矩阵【输入】
		 dLJPos---上一个关节角【输入】
		 dCJPos---当前关节角度【输出】		 
返 回 值：错误Id 
*************************************************/
int HS_Kinematics::HS_MPosToJPos_QYHandle_BR(double dMPos[4][4],double dLJPos[6],double dCJPos[6])
{
    int iErrorId = 0;

    double dFBMPos[4][4] = {0};
    double dLJPosTemp[6] = {0};
    memcpy(dFBMPos,dMPos,sizeof(double)*16);
    memcpy(dLJPosTemp,dLJPos,sizeof(double)*6);
    double dLCPos[6] = {0};
    double dLMPosTemp[4][4] = {0};
    HS_JPosToMPos(dLJPosTemp,dLMPosTemp);
    double dLMPos[4][4] = {0};
    Matrix_Multi(4,4,4,&dLMPosTemp[0][0],&m_dTFMatrix[0][0],&dLMPos[0][0]);	
    double dCVel[6] = {0};				//空间速度
    HS_MPosToCVel(dFBMPos,dLMPos,dCVel);

    double dCVelSet[6] = {0};
    HS_MPosToCVel(dFBMPos,m_tQYHandle.dFBMPos,dCVelSet);

    //设定阻尼，约束4/6轴的速度值
    double dKDamp = 1.0;
    double dLimitAngle = 0.0;
    //限制姿态速度，避免由于阻尼导致规划的姿态速度越来越大
    if(fabs(dLJPosTemp[4]) < m_tQYHandle.dWristQYAngle)
    {
        //dCVel[3] = dCVelSet[3];
        //dCVel[4] = dCVelSet[4];
        //dCVel[5] = dCVelSet[5];
    }

    //对5轴进行保护，避免5轴直接为0导致雅可比求解异常无法运动
    double dJ5Limit = 0.001;
    if(fabs(dLJPosTemp[4]) < dJ5Limit)
    {
        if(dLJPosTemp[4] < 0)
            dLJPosTemp[4] = -dJ5Limit;
        else
            dLJPosTemp[4] = dJ5Limit;
    }

    double dJVel[6] = {0};

	double dJacobian[6][5];
	HS_JacobianQYWOpt(dLJPosTemp,dJacobian);
	double dJacobianT[5][6]; //转置矩阵
	Matrix_Transpose(6,5,&dJacobian[0][0],&dJacobianT[0][0]);
	double dMulti[5][5];
	Matrix_Multi(5,6,5,&dJacobianT[0][0],&dJacobian[0][0],&dMulti[0][0]);
	double dInv_ATA[5][5];
	if(!Matrix_Inverse(5,&dMulti[0][0],&dInv_ATA[0][0]))
		return ERROR_JOCAB_INV;
	double dInv_LeftA[5][6];
	Matrix_Multi(5,5,6,&dInv_ATA[0][0],&dJacobianT[0][0],&dInv_LeftA[0][0]);
	double dJVelTemp[5] = {0};
	Matrix_Multi(5,6,1,&dInv_LeftA[0][0],dCVel,dJVelTemp);
	dJVel[0] = dJVelTemp[0]*rad2deg;
	dJVel[1] = dJVelTemp[1]*rad2deg;
	dJVel[2] = dJVelTemp[2]*rad2deg;
	dJVel[3] = 0;
	dJVel[4] = dJVelTemp[3]*rad2deg;
	dJVel[5] = dJVelTemp[4]*rad2deg;
 
    for(int i = 0;i < 6;i++)		
    {
        dCJPos[i] = dLJPosTemp[i] + dJVel[i];

    }
 
    memcpy(m_tQYHandle.dFBMPos,dFBMPos,sizeof(double)*16);
    m_tQYHandle.bIsMoveing = true;
    return iErrorId;
}
/************************************************
函数功能：齐次变换矩阵逆解求关节坐标
参    数：dMPos----齐次矩阵【输入】
		 dLJPos---上一个关节角【输入】
		 dCJPos---当前关节角度【输出】		 
		 bTool----雅可比是否包含工具
		 iToolNum-工具编号
返 回 值：错误Id 
*************************************************/
int HS_Kinematics::HS_MPosToJPos(double dMPos[4][4],double dLJPos[6],double dCJPos[6],bool bTool,int iToolNum,bool bWristQyFlag)
{
	int iErrorId = 0;
	double dCVel[6] = {0};				//空间速度
	double dLMPos[5][4] = {0};			//上个周期的齐次变换矩阵	
	double dJVel[6] = {0};				//关节速度
	int iErrorID = 0;
    double dSLJPos[MaxAxisNum] = {0};            //缓存数据，用缓存数据进行计算，避免修改
    memcpy(dSLJPos,dLJPos,sizeof(double)*6);

	if(bTool)
	{
		double dLMPosTemp[5][4] = {0};
		HS_JPosToMPos(dSLJPos,dLMPosTemp);
		double dTFMatrix[4][4] = {0};
		HS_CPosToMPos(m_dToolCoord[iToolNum],dTFMatrix);
		Matrix_Multi(4,4,4,&dLMPosTemp[0][0],&dTFMatrix[0][0],&dLMPos[0][0]);	
	}
	else
		HS_JPosToMPos(dSLJPos,dLMPos);

	HS_MPosToCVel(dMPos,dLMPos,dCVel);

	if(*m_eRobotType == HSROB_MD410)
	{
		dSLJPos[4] = dSLJPos[3];
		dSLJPos[3] = -(dSLJPos[1] + dSLJPos[2]);
	}

	iErrorID = HS_CVToJV(dSLJPos,dCVel,dJVel,bTool,iToolNum,bWristQyFlag);
    
	for(int i = 0;i < 6;i++)				
		dCJPos[i] = dSLJPos[i] + dJVel[i];

    if(*m_eRobotType == HSROB_MD410)
    {
		dCJPos[3] = dCJPos[4];
		dCJPos[4] = 0;
	}

	if(*m_eRobotType == HSROB_SCARA_3)
		dCJPos[3] = 0;

	return iErrorId;
}

/************************************************
函数功能：将齐次矩阵坐标转换为空间坐标【欧拉角方式为ZYX】
参    数：pMPos---齐次矩阵坐标	
		 pCPos---空间坐标,X,Y,Z,A,B,C
返 回 值：错误Id
*************************************************/
int HS_Kinematics::HS_MPosToCPos(double dMPos[4][4],double dCPos[6])
{
	int iErrorId = 0;
	//ZYX方式的欧拉角
	double dTempCB = 0;			
	//将阈值判断标准由Eps改为1e-8,存在J->C->M与J->M，产生偏差导致报警的情况
	if(fabs(dMPos[2][0]-1.0) < 1e-8||fabs(dMPos[2][0]+1.0) < 1e-8)	//B角的奇异位置：A,C角度特殊处理，令C角为0，计算A角度值
	{
		//绕Y轴旋转的角度
		dCPos[PB] = atan2(-dMPos[2][0],sqrt(dMPos[2][1]*dMPos[2][1] + dMPos[2][2]*dMPos[2][2]));					
		//绕Z轴旋转的角度
		dCPos[PA] = atan2(-dMPos[0][1],dMPos[1][1]);	
		//绕X轴旋转的角度
		dCPos[PC] = 0;
	}
	else
	{
		//绕Y轴旋转的角度
		dCPos[PB] = atan2(-dMPos[2][0],sqrt(dMPos[2][1]*dMPos[2][1] + dMPos[2][2]*dMPos[2][2]));					
		//绕Z轴旋转的角度
		dCPos[PA] = atan2(dMPos[1][0]*1000,dMPos[0][0]*1000);	
		//绕X轴旋转的角度
		dCPos[PC] = atan2(dMPos[2][1]*1000,dMPos[2][2]*1000);
	}		

	//位置获取
	dCPos[PX] = dMPos[0][3];
	dCPos[PY] = dMPos[1][3];
	dCPos[PZ] = dMPos[2][3];
	//弧度更改为角度
	dCPos[PA] = dCPos[PA]*rad2deg;
	dCPos[PB] = dCPos[PB]*rad2deg;
	dCPos[PC] = dCPos[PC]*rad2deg;

    if(*m_eRobotType == HSROB_SCARA&&m_bScaraA360Flag)
    {
        HS_NearestPoint(dCPos[PA],m_dA360BaseAngle,-1);
    }

	return iErrorId;
}
/************************************************
函数功能：空间位置转换为齐次矩阵
参    数：dCPos---空间位置
		 dMPos---齐次矩阵
返 回 值：错误ID
*************************************************/
int HS_Kinematics::HS_CPosToMPos(double dCPos[6],double dMPos[4][4])
{
	int iErrorId = 0;	
	double dCosA = cos(dCPos[PA]*deg2rad);
	double dSinA = sin(dCPos[PA]*deg2rad);
	double dCosB = cos(dCPos[PB]*deg2rad);
	double dSinB = sin(dCPos[PB]*deg2rad);
	double dCosC = cos(dCPos[PC]*deg2rad);
	double dSinC = sin(dCPos[PC]*deg2rad);

	dMPos[0][0] = dCosA*dCosB;
	dMPos[1][0] = dSinA*dCosB;
	dMPos[2][0] = -dSinB;

	dMPos[0][1] = dCosA*dSinB*dSinC - dSinA*dCosC;
	dMPos[1][1] = dSinA*dSinB*dSinC + dCosA*dCosC;
	dMPos[2][1] = dCosB*dSinC;

	dMPos[0][2] = dCosA*dSinB*dCosC + dSinA*dSinC;
	dMPos[1][2] = dSinA*dSinB*dCosC - dCosA*dSinC;
	dMPos[2][2] = dCosB*dCosC;


	dMPos[0][3] = dCPos[0];
	dMPos[1][3] = dCPos[1];
	dMPos[2][3] = dCPos[2];

	dMPos[3][0] = 0;
	dMPos[3][1] = 0;
	dMPos[3][2] = 0;
	dMPos[3][3] = 1;
	
	return iErrorId;
}

/************************************************
函数功能：判断当前点位是否超过区域空间范围
参    数：
         dPos---当前关节角度
返 回 值：0--在正常范围之内
		 其他--报警码
*************************************************/
int HS_Kinematics::HS_JPosLimitCheck(double *dPos)
{	
	for(int i = 0;i < MaxAxisNum;i++)
	{
		//限位空间取得一定的范围，并且限位使能打开
		if(fabs(m_tLimitPara->dPmax[i] - m_tLimitPara->dPmin[i]) > Eps&&m_tLimitPara->bOpen[i])
		{
			if((dPos[i] > m_tLimitPara->dPmax[i] + Eps)||(dPos[i] < m_tLimitPara->dPmin[i] - Eps))
			{
				LOG_ALGO("Check JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
					dPos[0],dPos[1],dPos[2],dPos[3],dPos[4],dPos[5],dPos[6],dPos[7],dPos[8]);
				LOG_ALGO("PMAX = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
					m_tLimitPara->dPmax[0],m_tLimitPara->dPmax[1],m_tLimitPara->dPmax[2],m_tLimitPara->dPmax[3],m_tLimitPara->dPmax[4],
					m_tLimitPara->dPmax[5],m_tLimitPara->dPmax[6],m_tLimitPara->dPmax[7],m_tLimitPara->dPmax[8]);
				LOG_ALGO("PMIN = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
					m_tLimitPara->dPmin[0],m_tLimitPara->dPmin[1],m_tLimitPara->dPmin[2],m_tLimitPara->dPmin[3],m_tLimitPara->dPmin[4],
					m_tLimitPara->dPmin[5],m_tLimitPara->dPmin[6],m_tLimitPara->dPmin[7],m_tLimitPara->dPmin[8]);
				LOG_ALGO("Pos[%d][1~6] = %.3lf,PMIN = %.3lf,PMAX = %.3lf",
					i+1,dPos[i],m_tLimitPara->dPmin[i],m_tLimitPara->dPmax[i]);
				return ERROR_UNREACHABLE;
			}
			else
			{
				//增加一个MD410的联合限位判断（JR6300也做联合限位计算）
				// if(*m_eRobotType == HSROB_MD410 || *m_eRobotType_sub==JR6300)
				// {
				//     //MD410有联合限位，进行区别化处理
				//     double dM410LimitMax[MaxAxisNum] = {0};	
				//     double dM410LimitMin[MaxAxisNum] = {0};

				//     HS_GetMD410Limit(pdPos,dM410LimitMax,dM410LimitMin);
				//     if((pdPos[i] > dM410LimitMax[i] + Eps)||(pdPos[i] < dM410LimitMin[i] - Eps))
				//     {
				//         m_HS_Printer->outDebugInfo("Motion_P","Kinematic","PosSpaceCheck",0,AllDeb,"JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
				//             pdPos[0],pdPos[1],pdPos[2],pdPos[3],pdPos[4],pdPos[5],pdPos[6],pdPos[7],pdPos[8]);
				//         m_HS_Printer->outDebugInfo("Motion_P","Kinematic","PosSpaceCheck",0,AllDeb,"PMAX = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
				//             dM410LimitMax[0],dM410LimitMax[1],dM410LimitMax[2],dM410LimitMax[3],dM410LimitMax[4],
				//             dM410LimitMax[5],dM410LimitMax[6],dM410LimitMax[7],dM410LimitMax[8]);
				//         m_HS_Printer->outDebugInfo("Motion_P","Kinematic","PosSpaceCheck",0,AllDeb,"PMIN = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
				//             dM410LimitMin[0],dM410LimitMin[1],dM410LimitMin[2],dM410LimitMin[3],dM410LimitMin[4],
				//             dM410LimitMin[5],dM410LimitMin[6],dM410LimitMin[7],dM410LimitMin[8]);
				//         m_HS_Printer->outDebugInfo("Motion_P","Kinematic","PosReachable",0,AllDeb,"Pos%d[1~6] = %.3lf,PMIN = %.3lf,PMAX = %.3lf",
				//             i+1,pdPos[i],dM410LimitMin[i],dM410LimitMax[i]);
				//         if(pdPos[i] > dM410LimitMax[i] + Eps)
				//             return ERROR_LOW(i+1,ERROR_MD410_UNION_PMAX);
				//         else
				//             return ERROR_LOW(i+1,ERROR_MD410_UNION_PMIN);
				//     }
				// }
			}
		}			
	}			
	
	return 0;
}

/************************************************
函数功能：最近关节点位的求解与约束
参    数：
          dPosSet---待优化关节点位
          dRefPos---参考点位
          iLimitAxis---进行限位约束的轴号【-1表示不进行限位约束】
返 回 值： 0----求解成功
		 -1---求解失败
*************************************************/
int HS_Kinematics::HS_NearestPoint(double &dPosSet,double dRefPos,int iLimitAxis)
{
    double dMinDis = 10000;
    bool bOverLimit = false;
    double dJPos = 0;
    double dJPosOrig = dPosSet;
    for(int i = -4;i < 5;i++)
    {
        dJPos = dJPosOrig + 360*i;
        if(fabs(dJPos - dRefPos) < dMinDis)
        {
            bOverLimit = false;
            //限位保护
            if(iLimitAxis >= 0&&iLimitAxis <= 8)
            {
               if((dJPos > m_tLimitPara->dPmax[iLimitAxis]||dJPos < m_tLimitPara->dPmin[iLimitAxis])&&m_tLimitPara->bOpen[iLimitAxis])
                   bOverLimit = true;
            }

            if(!bOverLimit)
            {
                dMinDis = fabs(dJPos - dRefPos);
                dPosSet = dJPos;
            }
        }
    }
    return 0;
}
/************************************************
函数功能：关节位置4/6轴按照最短路径获取坐标【所有轴，除直线轴外部轴外，均按最短路径处理】
参    数：
          dSetJPos---待优化关节点位
          dRegJPos---参考点位
返 回 值：错误码
*************************************************/
int HS_Kinematics::HS_JPosNearestHandle(double dSetJPos[6],double dRegJPos[6])
{
    //特定超出180的需要进行选解
	if(*m_eRobotType == HSROB_PUMA)
	{
        HS_NearestPoint(dSetJPos[0],dRegJPos[0],-1);
		HS_NearestPoint(dSetJPos[3],dRegJPos[3],-1);
		HS_NearestPoint(dSetJPos[5],dRegJPos[5],-1);
		if(m_bCobot6Flag)
		{
			HS_NearestPoint(dSetJPos[4],dRegJPos[4],-1);
		}
	}
	else if(*m_eRobotType == HSROB_SCARA)
	{
        if(!m_bScaraA360Flag)
        {
            HS_NearestPoint(dSetJPos[3],dRegJPos[3],-1);
        }
	}
	return 0;
}

/************************************************
函数功能：自动适配空间点动的速度，加速度参数值
参   数： dCVel-----空间速度
		 dCAcc-----空间加速度
返 回 值：无
*************************************************/
int HS_Kinematics::AutoHandleCVel(double dCVel[2],double dCAcc[2])
{
    //约束点动空间速度最大值为250
	//   dCVel[0] = 250;
	////姿态速度由各轴的关节速度
	//   dCVel[1] = 120;
	//for(int i = 0;i < 6;i++)
	//{
	//	dCVel[1] = Min(dCVel[1],m_dJVelPara[i]);
	//}
	//加速度先获取各轴的最大加速度时间，以此为基准计算加速度
    double dTAccBase = 0;
    for(int i = 0;i < 6;i++)
    {
        double dTTemp = 0;
        if(m_dJAccPara[i] > Eps)
            dTTemp = fabs(m_dJVelPara[i]/m_dJAccPara[i]);
        dTAccBase = Max(dTAccBase,dTTemp);
    }

	dCAcc[0] = dCVel[0]/dTAccBase;
	dCAcc[1] = dCVel[1]/dTAccBase;
    return 0;
}
/************************************************
函数功能：获取机型类型
参    数：
返 回 值：机型类型	 
*************************************************/
HS_RobotType HS_Kinematics::GetRobotType()
{
    return *m_eRobotType;
}
/************************************************
函数功能：获取Scara机型A360是否开启标识位
参    数：
返 回 值：标识位	 
*************************************************/
bool HS_Kinematics::GetA360Flag()
{
    return m_bScaraA360Flag;
}
/************************************************
函数功能：空间位置逆解关节坐标【手动运行使用】
参    数：dCPos-----空间位置【输入】
		 dInitCPos--初始空间位置
		 dLJPos-----上一个关节位置【输入】
		 dCJPos-----当前关节位置【输出】
         dKCVel-----空间点动速度自适应约束比例
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPosToJPos_Hand(double dCPos[6],double dInitCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iHandMoveAxis)
{
	int iErrorId = 0;
    double dLastJPos[MaxAxisNum] = {0};
    memcpy(dLastJPos,dLJPos,sizeof(double)*MaxAxisNum);

	//手动位移变化
	double dMovePos[6] = {0};
	bool bAngleMove = false;		//是否为姿态点动
    int iMoveAxis = iHandMoveAxis;
    if(iMoveAxis > 2&&iMoveAxis < 6)
        bAngleMove = true;            

	if(bAngleMove)
	{
        dMovePos[iMoveAxis] = dCPos[iMoveAxis] - dInitCPos[iMoveAxis];
		double dMoveMPos[5][4] = {0};
		HS_CPosToMPos(dMovePos,dMoveMPos);
		double dInitMPos[5][4] = {0};
		HS_CPosToMPos(dInitCPos,dInitMPos);

		double dMPos[5][4] = {0};
		memcpy(dMPos,dInitMPos,sizeof(dMPos));
		Matrix_Multi(4,3,&dMoveMPos[0][0],&dInitMPos[0][0],&dMPos[0][0]);

		iErrorId = HS_MPosToJPos(dMPos,CP_ToolWork,dLastJPos,dCJPos);
	}
	else
	{
        //暂时只对6轴的XYZ运动进行腕部过奇异处理
        if(*m_eRobotType != HSROB_PUMA)
            iErrorId = HS_CPosToJPos(dCPos,CP_ToolWork,dLastJPos,dCJPos);
        else if(m_bWristQYHandleFlag)
        {
			if(m_bTypeBR||m_bCobot6Flag)
				HS_CPos2JPos_QYHandle_BR(dCPos,dLastJPos,dCJPos,dKCVel,iMoveAxis);   
			else
				HS_CPos2JPos_QYHandle(dCPos,dLastJPos,dCJPos,dKCVel,iMoveAxis);            
        }
        else
        {
            //限制空间点动的速度变化，使得关节轴的速度在约束范围内
            double dTWMPos[5][4] = {0};
            double dFBMPos[4][4] = {0};
            //转化为法兰盘坐标系下的速度差值进行计算
            HS_CPosToMPos(dCPos,dTWMPos);
            HS_TWMPosToFBMPos(dTWMPos,CP_ToolWork,dFBMPos);
            double dLCPos[MaxAxisNum] = {0};
            double dLMPos[5][4] = {0};
            HS_JPosToMPos(dLastJPos,dLMPos);
            double dCVel[6] = {0};				//空间速度
            HS_MPosToCVel(dFBMPos,dLMPos,dCVel);
            double dJVel[6] = {0};
            HS_CVToJV(dLastJPos,dCVel,dJVel);

            //计算速度倍率
            double dKJVel[6] = {0};
            const double LIMITKVEL = 0.4;       //限制倍率系数
            double dMaxKVel = 0;
            for(int i = 0;i < 6;i++)
            {
                if(m_dJVelPara[i] > Eps)
                    dKJVel[i] = fabs(dJVel[i]/m_dCycle/m_dJVelPara[i]);
                dMaxKVel = Max(dMaxKVel,dKJVel[i]);
            }            
            if(dMaxKVel > LIMITKVEL)
            {
                //迭代求解
                double dKVelDown = 2.0;     //降速比例
                double dKVelMin = 1.0;      //
                double dKVelMax = 1.0;
                double dCVelOrgin[6] = {0};
                memcpy(dCVelOrgin,dCVel,sizeof(double)*6);
                bool bLimitOK = false;
                double dKVel = 1.0;
                for(int iCnt = 0;iCnt < 10;iCnt++)
                {
                    //综合速度
                    for(int j = 0;j < 6;j++)
                    {
                        dCVel[j] = dCVelOrgin[j]/dKVelDown;
                    }
                    dKVel = dKVelDown;
                    HS_CVToJV(dLastJPos,dCVel,dJVel);
                    dMaxKVel = 0;
                    for(int i = 0;i < 6;i++)
                    {
                        if(m_dJVelPara[i] > Eps)
                            dKJVel[i] = fabs(dJVel[i]/m_dCycle/m_dJVelPara[i]);
                        dMaxKVel = Max(dMaxKVel,dKJVel[i]);
                    } 
                    if(bLimitOK)
                    {
                        if(dMaxKVel > LIMITKVEL)
                        {
                            dKVelMin = dKVelDown;
                        }
                        else
                        {
                            dKVelMax = dKVelDown;
                        }
                        dKVelDown = (dKVelMin + dKVelMax)/2;
                    }
                    else
                    {
                        if(dMaxKVel > LIMITKVEL)
                        {
                            dKVelDown = dKVelDown*2;
                        }
                        else
                        {
                            dKVelMax = dKVelDown;
                            dKVelMin = dKVelDown/2;
                            bLimitOK = true;
                            dKVelDown = (dKVelMin + dKVelMax)/2;
                        }
                    }
                } 
                HS_JPosToCPos(dLastJPos,CP_ToolWork,dLCPos);
                dCPos[iMoveAxis] = dLCPos[iMoveAxis] + (dCPos[iMoveAxis] - dLCPos[iMoveAxis])/dKVel;
                dKCVel = dKVelDown;
            }

            for(int i = 0;i < 6;i++)				
                dCJPos[i] = dLastJPos[i] + dJVel[i];
        }
	}
	return iErrorId;
}
/************************************************
函数功能：空间位置逆解关节坐标【手动运行使用】---包含奇异处理【BR系列带偏置的处理】
		采用固定4轴坐标的方式进行处理
参    数：dCPos-----空间位置【输入】
		 dLJPos-----上一个关节位置【输入】
		 dCJPos-----当前关节位置【输出】
         dKCVel-----空间点动速度自适应约束比例
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPos2JPos_QYHandle_BR(double dCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iMoveAxis)
{
    //腕部奇异的阻尼处理方案:【由法兰盘的速度位置计算修改为工具点的计算】
    //将法兰盘相对于基坐标系，改为工具相对于基坐标系
    double dTWMPos[4][4] = {0};
    double dFBMPos[4][4] = {0};
    double dLJPosTemp[6] = {0};
    memcpy(dLJPosTemp,dLJPos,sizeof(double)*6);
    //转化为法兰盘坐标系下的速度差值进行计算
    HS_CPosToMPos(dCPos,dTWMPos);
    HS_TWMPosToFBMPos(dTWMPos,CP_ToolWork,dFBMPos);
    double dLCPos[6] = {0};
    double dLMPos[4][4] = {0};
    HS_JPosToMPos(dLJPosTemp,CP_Flange,dLMPos);
    double dCVel[6] = {0};				//空间速度

    //工具相对于基坐标系
    double dTBMPos[4][4] = {0};
    double dLTBMPos[4][4] = {0};

    Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dTFMatrix[0][0],&dTBMPos[0][0]);	
    Matrix_Multi(4,4,4,&dLMPos[0][0],&m_dTFMatrix[0][0],&dLTBMPos[0][0]);	

    HS_MPosToCVel(dTBMPos,dLTBMPos,dCVel);

    //设定阻尼，约束4/6轴的速度值
    double dKDamp = 1.0;
    double dLimitAngle = 0.0;
    //限制姿态速度，避免由于阻尼导致规划的姿态速度越来越大
    if(fabs(dLJPosTemp[4]) < m_tQYHandle.dWristQYAngle)
    {
        dCVel[3] = 0;
        dCVel[4] = 0;
        dCVel[5] = 0;
    }

    double dJVel[6] = {0};

    //对5轴进行保护，避免5轴直接为0导致雅可比求解异常无法运动
    double dJ5Limit = 0.001;
    if(fabs(dLJPosTemp[4]) < dJ5Limit)
    {
        if(dLJPosTemp[4] < 0)
            dLJPosTemp[4] = -dJ5Limit;
        else
            dLJPosTemp[4] = dJ5Limit;
    }

	double dJacobian[6][5];
	HS_JacobianQYWOpt(dLJPosTemp,dJacobian);
	double dJacobianT[5][6]; //转置矩阵
	Matrix_Transpose(6,5,&dJacobian[0][0],&dJacobianT[0][0]);
	double dMulti[5][5];
	Matrix_Multi(5,6,5,&dJacobianT[0][0],&dJacobian[0][0],&dMulti[0][0]);
	double dInv_ATA[5][5];
	if(!Matrix_Inverse(5,&dMulti[0][0],&dInv_ATA[0][0]))
		return ERROR_JOCAB_INV;
	double dInv_LeftA[5][6];
	Matrix_Multi(5,5,6,&dInv_ATA[0][0],&dJacobianT[0][0],&dInv_LeftA[0][0]);
	double dJVelTemp[5] = {0};
	Matrix_Multi(5,6,1,&dInv_LeftA[0][0],dCVel,dJVelTemp);
	dJVel[0] = dJVelTemp[0]*rad2deg;
	dJVel[1] = dJVelTemp[1]*rad2deg;
	dJVel[2] = dJVelTemp[2]*rad2deg;
	dJVel[4] = dJVelTemp[3]*rad2deg;
	dJVel[3] = 0;
	dJVel[5] = dJVelTemp[4]*rad2deg;

    for(int i = 0;i < 6;i++)		
    {
        dCJPos[i] = dLJPosTemp[i] + dJVel[i];
    }
    return 0;
}

/************************************************
函数功能：空间位置逆解关节坐标【手动运行使用】---包含奇异处理
参    数：dCPos-----空间位置【输入】
		 dLJPos-----上一个关节位置【输入】
		 dCJPos-----当前关节位置【输出】
         dKCVel-----空间点动速度自适应约束比例
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPos2JPos_QYHandle(double dCPos[6],double dLJPos[6],double dCJPos[6],double &dKCVel,int iMoveAxis)
{
    //腕部奇异的阻尼处理方案:【由法兰盘的速度位置计算修改为工具点的计算】
    //将法兰盘相对于基坐标系，改为工具相对于基坐标系
    double dTWMPos[4][4] = {0};
    double dFBMPos[4][4] = {0};
    double dLJPosTemp[MaxAxisNum] = {0};
    memcpy(dLJPosTemp,dLJPos,sizeof(double)*6);
    //转化为法兰盘坐标系下的速度差值进行计算
    HS_CPosToMPos(dCPos,dTWMPos);
    HS_TWMPosToFBMPos(dTWMPos,CP_ToolWork,dFBMPos);
    double dLCPos[MaxAxisNum] = {0};
    double dLMPos[4][4] = {0};
    HS_JPosToMPos(dLJPosTemp,CP_Flange,dLMPos);
    double dCVel[6] = {0};				//空间速度

    //工具相对于基坐标系
    double dTBMPos[4][4] = {0};
    double dLTBMPos[4][4] = {0};

	Matrix_Multi(4,4,4,&dFBMPos[0][0],&m_dTFMatrix[0][0],&dTBMPos[0][0]);	
	Matrix_Multi(4,4,4,&dLMPos[0][0],&m_dTFMatrix[0][0],&dLTBMPos[0][0]);	

    HS_MPosToCVel(dTBMPos,dLTBMPos,dCVel);

    //设定阻尼，约束4/6轴的速度值
    double dKDamp = 1.0;
    double dLimitAngle = 0.0;
    //限制姿态速度，避免由于阻尼导致规划的姿态速度越来越大
    if(fabs(dLJPosTemp[4]) < m_tQYHandle.dWristQYAngle)
    {
        dCVel[3] = 0;
        dCVel[4] = 0;
        dCVel[5] = 0;
    }

    double dJVel[6] = {0};

    //对5轴进行保护，避免5轴直接为0导致雅可比求解异常无法运动
    double dJ5Limit = 0.001;
    if(fabs(dLJPosTemp[4]) < dJ5Limit)
    {
        if(dLJPosTemp[4] < 0)
            dLJPosTemp[4] = -dJ5Limit;
        else
            dLJPosTemp[4] = dJ5Limit;
    }

    HS_CVToJV(dLJPosTemp,dCVel,dJVel,true);

    double dJXLimit = 20;
    //如果初始进入，5轴就较小，需要直接进行奇异区间处理
    if(!m_tQYHandle.bIsMoveing&&fabs(dLJPosTemp[4]) < dJXLimit)
    {
        //如果角度较小，速度也较小，则表示当前应该时启动就处于腕部区间内，则应当放大阈值以及限制条件
        m_tQYHandle.dWristQYAngle = 20;
        m_tQYHandle.dWristKDamp = m_tQYHandle.dWristQYAngle/40 + 0.1;
        m_tQYHandle.bSetQYAngle = true;
        LOG_ALGO("Start Move Angle5 = %.3lf,KDamp = %.3lf!",dLJPosTemp[4],m_tQYHandle.dWristKDamp);
    }

    if(fabs(dLJPosTemp[4]) < m_tQYHandle.dWristQYAngle)
    {
        double dKW = fabs(sin(angle2Rad(m_tQYHandle.dWristQYAngle)));
        double dKAngle = fabs(sin(angle2Rad(dLJPosTemp[4])));
        if(dKAngle > Eps)
        {
            double dTemp = (1 - dKW/dKAngle)*(1 - dKW/dKAngle)*m_tQYHandle.dWristKDamp;
            dKDamp = dKW*dKW/(dKW*dKW + dTemp);
        }
        else
            dKDamp = 0;
        dJVel[3] *= dKDamp;
        dJVel[5] *= dKDamp;
    }
    else
    {
        if(m_tQYHandle.bQYWristInFlag)
        {
            //出腕部奇异，要对姿态速度进行限制,固定捷度限制，约束最大加速度和最大速度值
            //基于比例迭代约束处理【取消】
            m_tQYHandle.bQYQuitHandleFLag = true;
            m_tQYHandle.bSetQYAngle = false;
            //求解姿态偏差，进行补偿运动的规划
            //修改当前的CPos为新的位姿【主要改姿态】，并计算出误差量
            double dNewFBMPos[4][4] = {0};
            memcpy(dNewFBMPos,dLMPos,sizeof(double)*16);
            dNewFBMPos[0][3] = dFBMPos[0][3];
            dNewFBMPos[1][3] = dFBMPos[1][3];
            dNewFBMPos[2][3] = dFBMPos[2][3];

            HS_MPosToCVel(dNewFBMPos,dLMPos,dCVel);
            HS_CVToJV(dLJPosTemp,dCVel,dJVel,true);

            HS_MPosToCPos(dNewFBMPos,dCPos);
        }

        if(m_tQYHandle.bQYQuitHandleFLag)
        {
            //HS_QYQuitWristHandle(dLJPosTemp,dCVel,dJVel);
        }
    }

    if(fabs(dLJPosTemp[4]) < m_tQYHandle.dWristQYAngle)
        m_tQYHandle.bQYWristInFlag = true;
    else 
        m_tQYHandle.bQYWristInFlag = false;

    //指标量计算：速度、加速度和加加速度值
    double dJVelS[6] = {0};
    double dJAccS[6] = {0};
    double dJJerkS[6] = {0};
    for(int i = 0;i < 6;i++)		
    {
        dCJPos[i] = dLJPosTemp[i] + dJVel[i];
        dJVelS[i] = dJVel[i]/m_dCycle;
        dJAccS[i] = (dJVelS[i]- m_tQYHandle.dJVel[i])/m_dCycle;
        dJJerkS[i] = (dJAccS[i] - m_tQYHandle.dJAcc[i])/m_dCycle;
    }

    memcpy(m_tQYHandle.dJVel,dJVelS,sizeof(double)*6);
    memcpy(m_tQYHandle.dJAcc,dJAccS,sizeof(double)*6);

    double dMaxKVel = 0;
    double dMaxKAcc = 0;
    double dMaxKJerk = 0;
    double dKJVel[6] = {0};
    double dKJAcc[6] = {0};
    double dKJJerk[6] = {0};
    for(int i = 0;i < 6;i++)
    {
        if(m_dJVelPara[i] > Eps)
            dKJVel[i] = fabs(m_tQYHandle.dJVel[i]/m_dJVelPara[i]);
        if(m_dJAccPara[i] > Eps)
        {
            dKJAcc[i] = fabs(m_tQYHandle.dJAcc[i]/m_dJAccPara[i]);
            dKJJerk[i] = fabs(dJJerkS[i]/m_dJAccPara[i]*0.15);       //0.15ms加速至AMax作为参考捷度
        }
        dMaxKVel = Max(dMaxKVel,dKJVel[i]);    
        dMaxKAcc = Max(dMaxKAcc,dKJAcc[i]);  
        dMaxKJerk = Max(dMaxKJerk,dKJJerk[i]); 
    } 

    //奇异约束的判断条件，开启进行奇异处理的判断条件较为重要，会影响奇异约束处理的效果以及可能产生的超速和超加速等问题
    //角度值越小，奇异作用也越大，则判断的阈值要适当放宽，同时，阻尼的影响因子作用也会更大
    //动态阈值
    const double dQYWristLimit = 25;
    if((dMaxKVel > 0.4||dMaxKAcc > 0.4||dMaxKJerk > 0.3)&&!m_tQYHandle.bSetQYAngle&&fabs(dLJPosTemp[4]) < dQYWristLimit)
    {
        m_tQYHandle.dWristQYAngle = fabs(dLJPosTemp[4]);
        //如果角度较小，速度也较小，则表示当前应该时启动就处于腕部区间内，则应当放大阈值以及限制条件***修改，容易引发加速度突变
        //double dSetCVel = dCVel[iMoveAxis]/m_dCycle;
        //if(fabs(dSetCVel) < 2.0&&dMaxKVel < 0.4)
        //    m_tQYHandle.dWristQYAngle = 20;

        m_tQYHandle.dWristKDamp = m_tQYHandle.dWristQYAngle/40 + 0.1;
        m_tQYHandle.bSetQYAngle = true;
        LOG_ALGO("KVel = %.3lf,KAcc = %.3lf,KJerk = %.3lf,Angle5 = %.3lf,KDamp = %.3lf!",dMaxKVel,dMaxKAcc,dMaxKJerk,dLJPosTemp[4],m_tQYHandle.dWristKDamp);
    }

    //空间速度的自适应处理，通过约束空间速度的变化，减小关节速度/加速度的变化，以此通过奇异空间
    if(dMaxKVel > 0.4&&!m_tQYHandle.bSetQYAngle)
    {
        //速度的自适应限制处理
        HS_CVelAutoLimit(dCVel,dLJPosTemp,dJVel,dKCVel,iMoveAxis);

        HS_JPosToCPos(dLJPosTemp,CP_ToolWork,dLCPos);
        dCPos[iMoveAxis] = dLCPos[iMoveAxis] + (dCPos[iMoveAxis] - dLCPos[iMoveAxis])/dKCVel;

        for(int i = 0;i < 6;i++)				
            dCJPos[i] = dLJPosTemp[i] + dJVel[i];
    }
    m_tQYHandle.bIsMoveing = true;
    return 0;
}

/************************************************
函数功能：空间速度自适应处理，奇异区间约束
参    数：
         dCVel-----待约束空间速度值
         dLJPos----上一次的关节位置
         dJVel-----求解得到的关节速度【输出】
         dKCVel----空间速度约束比例
         iMoveAxis--点动运动轴
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CVelAutoLimit(double dCVel[6],double dLJPos[6],double dJVel[6],double &dKCVel,int iMoveAxis)
{
    //迭代求解
    double dKVelDown = 2.0;     //降速比例
    double dKVelMin = 1.0;      //
    double dKVelMax = 1.0;
    double dCVelOrgin[6] = {0};
    memcpy(dCVelOrgin,dCVel,sizeof(double)*6);
    double dLJPosTemp[6] = {0};
    memcpy(dLJPosTemp,dLJPos,sizeof(double)*6);
    double dJVelOut[6] = {0};
    double dJCVelCalc[6] = {0};
    memcpy(dJCVelCalc,dCVel,sizeof(double)*6);
    bool bLimitOK = false;
    double dKVel = 1.0;
    bool bVelDir = false;           //当前约束速度的方向，即如果初始速度>上个周期的速度，不能使得速度 < 上个周期的速度值
    int iMaxAxis = 0;
    for(int iCnt = 0;iCnt < 10;iCnt++)
    {
        //综合速度
        dJCVelCalc[iMoveAxis] = dCVelOrgin[iMoveAxis]/dKVelDown;    
        dKVel = dKVelDown;
        HS_CVToJV(dLJPosTemp,dJCVelCalc,dJVelOut,true);
        bool bOverPara = false;         //超出规划参数，需要进行限制//判断速度和加速度
        double dKJVel[6] = {0};
        double dKJAcc[6] = {0};
        double dMaxKVel = 0;
        double dMaxVel = 0;
        for(int i = 0;i < 6;i++)
        {
            if(m_dJVelPara[i] > Eps)
                dKJVel[i] = fabs(dJVelOut[i]/m_dCycle/m_dJVelPara[i]);
            dMaxKVel = Max(dMaxKVel,dKJVel[i]);        
        } 

        if(dMaxKVel > 0.4)
            bOverPara = true;
        if(bLimitOK)
        {
            if(bOverPara)
            {
                dKVelMin = dKVelDown;
            }
            else
            {
                dKVelMax = dKVelDown;
            }
            dKVelDown = (dKVelMin + dKVelMax)/2;
        }
        else
        {
            if(bOverPara)
            {
                dKVelDown = dKVelDown*2;
            }
            else
            {
                dKVelMax = dKVelDown;
                dKVelMin = dKVelDown/2;
                bLimitOK = true;
                dKVelDown = (dKVelMin + dKVelMax)/2;
            }
        }
    }
    dKCVel = dKVelDown;
    memcpy(dJVel,dJVelOut,sizeof(double)*6);
    return 0;
}
/************************************************
函数功能：空间位置逆解关节坐标【手动运行使用,前瞻处理】
参    数：dCPos-----空间位置【输入】
		 dInitCPos--初始空间位置
		 dLJPos-----上一个关节位置【输入】
		 dCJPos-----当前关节位置【输出】
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_CPosToJPos_HandAhead(double dCPos[6],double dInitCPos[6],double dLJPos[6],double dCJPos[6])
{
	int iErrorId = 0;
	//手动位移变化
	double dMovePos[6] = {0};
	bool bAngleMove = false;		//是否为姿态点动
    unsigned char eState = HS_JPosToAState(dLJPos);
	for(int i = 0;i < 6;i++)
	{
		dMovePos[i] = dCPos[i] - dInitCPos[i];
		if((fabs(dMovePos[i]) > Eps)&&i > 2)
			bAngleMove = true;
	}
    double dMPos[4][4] = {0};
	if(bAngleMove)
	{
		double dMoveMPos[4][4] = {0};
		HS_CPosToMPos(dMovePos,dMoveMPos);
		double dInitMPos[4][4] = {0};
		HS_CPosToMPos(dInitCPos,dInitMPos);

        memcpy(dMPos,dInitMPos,sizeof(dMPos));
		Matrix_Multi(4,3,&dMoveMPos[0][0],&dInitMPos[0][0],&dMPos[0][0]);
	}
	else
	{
        HS_CPosToMPos(dCPos,dMPos);	
	}

    double dFBMPos[4][4] = {0};
    HS_TWMPosToFBMPos(dMPos,CP_ToolWork,dFBMPos);

    double dJPosOut[MaxAxisNum] = {0};
    iErrorId = HS_MPosToJPos_JXJ(dFBMPos,-1,-1,eState,CP_ToolWork,dJPosOut);
    if(iErrorId != 0)
        return ERROR_C2J;

    //添加检测处理，如果dLJPos和dCJPos的形态不一致，则报警【BR系列机型可能出现求解异常的情况】
    unsigned char eStateOut = HS_JPosToAState(dJPosOut);
    //////////////////////////////////////
    if((eState&0x0F) != (eStateOut&0x0F)&& *m_eRobotType != HSROB_SCARA_3)
    {
        LOG_ALGO("CJPos:%.3lf,%.3lf%.3lf,%.3lf,%.3lf,%.3lf;eState = %d,Out = %d",
            dJPosOut[0],dJPosOut[1],dJPosOut[2],dJPosOut[3],dJPosOut[4],dJPosOut[5],(int)eState,(int)eStateOut);
        return -1;
    }

    memcpy(dCJPos,dJPosOut,sizeof(double)*6);
    HS_JPosNearestHandle(dCJPos,dLJPos);
	return iErrorId;
}
/************************************************
函数功能：轴组相对位置关系转化
参    数：dMasterCPos----主运动位置
	     dSlaveCPos-----从运动位置
		 dRelativeCPos--相对运动位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_SyncRelativePosChange(double dMasterCPos[6],double dSlaveCPos[6],double dRelativeCPos[6])
{
	int iErrorId = 0;

	double dMaWTMPos[4][4] = {0};
	HS_CPosToMPos(dMasterCPos,dMaWTMPos);

	double dSlWTMPos[4][4] = {0};
	HS_CPosToMPos(dSlaveCPos,dSlWTMPos);

	double dMaTWMPos[4][4] = {0};
	HS_Math::Matrix_Inverse(4,&dMaWTMPos[0][0],&dMaTWMPos[0][0]);

	double dRelativeMPos[4][4] = {0};
	HS_Math::Matrix_Multi(4,4,4,&dMaTWMPos[0][0],&dSlWTMPos[0][0],&dRelativeMPos[0][0]);

	HS_MPosToCPos(dRelativeMPos,dRelativeCPos);

	return iErrorId;
}
/************************************************
函数功能：变位机坐标转化，由世界坐标位置转化为变位机坐标位置
参    数：dTWCPos----工具在世界坐标系下的位置
	     dTCCPos----工具在变位机坐标系下的位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_TWPosToTCPos(double dTWCPos[MaxAxisNum],double dTCCPos[MaxAxisNum])
{
	int iErrorId = 0;

	double dTWMPos[4][4] = {0};
	HS_CPosToMPos(dTWCPos,dTWMPos);		                                    //W/T

	double dCWMPos[4][4] = {0};
	HS_JPosToPoMPos(&dTWCPos[6],dCWMPos);									//W/C

	double dWCMPos[4][4] = {0};
	Matrix_Inverse(4,&dCWMPos[0][0],&dWCMPos[0][0]);						//C/W

	double dTCMPos[4][4] = {0};
	Matrix_Multi(4,4,&dWCMPos[0][0],&dTWMPos[0][0],&dTCMPos[0][0]);			//C/W*W/T = C/T

	HS_MPosToCPos(dTCMPos,dTCCPos);

	return iErrorId;
}
/************************************************
函数功能：变位机坐标转化，由变位机坐标位置转化为世界坐标位置
参    数：dTCMPos----工具在变位机坐标系下的位置
	     dTWMPos----工具在世界坐标系下的位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_TCMPosToTWMPos(double dTCMPos[5][4],double dTWMPos[5][4])
{
	int iErrorId = 0;

	memcpy(dTWMPos,dTCMPos,sizeof(double)*20);

	//计算出当前的变位机矩阵
	double dCWMPos[4][4] = {0};
	HS_JPosToPoMPos(&dTCMPos[4][0],dCWMPos);	                             //W/C

	//转换至基坐标系
	double dTBMPos[4][4] = {0};
	Matrix_Multi(4,4,4,&dCWMPos[0][0],&dTCMPos[0][0],&dTWMPos[0][0]);        //W/C*C/T = W/T

	return iErrorId;
}

/************************************************
函数功能：变位机坐标,点动附加轴保证相对位置不变
参    数：
		 dBaseTCMPos----固定工具相对于变位机坐标
		 iToolNum-------工具号
	     dRJPos---------关节坐标值
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_TCMPosToJPos(double dBaseTCMPos[5][4],int iToolNum,double dRJPos[MaxAxisNum])
{
	int iErrorId = 0;

	double dJPosIn[MaxAxisNum] = {0};
	memcpy(dJPosIn,dRJPos,sizeof(double)*MaxAxisNum);

	double dCWMPos[4][4] = {0};
	HS_JPosToPoMPos(&dJPosIn[6],dCWMPos);

	double dTWPos[5][4] = {0};
	Matrix_Multi(4,4,4,&dCWMPos[0][0],&dBaseTCMPos[0][0],&dTWPos[0][0]);

	dTWPos[4][0] = dJPosIn[6];
	dTWPos[4][1] = dJPosIn[7];
	dTWPos[4][2] = dJPosIn[8];

	HS_MPosToJPos_LJ(dTWPos,iToolNum,-1,CP_ToolWork,dJPosIn,dRJPos);

	return iErrorId;
}
/************************************************
函数功能：轴组相对位置关系转化【相对位置求解从运动位置】
参    数：dMasterCPos----主运动位置
	     dSlaveCPos-----从运动位置
		 dRelativeCPos--相对运动位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_SyncRelativeToSlave(double dMasterCPos[6],double dRelativeMPos[5][4],double dSlaveMPos[5][4])
{
	int iErrorId = 0;

	memcpy(dSlaveMPos,dRelativeMPos,sizeof(double)*20);

	double dMaWTMPos[4][4] = {0};
	HS_CPosToMPos(dMasterCPos,dMaWTMPos);

	HS_Math::Matrix_Multi(4,4,4,&dMaWTMPos[0][0],&dRelativeMPos[0][0],&dSlaveMPos[0][0]);

	return iErrorId;
}
/************************************************
函数功能：对关节点位进行奇异位置检测，用来检测点位是否可进行空间运动
参    数：dJPos----需检测的关节位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_QYLimitCheck(double dJPos[6])
{
    int iErrorId = 0;
    if(m_tLimitPara->dQYPara_Inter < Eps||m_tLimitPara->dQYPara_Inter > 1000)
        m_tLimitPara->dQYPara_Inter = 100.0;
    if(m_tLimitPara->dQYPara_Border < Eps||m_tLimitPara->dQYPara_Border > 100)
        m_tLimitPara->dQYPara_Border = 10.0;
    //if(m_tLimitPara->dQYPara_Wrist < Eps||m_tLimitPara->dQYPara_Wrist > 50)
    //    m_tLimitPara->dQYPara_Wrist = 2.0;

	if(m_tLimitPara->dQYPara_Wrist > 50)
		m_tLimitPara->dQYPara_Wrist = 2.0;

    iErrorId = HS_QYStaticCheck(dJPos,m_tLimitPara->dQYPara_Inter,m_tLimitPara->dQYPara_Border,m_tLimitPara->dQYPara_Wrist);
    return iErrorId;
}
/************************************************
函数功能：对关节点位进行奇异位置动态检测，是否逐渐进入奇异区间
参    数：
         dCurJPos----当前关节位置
         dNexJPos----下一个关节位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_QYDynCheck(double dCurJPos[6],double dNexJPos[6])
{
    int iErrorId = 0;
	double dCurJTemp[6] = {0};
	double dNexJTemp[6] = {0};

	memcpy(dCurJTemp,dCurJPos,sizeof(double)*6);
	memcpy(dNexJTemp,dNexJPos,sizeof(double)*6);

	if(m_bCobot6Flag)
	{
		dCurJTemp[2] += 90;
		dNexJTemp[2] += 90;
	}

    if(*m_eRobotType == HSROB_SCARA)
    {
        if(fabs(dNexJTemp[1]) < m_tLimitPara->dQYPara_Border)
        {
            if((dNexJTemp[1] < -Eps&&dNexJTemp[1] > dCurJTemp[1]+Eps)||
                (dNexJTemp[1] > Eps&&dNexJTemp[1] < dCurJTemp[1]-Eps))
            {
                LOG_ALGO("Dyn QY Border: JPos2Nex = %.6lf,JPos2Cur = %.6lf",dNexJTemp[1],dCurJTemp[1]);
                return ERROR_QY_BORDER;
            }
        }
    }
    else if(*m_eRobotType == HSROB_PUMA)
    {
        //边界奇异位置
        if(m_bTypeBR||m_bCobot6Flag)
        {
            if(fabs(dNexJTemp[2] - 90) < m_tLimitPara->dQYPara_Border)
            {
                if((dNexJTemp[2] < 90 + m_QYKbPara-Eps&&dNexJTemp[2] > dCurJTemp[2]+Eps)||
                    (dNexJTemp[2] > 90 + m_QYKbPara+Eps&&dNexJTemp[2] < dCurJTemp[2]-Eps))
                {
                    LOG_ALGO("Dyn QY Border: JPos3Nex = %.6lf,JPos3Cur = %.6lf",dNexJTemp[2],dCurJTemp[2]);
                    return ERROR_QY_BORDER;
                }  
            }
            if(fabs(dNexJTemp[2] - 270) < m_tLimitPara->dQYPara_Border)
            {
                if((dNexJTemp[2] < 270 + m_QYKbPara-Eps&&dNexJTemp[2] > dCurJTemp[2]+Eps)||
                    (dNexJTemp[2] > 270 + m_QYKbPara+Eps&&dNexJTemp[2] < dCurJTemp[2]-Eps))
                {
                    LOG_ALGO("Dyn QY Border: JPos3Nex = %.6lf,JPos3Cur = %.6lf",dNexJTemp[2],dCurJTemp[2]);
                    return ERROR_QY_BORDER;
                }       
            }
			if(fabs(dNexJTemp[2] - 450) < m_tLimitPara->dQYPara_Border)
			{
				if((dNexJTemp[2] < 450 + m_QYKbPara-Eps&&dNexJTemp[2] > dCurJTemp[2]+Eps)||
					(dNexJTemp[2] > 450 + m_QYKbPara+Eps&&dNexJTemp[2] < dCurJTemp[2]-Eps))
				{
					LOG_ALGO("Dyn QY Border: JPos3Nex = %.6lf,JPos3Cur = %.6lf",dNexJTemp[2],dCurJTemp[2]);
					return ERROR_QY_BORDER;
				}       
			}
        }
        else
        {
            if(fabs(dNexJTemp[2] - (90 + m_QYKbPara)) < m_tLimitPara->dQYPara_Border)
            {
                if((dNexJTemp[2] < 90 + m_QYKbPara-Eps&&dNexJTemp[2] > dCurJTemp[2]+Eps)||
                    (dNexJTemp[2] > 90 + m_QYKbPara+Eps&&dNexJTemp[2] < dCurJTemp[2]-Eps))
                {
                    LOG_ALGO("Dyn QY Border: JPos3Nex = %.6lf,JPos3Cur = %.6lf",dNexJTemp[2],dCurJTemp[2]);
                    return ERROR_QY_BORDER;
                }
            }
        }

        if(fabs(dNexJTemp[4]) < m_tLimitPara->dQYPara_Wrist)
        {
            if((dNexJTemp[4] < -Eps&&dNexJTemp[4] > dCurJTemp[4]+Eps)||
                (dNexJTemp[4] > Eps&&dNexJTemp[4] < dCurJTemp[4]-Eps))
            {
                LOG_ALGO("Dyn QY Wrist: JPos5Nex = %.6lf,JPos5Cur = %.6lf",dNexJTemp[4],dCurJTemp[4]);
                return ERROR_QY_WRIST;
            }
        }

        double dCenterDis = BorderCenterDis(dNexJTemp);
        if(dCenterDis < m_tLimitPara->dQYPara_Inter)
        {
            double dCenterDisCur = BorderCenterDis(dCurJTemp);
            if(dCenterDis > Eps&&dCenterDis < dCenterDisCur-Eps)
            {
                LOG_ALGO("Dyn QY Inter: CenterDisNex = %.6lf,CenterDisCur = %.6lf",dCenterDis,dCenterDisCur);
                return ERROR_QY_INSIDE;
            }
        }
    }

    return iErrorId;
}
/************************************************
函数功能：对关节点位进行奇异位置检测，用来识别不同的报警可能
参    数：dJPos----需检测的关节位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_QYErrorCheck(double dJPos[6])
{
    int iErrorId = 0;

    iErrorId = HS_QYStaticCheck(dJPos,200,30,30);
    return iErrorId;
}
/************************************************
函数功能：对关节点位进行奇异位置检测【静态基础检测】
参    数：dJPos----需检测的关节位置
返 回 值：错误ID	 
*************************************************/
int HS_Kinematics::HS_QYStaticCheck(double dJPos[6],double dQYPara_Inter,double dQYPara_Border,double dQYPara_Wrist)
{
    int iErrorId = 0;

	double dCheckJPos[6] = {0};
	memcpy(dCheckJPos,dJPos,sizeof(double)*6);

    if(*m_eRobotType == HSROB_PUMA)
    {
		if(m_bCobot6Flag)
		{
			dCheckJPos[2] += 90;
		}

        //内部奇异位置
        double dCenterDis = BorderCenterDis(dCheckJPos);
        if(dCenterDis < dQYPara_Inter)
        {
            iErrorId = ERROR_QY_INSIDE;
            LOG_ALGO("Error QYInter!");
            LOG_ALGO("CenterDis = %.3lf;JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",
                dCenterDis,dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5]);
        }

        //边界奇异位置
        if(m_bTypeBR||m_bCobot6Flag)
        {
            if(fabs(dCheckJPos[2] - 90) < dQYPara_Border)
            {
                LOG_ALGO("Error QYBorder!");
                LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Border = %.3lf",
                    dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Border);
                iErrorId = ERROR_QY_BORDER;
            }
            if(fabs(dCheckJPos[2] - 270) < dQYPara_Border)
            {
                LOG_ALGO("Error QYBorder!");
                LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Border = %.3lf",
                    dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Border);
                iErrorId = ERROR_QY_BORDER;         
            }
			if(fabs(dCheckJPos[2] - 450) < dQYPara_Border)
			{
				LOG_ALGO("Error QYBorder!");
				LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Border = %.3lf",
					dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Border);
				iErrorId = ERROR_QY_BORDER;         
			}
        }
        else
        {
            if(fabs(dCheckJPos[2] - (90 + m_QYKbPara)) < dQYPara_Border)
            {
                iErrorId = ERROR_QY_BORDER;
                LOG_ALGO("Error QYBorder!");
                LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Border = %.3lf",
                    dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Border);
            }
        }

        //腕部奇异位置
        if(fabs(dCheckJPos[4]) < dQYPara_Wrist)
        {
            iErrorId = ERROR_QY_WRIST;
            LOG_ALGO("Error QYWrist!");
            LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Wrist = %.3lf",
                dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Wrist);
        }

    }
    else if(*m_eRobotType == HSROB_SCARA)
    {
        if(fabs(dCheckJPos[1]) < dQYPara_Border)
		{
            iErrorId = ERROR_QY_BORDER;
			LOG_ALGO("Error QYBorder!");
			LOG_ALGO("JPos = %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf;QYPara_Border = %.3lf",
				dCheckJPos[0],dCheckJPos[1],dCheckJPos[2],dCheckJPos[3],dCheckJPos[4],dCheckJPos[5],dQYPara_Border);
		}
    }
    return iErrorId;
}
/************************************************
函数功能： 求解腕部中心点至1轴轴线的距离，带正负
参   数： dJPos---关节坐标值
返 回 值： 距离量
*************************************************/
double HS_Kinematics::BorderCenterDis(double dJPos[6])
{
	double dNewDH[6][4];				//DH参数表 
	memcpy(dNewDH,m_dDHPara,sizeof(dNewDH));

	if(m_bCobot6Flag)
	{
		dNewDH[0][0] = 0;               dNewDH[0][1] = m_dDHPara[0][1];	    dNewDH[0][2] = -90;
		dNewDH[1][0] = m_dDHPara[1][0]; dNewDH[1][1] = m_dDHPara[2][1];		dNewDH[1][2] = 0;
		dNewDH[2][0] = 0;               dNewDH[2][1] = 0;		            dNewDH[2][2] = 90;
		dNewDH[3][0] = 0;	            dNewDH[3][1] = (m_dDHPara[2][0]-m_dDHPara[4][1]);	    dNewDH[3][2] = -90;
		dNewDH[4][0] = 0;	            dNewDH[4][1] = 0;	                dNewDH[4][2] = 90;
		dNewDH[5][0] = 0;	            dNewDH[5][1] = m_dDHPara[5][1];	    dNewDH[5][2] = 0;
	}

    //相应的DH参数的提取
    double dA0 = dNewDH[0][0];
    double dA1 = dNewDH[1][0];
    double dA2 = dNewDH[2][0];
    double dD1 = dNewDH[1][1];
    double dD3 = dNewDH[3][1];

    double dCosA2 = cos(dJPos[1]*deg2rad);
    double dSin23 = sin((dJPos[1]+dJPos[2])*deg2rad);
    double dCos23 = cos((dJPos[1]+dJPos[2])*deg2rad);

    //修改:dA0为分解线，实际处于A0位置处就是内部奇异分界点
    //修改:dA0的偏置需要添加，原取消应是错误
    double dWristPx = dA0 + dD3*dSin23 + dA1*dCosA2 + dA2*dCos23;
    //修改：Y方向的偏置取消，BR系列机器人Y方向的偏置正常
    double dWristPy = 0;//dD1;

    return sqrt(dWristPx*dWristPx + dWristPy*dWristPy);
}

/************************************************
函数功能：腕部过奇异功能开关
参    数：bWristQYHandleFlag---腕部奇异处理标识
返 回 值：腕部过奇异标识
*************************************************/
bool HS_Kinematics::HS_SetWristQYHandle(bool bWristQYHandleFlag)
{
    if(bWristQYHandleFlag&&*m_eRobotType == HSROB_PUMA)
        m_bWristQYHandleFlag = true;
    else
        m_bWristQYHandleFlag = false;

    return m_bWristQYHandleFlag;
}

/************************************************
函数功能：判断点位是否重复
参    数：
      tPosA---点位值A
      tPosB---点位置B
返 回 值：
	  0--非重复点位
      1--重复点位  
*************************************************/
bool HS_Kinematics::HS_RepeatPosCheck(PosInfo tPosA,PosInfo tPosB)
{
	double DIS = 1e-3;

	if(tPosA.hs_coordinate.iCoordinate != JOINT_COORD_SYSTEM)
	{
		int iToolNum = 0;
		int iWorkNum = 0;
		if(tPosA.hs_coordinate.iCoordinate != BASE_COORD_SYSTEM) 
		{
			iToolNum = tPosA.hs_coordinate.iToolNum;
			iWorkNum = tPosA.hs_coordinate.iWorkNum;
		}
		double dMPosA[4][4] = {0};
		double dMPosB[4][4] = {0};
		HS_CPosToMPos(tPosA.dPos,dMPosA);

        unsigned char eStateB = 0;
		if(tPosB.hs_coordinate.iCoordinate == JOINT_COORD_SYSTEM)
		{
			double dFBMPos[4][4] = {0};			
			int iErrorId = HS_JPosToMPos(tPosB.dPos,dFBMPos);
			HS_FBMPosToTWMPos(dFBMPos,iToolNum,iWorkNum,dMPosB);	
            eStateB = HS_JPosToAState(tPosB.dPos);
		}
		else
		{
			HS_CPosToMPos(tPosB.dPos,dMPosB);
            eStateB = tPosB.iPose;
		}

        //形态位判断
        if((tPosA.iPose&0x07) != (eStateB&0x07))
            return false;

		//判断齐次矩阵
		double dDis = HS_MatrixDis(dMPosA,dMPosB);
		if(dDis > DIS)
			return false;	

		//附加轴判断
		for(int i = 6;i < MaxAxisNum;i++)	
		{
			if(fabs(tPosA.dPos[i] - tPosB.dPos[i]) > DIS)
				return false;
		}
	}
	else if(tPosB.hs_coordinate.iCoordinate != JOINT_COORD_SYSTEM)
	{
		int iToolNum = 0;
		int iWorkNum = 0;
		if(tPosB.hs_coordinate.iCoordinate != BASE_COORD_SYSTEM) 
		{
			iToolNum = tPosB.hs_coordinate.iToolNum;
			iWorkNum = tPosB.hs_coordinate.iWorkNum;
		}
		double dMPosA[4][4] = {0};
		double dMPosB[4][4] = {0};
		HS_CPosToMPos(tPosB.dPos,dMPosB);

        unsigned char eStateA = 0;
		if(tPosA.hs_coordinate.iCoordinate == JOINT_COORD_SYSTEM)
		{
			double dFBMPos[4][4] = {0};			
			int iErrorId = HS_JPosToMPos(tPosA.dPos,dFBMPos);
			HS_FBMPosToTWMPos(dFBMPos,iToolNum,iWorkNum,dMPosA);	
            eStateA = HS_JPosToAState(tPosA.dPos);
		}
		else
		{
			HS_CPosToMPos(tPosA.dPos,dMPosA);
            eStateA = tPosA.iPose;
		}

        //形态位判断
        if((tPosB.iPose&0x07) != (eStateA&0x07))
            return false;

		//判断齐次矩阵
		double dDis = HS_MatrixDis(dMPosA,dMPosB);
		if(dDis > DIS)
			return false;	

		//附加轴判断
		for(int i = 6;i < MaxAxisNum;i++)	
		{
			if(fabs(tPosA.dPos[i] - tPosB.dPos[i]) > DIS)
				return false;
		}
	}
	else
	{
		//两个关节
		for(int i = 0;i < MaxAxisNum;i++)	
		{
			if(fabs(tPosA.dPos[i] - tPosB.dPos[i]) > DIS)
				return false;
		}
	}

	return true;	
}