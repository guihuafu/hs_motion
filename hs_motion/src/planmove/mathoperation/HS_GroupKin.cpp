
#include "HS_GroupKin.h"

HS_GroupKin::HS_GroupKin()
{
	for(int i = 0;i < MAXGROUPNUM;i++)
	{
		m_HS_Kinematics[i] = new HS_Kinematics(i);
	}
	m_iGroupNum = 0;
	memset(m_iGroupToolNum,0,sizeof(m_iGroupToolNum));
}

HS_GroupKin::~HS_GroupKin()
{
	for(int i = 0;i < MAXGROUPNUM;i++)
	{
		delete m_HS_Kinematics[i];
		m_HS_Kinematics[i] = NULL;
	}
}

/************************************************
函数功能：获取对应轴组编号的算法对象
参    数：iGroupNum----轴组编号
返 回 值：算法计算对象
*************************************************/	
HS_Kinematics* HS_GroupKin::GetKinematicsByNum(int iGroupNum)
{
	if(iGroupNum < 0||iGroupNum >= MAXCOORDNUM)
		return m_HS_Kinematics[0];

	return m_HS_Kinematics[iGroupNum];
}

/************************************************
函数功能：设置轴组编号
参    数：iGroupNum----轴组编号
返 回 值：是否有关联关系，需要协同计算
*************************************************/	
bool HS_GroupKin::SetGroupNum(int iGroupNum,HS_GroupRel tHS_GroupRel)
{
	bool bSyncMoveFlag = false;

	LOG_ALGO("---------------Group %d!---------------",iGroupNum);
	
	if(iGroupNum < 0||iGroupNum >= MAXCOORDNUM)
		m_iGroupNum = 0;
	else 
		m_iGroupNum = iGroupNum;

	m_HS_Kinematics[m_iGroupNum]->PrintKeyInfo();

	if(tHS_GroupRel.eGroupRelType[m_iGroupNum] == GRT_Master)
	{
		int iSlaveCnt = 0;
		//判断从机
		for(int i = 0;i < MAXGROUPNUM;i++)
		{
			if(i != m_iGroupNum)
			{
				if(tHS_GroupRel.eGroupRelType[i] == GRT_Slave)
				{
					LOG_ALGO("---------------Slave Group %d!---------------",i);
					m_HS_Kinematics[i]->PrintKeyInfo();
					iSlaveCnt++;
					m_iSlaveNum = i;
				}
			}			
		}

		if(iSlaveCnt > 0)
			bSyncMoveFlag = true;
	}

	return bSyncMoveFlag;
}

/************************************************
函数功能：协同同步运动：定点同步，初始位置设定
参    数：
		 tHS_GroupJPos----实际关节位置
		 iToolNum---------工具号【从机是机器人组使用】
返 回 值：错误码
*************************************************/	
int HS_GroupKin::HandSyncInit(HS_GroupJPos &tHS_GroupJPos,int iToolNum[MAXGROUPNUM])
{
	//计算从机相对于主机的位置作为定点位置 M/S
	
	double dMasterJPos[MaxAxisNum] = {0};
	memcpy(dMasterJPos,tHS_GroupJPos.dJPos[m_iGroupNum],sizeof(double)*MaxAxisNum);

	// 主机W/T[M]
	double dWTMPos_Master[5][4] = {0};
	m_HS_Kinematics[m_iGroupNum]->HS_JPosToMPos(dMasterJPos,iToolNum[m_iGroupNum],-1,dWTMPos_Master);


	double dSlaveJPos[MaxAxisNum] = {0};
	memcpy(dSlaveJPos,tHS_GroupJPos.dJPos[m_iSlaveNum],sizeof(double)*MaxAxisNum);

	// 从机W/T[S]
	double dWTMPos_Slave[5][4] = {0};
	m_HS_Kinematics[m_iSlaveNum]->HS_JPosToMPos(dSlaveJPos,iToolNum[m_iSlaveNum],-1,dWTMPos_Slave);

	double dMWMPos[4][4] = {0};

	// M/W * W/S = M/S
	HS_Math::Matrix_Inverse(4,&dWTMPos_Master[0][0],&dMWMPos[0][0]);

	HS_Math::Matrix_Multi(4,4,4,&dMWMPos[0][0],&dWTMPos_Slave[0][0],&m_dHandMSMPosInit[0][0]);

	memcpy(m_iGroupToolNum,iToolNum,sizeof(m_iGroupToolNum));
	
	return 0;
}

/************************************************
函数功能：协同同步运动：根据主运动的位置计算从动的位置【手动定点协同】
参    数：
		tHS_GroupJPos----规划得到的关节位置
返 回 值：错误码
*************************************************/	
int HS_GroupKin::HandSyncMove(HS_GroupJPos &tHS_GroupJPos)
{
	double dMasterJPos[MaxAxisNum] = {0};
	memcpy(dMasterJPos,tHS_GroupJPos.dJPos[m_iGroupNum],sizeof(double)*MaxAxisNum);
		
	double dSlaveJPos[MaxAxisNum] = {0};
	memcpy(dSlaveJPos,tHS_GroupJPos.dJPos[m_iSlaveNum],sizeof(double)*MaxAxisNum);

	// 已知 M/S 固定，W/M的位置，求解得到 W/S

	// 主机 W/T [M]
	double dWTMPos_Master[5][4] = {0};
	m_HS_Kinematics[m_iGroupNum]->HS_JPosToMPos(dMasterJPos,m_iGroupToolNum[m_iGroupNum],-1,dWTMPos_Master);

	double dWTMPos_Slave[5][4] = {0};
	HS_Math::Matrix_Multi(4,4,4,&dWTMPos_Master[0][0],&m_dHandMSMPosInit[0][0],&dWTMPos_Slave[0][0]);

	m_HS_Kinematics[m_iSlaveNum]->HS_MPosToJPos_LJ(dWTMPos_Slave,m_iGroupToolNum[m_iSlaveNum],-1,CP_ToolWork,dSlaveJPos,dSlaveJPos);

	memcpy(tHS_GroupJPos.dJPos[m_iSlaveNum],dSlaveJPos,sizeof(double)*MaxAxisNum);

	return 0;
}