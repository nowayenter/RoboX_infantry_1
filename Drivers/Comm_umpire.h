#ifndef UMPIRE_H
#define UMPIRE_H
#include "sys.h"
#include "usart.h"


//数据包格式
typedef __packed struct
{
	uint8_t SOF;//帧起始字节，固定值0xA5
	uint8_t Seq;//包序号
	uint8_t CRC8;//包头crc8检验
	uint16_t DataLength;//数据段data长度
	uint16_t CmdID;//命令码ID
	uint16_t FrameTail;//整包crc16检验
	
}Frame_TypeDef;


//机器人状态
typedef __packed struct
{
	uint16_t stageRemianTime;//当前阶段剩余时间
	uint8_t gameProgress;//比赛阶段,0未开始比赛,1准备阶段,2自检阶段,3 5s倒计时,4对战中,比赛结算中
	uint8_t robotLevel;//等级
	uint16_t remainHP;//当前血量
	uint16_t maxHP;//满血量
	
}RoboState_TypeDef;//CmdID:0x0001


//伤害数据
typedef __packed struct
{
	uint8_t armorType;//受到伤害时装甲的id，0~5,0前,1左,2后,3右
	uint8_t hurtType;//血量变化类型，0受到攻击，1模块掉线
	
}RoboHurt_TypeDef;//CmdID:0x0002


//实时射击信息
typedef __packed struct
{
	uint8_t bulletType;//弹丸类型，1小蛋，2大蛋
	uint8_t bulletFreq;//射频，发每秒
	float bulletSpeed;//射速，米每秒
	
}ShootData_TypeDef;//CmdID:0x0003


//实时功率热量数据
typedef __packed struct
{
	float chassisVolt;//底盘输出电压，V
	float chassisCurrent;//底盘输出电流，A
	float chassisPower;//底盘输出功率，W
	float chassisPowerBuffer;//底盘功率缓冲，W
	uint16_t shooterHeat0;//小枪口热量
	uint16_t shooterHeat1;//大枪口热量
	
}PowerHeatData_TypeDef;//CmdID:0x0004


//场地交互数据
typedef __packed struct
{
	uint8_t cardType;//卡类型
									/* 0：攻击加成卡，1：防御加成卡
										*2：红方加血卡，3：蓝方加血卡
										*4：红方治疗卡，5：蓝方治疗卡
										*6：红方冷却卡，7：蓝方冷却卡
										*8：碉堡卡，9：保留
										*10：资源岛卡 */
	uint8_t cardldx;//卡索引号，可用于不同区域
	
}RfidDetect_TypeDef;//CmdID:0x0005


//比赛胜负数据
typedef __packed struct
{
	uint8_t winner;//比赛结果，0:平局，1红方胜，2蓝方胜
	
}GameResult_TypeDef;//CmdID:0x0006


//Buff数据
/*bit0:补血点回血，bit1:工程机器人回血，bit2:治疗卡回血，bit3:资源岛防御，
 *bit4:己方大能量机关，bit5:敌方大能量机关，bit6:己方小能量机关，bit7:敌方小能量机关，
 *bit8:加速冷却，bit9:碉堡防御，bit10:百分百防御，bit11:无哨兵基地防御，
 *bit12:有哨兵基地防御*/
typedef __packed struct
{
	uint16_t buffMusk;//buff类型，1有效
	
}BuffMusk_TypeDef;//CmdID:0x0007


//机器人位置朝向信息
typedef __packed struct
{
	float x;//单位米
	float y;
	float z;
	float yaw;//枪口朝向角度值，单位度
	
}GameRobotPos_TypeDef;//CmdID:0x0008


/////////////////////////////////////////////////////////////////

//整合所有信息...有bug...
typedef __packed struct
{
	RoboState_TypeDef State;					//机器人状态
	RoboHurt_TypeDef Hurt;						//伤害数据
	ShootData_TypeDef Shoot;					//实时射击信息
	PowerHeatData_TypeDef PowerHeat;	//实时功率热量数据
	RfidDetect_TypeDef Rfid;					//场地交互数据
	GameResult_TypeDef GameResult;		//比赛胜负数据
	BuffMusk_TypeDef Buff;						//Buff数据
	GameRobotPos_TypeDef RobotPos;		//机器人位置朝向信息
	
}UmpireData_TypeDef;


extern RoboState_TypeDef Umpire_State;					//机器人状态
extern RoboHurt_TypeDef Umpire_Hurt;						//伤害数据
extern ShootData_TypeDef Umpire_Shoot;					//实时射击信息
extern PowerHeatData_TypeDef Umpire_PowerHeat;	//实时功率热量数据
extern RfidDetect_TypeDef Umpire_Rfid;					//场地交互数据
extern GameResult_TypeDef Umpire_GameResult;		//比赛胜负数据
extern BuffMusk_TypeDef Umpire_Buff;						//Buff数据
extern GameRobotPos_TypeDef Umpire_RobotPos;		//机器人位置朝向信息


void Comm_umpire_Init(void);//初始化
void Get_UmpireData(uint16_t Length);//获取裁判系统数据
void DealDataPack(uint16_t CmdID, uint16_t DataLength);//解析数据



#endif

