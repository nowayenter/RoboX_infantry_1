#ifndef UMPIRE_H
#define UMPIRE_H
#include "sys.h"
#include "usart.h"


//���ݰ���ʽ
typedef __packed struct
{
	uint8_t SOF;//֡��ʼ�ֽڣ��̶�ֵ0xA5
	uint8_t Seq;//�����
	uint8_t CRC8;//��ͷcrc8����
	uint16_t DataLength;//���ݶ�data����
	uint16_t CmdID;//������ID
	uint16_t FrameTail;//����crc16����
	
}Frame_TypeDef;


//������״̬
typedef __packed struct
{
	uint16_t stageRemianTime;//��ǰ�׶�ʣ��ʱ��
	uint8_t gameProgress;//�����׶�,0δ��ʼ����,1׼���׶�,2�Լ�׶�,3 5s����ʱ,4��ս��,����������
	uint8_t robotLevel;//�ȼ�
	uint16_t remainHP;//��ǰѪ��
	uint16_t maxHP;//��Ѫ��
	
}RoboState_TypeDef;//CmdID:0x0001


//�˺�����
typedef __packed struct
{
	uint8_t armorType;//�ܵ��˺�ʱװ�׵�id��0~5,0ǰ,1��,2��,3��
	uint8_t hurtType;//Ѫ���仯���ͣ�0�ܵ�������1ģ�����
	
}RoboHurt_TypeDef;//CmdID:0x0002


//ʵʱ�����Ϣ
typedef __packed struct
{
	uint8_t bulletType;//�������ͣ�1С����2��
	uint8_t bulletFreq;//��Ƶ����ÿ��
	float bulletSpeed;//���٣���ÿ��
	
}ShootData_TypeDef;//CmdID:0x0003


//ʵʱ������������
typedef __packed struct
{
	float chassisVolt;//���������ѹ��V
	float chassisCurrent;//�������������A
	float chassisPower;//����������ʣ�W
	float chassisPowerBuffer;//���̹��ʻ��壬W
	uint16_t shooterHeat0;//Сǹ������
	uint16_t shooterHeat1;//��ǹ������
	
}PowerHeatData_TypeDef;//CmdID:0x0004


//���ؽ�������
typedef __packed struct
{
	uint8_t cardType;//������
									/* 0�������ӳɿ���1�������ӳɿ�
										*2���췽��Ѫ����3��������Ѫ��
										*4���췽���ƿ���5���������ƿ�
										*6���췽��ȴ����7��������ȴ��
										*8���ﱤ����9������
										*10����Դ���� */
	uint8_t cardldx;//�������ţ������ڲ�ͬ����
	
}RfidDetect_TypeDef;//CmdID:0x0005


//����ʤ������
typedef __packed struct
{
	uint8_t winner;//���������0:ƽ�֣�1�췽ʤ��2����ʤ
	
}GameResult_TypeDef;//CmdID:0x0006


//Buff����
/*bit0:��Ѫ���Ѫ��bit1:���̻����˻�Ѫ��bit2:���ƿ���Ѫ��bit3:��Դ��������
 *bit4:�������������أ�bit5:�з����������أ�bit6:����С�������أ�bit7:�з�С�������أ�
 *bit8:������ȴ��bit9:�ﱤ������bit10:�ٷְٷ�����bit11:���ڱ����ط�����
 *bit12:���ڱ����ط���*/
typedef __packed struct
{
	uint16_t buffMusk;//buff���ͣ�1��Ч
	
}BuffMusk_TypeDef;//CmdID:0x0007


//������λ�ó�����Ϣ
typedef __packed struct
{
	float x;//��λ��
	float y;
	float z;
	float yaw;//ǹ�ڳ���Ƕ�ֵ����λ��
	
}GameRobotPos_TypeDef;//CmdID:0x0008


/////////////////////////////////////////////////////////////////

//����������Ϣ...��bug...
typedef __packed struct
{
	RoboState_TypeDef State;					//������״̬
	RoboHurt_TypeDef Hurt;						//�˺�����
	ShootData_TypeDef Shoot;					//ʵʱ�����Ϣ
	PowerHeatData_TypeDef PowerHeat;	//ʵʱ������������
	RfidDetect_TypeDef Rfid;					//���ؽ�������
	GameResult_TypeDef GameResult;		//����ʤ������
	BuffMusk_TypeDef Buff;						//Buff����
	GameRobotPos_TypeDef RobotPos;		//������λ�ó�����Ϣ
	
}UmpireData_TypeDef;


extern RoboState_TypeDef Umpire_State;					//������״̬
extern RoboHurt_TypeDef Umpire_Hurt;						//�˺�����
extern ShootData_TypeDef Umpire_Shoot;					//ʵʱ�����Ϣ
extern PowerHeatData_TypeDef Umpire_PowerHeat;	//ʵʱ������������
extern RfidDetect_TypeDef Umpire_Rfid;					//���ؽ�������
extern GameResult_TypeDef Umpire_GameResult;		//����ʤ������
extern BuffMusk_TypeDef Umpire_Buff;						//Buff����
extern GameRobotPos_TypeDef Umpire_RobotPos;		//������λ�ó�����Ϣ


void Comm_umpire_Init(void);//��ʼ��
void Get_UmpireData(uint16_t Length);//��ȡ����ϵͳ����
void DealDataPack(uint16_t CmdID, uint16_t DataLength);//��������



#endif

