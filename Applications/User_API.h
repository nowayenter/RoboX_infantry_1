#ifndef __USERAPI_H
#define __USERAPI_H
#include "sys.h"

typedef struct
{
	float Vx;			//ǰ���ٶ�
	float Vy;			//�����ٶ�
	float Wz;			//��ת�ٶ�
	
	float Pitch_angle;	//pitch��Ƕ�
	float Yaw_angle;		//yaw
	
}ControlDATA_TypeDef;
extern ControlDATA_TypeDef Control_data;//��������

//�������
typedef struct
{
	u16 Frequency;	//��Ƶ
	float Speed;		//����
	
}ShootProject_TypeDef;


extern uint8_t status[];

extern u8 TrashyTime_Flag;	//����ʱ�䣬����ʱ������ʱ��
extern u8 Vision_Flag;			//�Ӿ�����
extern u8 ShootBuff_Flag;		//���
extern u8 Shoot_Flag;				//���
extern u8 Twist_Flag;				//Ť��
extern u8 Shoot_Motor;			//Ħ����

extern void Chassis_Task(void);	//���̿���
extern void Stop_Chassis_Task(void);//��ֹ���̿���
extern void Gimbal_Task(void);		//��̨����

extern void User_Api(void);
extern void Mode_Monitor(void);
extern void Get_Ctrl_Data(void);
extern void MultiFuntion(void);
extern void Shoot_Ctrl(void);
#endif



