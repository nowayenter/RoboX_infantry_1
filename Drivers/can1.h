#ifndef __CAN1_H
#define __CAN1_H
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "Configuration.h"

#define Hz_Flag (1000/PID_Hz)		//��������

typedef struct Motor_data    //���������ݽṹ��
{	
	int16_t ActualSpeed;		//�����ٶ�ֵ
	int16_t last_speed;			//�ϴ��ٶ�ֵ
	int16_t NowAngle;				//��ǰ��е��
	int16_t OldAngle;				//�ϴλ�е��
	int16_t D_Angle;				//һ���ڻ�е�ǲ�ֵ
	
	int16_t Intensity;			//����ת�ص�������̨�������
}M_Data;

extern M_Data motor_data[7];		//�������



//6�������λ��
enum {M0,M1,M2,M3,YAW,PITCH};

//���can��ʼ��
void CAN1_Configuration(void);

//������ݷ���
void CAN1_SendCommand_chassis(signed long ESC_201,signed long ESC_202,
							signed long ESC_203,signed long ESC_204);
void CAN1_Send_Msg_gimbal(int16_t control_205,int16_t control_206, int16_t control_2006);

int16_t kalman(int16_t x,uint8_t i);			//��е�ǿ�����
void Angle_deal(int i);				//��е��Խ�紦��


#endif
