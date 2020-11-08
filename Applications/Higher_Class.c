/*************************************************************************
 *�߼�����ʵ��
 *���ƹ���
 *�Ӿ��Զ���׼ģʽ
 *�ӽ��л�
 *��е�۶���ʵ��
 *
 *************************************************************************/
#include "Higher_Class.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "IMU.h"
#include "MPU6500.h"
#include "Configuration.h"
#include "led.h"
#include "pwm.h"
#include "buzzer.h"
#include "RemoteControl.h"
#include "can1.h"
#include "can2.h"
#include "Ctrl_chassis.h"
#include <math.h>
#include <stdlib.h>


float Vx_Lpf=0,Vy_Lpf=0,Wz_Lpf=0;	//���Ƽ��ٶȺ���˶���



//ƽ���ά���������ת
//������Ҫ����ת��x,y,��ת�Ƕ�
//�Ƕȵ�λΪ��
void Matrix_Translate(float *X,float *Y,float angle)
{
	angle = angle/180*PI;
	float cosX = (*X)*cosf(angle);
	float sinY = (*Y)*sinf(angle);
	float sinX = (*X)*sinf(angle);
	float cosY = (*Y)*cosf(angle);
	
	*X = cosX - sinY;
	*Y = sinX + cosY; 
}

//���Ƽ��ٶ�
//���ƺ���ֵ������Vx_Lpf,Vy_Lpf,Wz_Lpf��
#define LOW_SPEED	800	//��������
static float Acc_x = X_ACC_MAX/PID_Hz,Acc_y = Y_ACC_MAX/PID_Hz,Acc_z = Z_ACC_MAX/PID_Hz;//�����ٶ�
void Astrict_Acc(float Vx,float Vy,float Wz)
{
	if((Vx_Lpf>LOW_SPEED)||(Vx_Lpf<-LOW_SPEED))
	{
		if((Vx-Vx_Lpf)>Acc_x)	Vx_Lpf += Acc_x;
		else if((Vx-Vx_Lpf)<-Acc_x)	Vx_Lpf -= Acc_x;
		else Vx_Lpf = Vx;
	}
	else
	{
		//�����ٶȸı���ٶȴ�С����Ϊ������ɲ�������״�
		float k = 0.6 + 0.4*fabs(Vx_Lpf)/LOW_SPEED;
		k *= Acc_x;
		if((Vx-Vx_Lpf)>k)	Vx_Lpf += k;
		else if((Vx-Vx_Lpf)<-k)	Vx_Lpf -= k;
		else Vx_Lpf = Vx;
	}
	if((Vy_Lpf>LOW_SPEED)||(Vy_Lpf<-LOW_SPEED))
	{
		if((Vy-Vy_Lpf)>Acc_y)	Vy_Lpf += Acc_y;
		else if((Vy-Vy_Lpf)<-Acc_y)	Vy_Lpf -= Acc_y;
		else Vy_Lpf = Vy;
	}
	else
	{
		float k = 0.6 + 0.4*fabs(Vy_Lpf)/LOW_SPEED;
		k *= Acc_y;
		if((Vy-Vy_Lpf)>k)	Vy_Lpf += k;
		else if((Vy-Vy_Lpf)<-k)	Vy_Lpf -= k;
		else Vy_Lpf = Vy;
	}
	if((Wz-Wz_Lpf)>Acc_z)	Wz_Lpf += Acc_z;
	else if((Wz-Wz_Lpf)<-Acc_z)	Wz_Lpf -= Acc_z;
	else Wz_Lpf = Wz;
}




/***********************************************�Ӿ�********************************************************/
//�Ӿ�ģʽ�л�
//mode��ѡVision_Discern_RedArmor��ʶ���ɫװ��
//				Vision_Discern_BlueArmor��ʶ����ɫװ��

void Vision_Mode(u8 mode)
{
	switch (mode)
	{
		case Vision_Discern_RedArmor:
		{
			if(Vision_Data.mode != Vision_Discern_RedArmor)
				CAN2_Send(Vision_Discern_RedArmor);
			break;
		}
		case Vision_Discern_BlueArmor:
		{
			if(Vision_Data.mode != Vision_Discern_BlueArmor)
				CAN2_Send(Vision_Discern_BlueArmor);
			break;
		}
		default:
				break;
	}
}


///************************************************��������********************************************************/

//void left_chassis(u8 k)
//{
//  if(k!=1)
//	{
//		GPIO_SetBits(GPIOC,GPIO_Pin_2);
//		
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
//		
//	}
//}

///************************************************��������********************************************************/

//void right_chassis(u8 k)
//{
//  if(k!=1)
//	{
//		GPIO_SetBits(GPIOC,GPIO_Pin_3);
//		
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
//		
//	}
//}
///************************************************��������********************************************************/

//void chassis_ctrl(u8 k)
//{
//  if(k!=1)
//	{
//		GPIO_SetBits(GPIOC,GPIO_Pin_2);
//		GPIO_SetBits(GPIOC,GPIO_Pin_3);
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
//		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
//	}
//}

///************************************************���տ���********************************************************/

//void bullet_ctrl(u8 k)
//{
//	if(k!=1)
//	{
//	GPIO_SetBits(GPIOC,GPIO_Pin_0);		
//	}
//	else
//	{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_0);		
//	}

//}

///************************************************ȡ��ģ�����********************************************************/

//void arm_ctrl(void)
//{
//		arm_open(); //��е����չ
//		GPIO_SetBits(GPIOC,GPIO_Pin_1); //�ر��˵�
//		arm_catch();//��е�ۼ�ȡ
//		delay_ms(1000);
//		arm_release(); //��е���ջ�
//		delay_ms(5000);
//		GPIO_ResetBits(GPIOC,GPIO_Pin_1);		//�˵�
//		
//}

//void arm_open(void)
//{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
//}

//void arm_close(void)
//{
//		GPIO_SetBits(GPIOC,GPIO_Pin_5);
//}

//void arm_catch(void)
//{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_4);
//}

//void arm_release(void)
//{
//		GPIO_SetBits(GPIOC,GPIO_Pin_4);
//}

