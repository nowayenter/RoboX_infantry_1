#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "Configuration.h"
#include "can1.h"
#include "can2.h"
#include "led.h"
#include "RemoteControl.h"
#include "timer.h"
#include "spi.h"
#include "IMU.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "pwm.h"
#include "User_Api.h"
#include "PID.h"
#include "Higher_Class.h"
#include "Shoot.h"
#include <math.h>
#include <stdlib.h>
#include "digital.h"
#include "fric.h"
#include "Comm_umpire.h"
#include "power_ctrl.h"

/***********************
 *���������԰� edition1  5/25/19
 *��λ��/ң���������л���ң�����Ҽ���������ʱ���Կ��ƣ�����ң��������
 ***********************/


u16 loopcnt=0;
int fricOn = 0;

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);	//SysTick����ϵͳtick��ʱ������ʼ�����жϣ�1ms
	cycleCounterInit();		//��ʼ��ϵͳʱ�亯��
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	usart_init(115200);			//���ڳ�ʼ��������Ϊ115200
	
	Comm_umpire_Init();
	
	TIM3_PWM_Init();				//imu�¶Ȳ������ӿڳ�ʼ��
	//TIM12_PWM_Init();				//��������ʼ��
	TIM1_PWM_Init();				//Ħ����pwm
	Laser_Init();						//�����ʼ��
	LED_Init();
	
	digital_init();					//��ʼ��GPIO��
	CAN1_Configuration();		//��ʼ��can����
	RC_Init();							//��ʼ��ң����
	
	SPI5_Init();						//��ʼ��SPI1��
	MPU6500_Init();					//��ʼ��MPU6500
	IMU_Init();
	
	TIM4_Int_Init(9,8999);	//10KhzƵ�ʼ�����10Ϊ1ms��ϵͳʱ����ʱ��
	Total_PID_Init();    
	
	CAN2_Configuration();
	Gimbal_Ctrl_init();		//��ʼ����̨���ֵ�����轫���������䵯���ֶ�����
	
	//24������ƿ� ��ʼ��
  power_ctrl_configuration();
	
	//power_ctrl_on(2);//���2��24V�ɿص�Դ

	//fric_PWM_configuration();//Ħ����pwm��ʼ��
	
	delay_ms(500);					//�ȴ������ʼ��
	
	while(1)
	{
		
		if(Flag500Hz)
		{
			Flag500Hz = 0;
			
			IMUSO3Thread();		//imu��̬����
			IMU_Temp_Ctrl();	//imu�¶ȱջ�
			
			Get_Ctrl_Data();

			if(loopcnt>1500)
			{
				Get_Ctrl_Data();
				Shoot_Ctrl();
			}else loopcnt++;
			
			if(RC_Ctl.rc.s2 == RC_SW_MID){//ʧ����ֹ
				Stop_Chassis_Task();
				CAN1_Send_Msg_gimbal(0, 0, 0);
				laser_off();
				fric_off();
			}
			
			else if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_DOWN)//ң���� ���� ���� ���̿��� ���̸�����̨
			{
				laser_off();
//				shoot_control_loop(Fric_OFF);
				Chassis_Task();
				//Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				fric_off();
			}
			else if(RC_Ctl.rc.s1 == RC_SW_MID && RC_Ctl.rc.s2 == RC_SW_DOWN){//ң���� ���� ���� Ħ���ֿ��� �����ֿ��� 
				//Chassis_Task();
				//shoot_control_loop(Fric_DOWN);
				//Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				//Twist(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle, 50);
				//Gimbal_Ctrl(Control_data.Pitch_angle, Control_data.Yaw_angle, 0);
			}
			else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN){//ң���� ���� ����
				//PWM_Write(PWM1_CH1, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//Ħ����	
				//Chassis_Task();
				//PWM_Write(PWnM1_CH4, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//Ħ����
				shoot_control_loop(Fric_MID);
				//Twist(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,200);
				Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,-200);
			}
			//else if(RC_Ctl.rc.s2 == RC_SW_UP){
//				shoot_control_loop(targetSpeed);
//				
//				if(RC_Ctl.mouse.press_l){//Ħ���ָ��ٿ��������̵��ת��
//					if(RC_Ctl.key.Shift){
//						targetSpeed = Fric_UP;
//						shootSpeed = 150;
//					}
//					else if(RC_Ctl.key.Ctrl){//�������
//						targetSpeed = Fric_MID;
//						shootSpeed = 200;
//					}
//					else{//��ͨ����
//						targetSpeed = Fric_MID;
//						shootSpeed = 300;
//					}
				//}
//				else if(RC_Ctl.mouse.press_r){//Ħ���ָ��ٿ��������̵��ת��
//					shootSpeed = -300;
//				}
//				else{
//					shootSpeed = 0;
//					targetSpeed = Fric_RC;
//				}
//				
//				if(RC_Ctl.key.R){
//					Twist(-Control_data.Vy,Control_data.Vx,Control_data.Wz,Control_data.Pitch_angle,shootSpeed);
//				}
//				else{
//					Chassis_AutoFollow(-Control_data.Vy,Control_data.Vx,Control_data.Wz,Control_data.Pitch_angle,shootSpeed);		
//				}	
			}
			
			
			//Astrict_Acc(Control_data.Vx,Control_data.Vy,Control_data.Wz);//���ٶ�����
			//Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//���̿��ƽӿ�
			//Gimbal_Ctrl2(3,0);
			
			/*
			if(RC_Ctl.rc.s1 == RC_SW_DOWN) Shoot_Flag =1;
			else Shoot_Flag = 0; */
		
		if(Flag200Hz_Thread1)
		{
			Flag200Hz_Thread1 = 0;	
		}
		
		if(Flag200Hz_Thread2)
		{
			Flag200Hz_Thread2 = 0;	
		}
		
		if(Flag100Hz_Thread1)    
		{
			Flag100Hz_Thread1 = 0;	
		}
		
		if(Flag100Hz_Thread2)
		{
			Flag100Hz_Thread2 = 0;	
		}
		
		if(Flag50Hz)
		{
			Flag50Hz = 0;
			
			LED0 = !LED0;
//			PWM_Write(PWM1_CH1,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//Ħ����	
//      PWM_Write(PWM1_CH4,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//Ħ����
		}
	}
}

