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
 *步兵车测试版 edition1  5/25/19
 *上位机/遥控器控制切换：遥控器右键拨至最上时电脑控制，往下遥控器控制
 ***********************/


u16 loopcnt=0;
int fricOn = 0;

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);	//SysTick开启系统tick定时器并初始化其中断，1ms
	cycleCounterInit();		//初始化系统时间函数
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	usart_init(115200);			//串口初始化波特率为115200
	
	Comm_umpire_Init();
	
	TIM3_PWM_Init();				//imu温度补偿器接口初始化
	//TIM12_PWM_Init();				//蜂鸣器初始化
	TIM1_PWM_Init();				//摩擦轮pwm
	Laser_Init();						//激光初始化
	LED_Init();
	
	digital_init();					//初始化GPIO口
	CAN1_Configuration();		//初始化can总线
	RC_Init();							//初始化遥控器
	
	SPI5_Init();						//初始化SPI1口
	MPU6500_Init();					//初始化MPU6500
	IMU_Init();
	
	TIM4_Int_Init(9,8999);	//10Khz频率计数到10为1ms，系统时基定时器
	Total_PID_Init();    
	
	CAN2_Configuration();
	Gimbal_Ctrl_init();		//初始化云台波轮电机，需将波轮盘与落弹孔手动对齐
	
	//24输出控制口 初始化
  power_ctrl_configuration();
	
	//power_ctrl_on(2);//输出2号24V可控电源

	//fric_PWM_configuration();//摩擦轮pwm初始化
	
	delay_ms(500);					//等待电机初始化
	
	while(1)
	{
		
		if(Flag500Hz)
		{
			Flag500Hz = 0;
			
			IMUSO3Thread();		//imu姿态解算
			IMU_Temp_Ctrl();	//imu温度闭环
			
			Get_Ctrl_Data();

			if(loopcnt>1500)
			{
				Get_Ctrl_Data();
				Shoot_Ctrl();
			}else loopcnt++;
			
			if(RC_Ctl.rc.s2 == RC_SW_MID){//失控终止
				Stop_Chassis_Task();
				CAN1_Send_Msg_gimbal(0, 0, 0);
				laser_off();
				fric_off();
			}
			
			else if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_DOWN)//遥控器 左上 右下 底盘控制 底盘跟随云台
			{
				laser_off();
//				shoot_control_loop(Fric_OFF);
				Chassis_Task();
				//Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				fric_off();
			}
			else if(RC_Ctl.rc.s1 == RC_SW_MID && RC_Ctl.rc.s2 == RC_SW_DOWN){//遥控器 左中 右下 摩擦轮开启 拨弹轮开启 
				//Chassis_Task();
				//shoot_control_loop(Fric_DOWN);
				//Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,0);
				//Twist(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle, 50);
				//Gimbal_Ctrl(Control_data.Pitch_angle, Control_data.Yaw_angle, 0);
			}
			else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN){//遥控器 左下 右下
				//PWM_Write(PWM1_CH1, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//摩擦轮	
				//Chassis_Task();
				//PWM_Write(PWnM1_CH4, 1000+abs(RC_Ctl.rc.ch3-1024)*0.6f);//摩擦轮
				shoot_control_loop(Fric_MID);
				//Twist(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,200);
				Chassis_AutoFollow(Control_data.Vx,Control_data.Vy,Control_data.Wz,Control_data.Pitch_angle,-200);
			}
			//else if(RC_Ctl.rc.s2 == RC_SW_UP){
//				shoot_control_loop(targetSpeed);
//				
//				if(RC_Ctl.mouse.press_l){//摩擦轮高速开启，拨盘电机转动
//					if(RC_Ctl.key.Shift){
//						targetSpeed = Fric_UP;
//						shootSpeed = 150;
//					}
//					else if(RC_Ctl.key.Ctrl){//单发射击
//						targetSpeed = Fric_MID;
//						shootSpeed = 200;
//					}
//					else{//普通射速
//						targetSpeed = Fric_MID;
//						shootSpeed = 300;
//					}
				//}
//				else if(RC_Ctl.mouse.press_r){//摩擦轮高速开启，拨盘电机转动
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
			
			
			//Astrict_Acc(Control_data.Vx,Control_data.Vy,Control_data.Wz);//加速度限制
			//Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//底盘控制接口
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
//			PWM_Write(PWM1_CH1,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//摩擦轮	
//      PWM_Write(PWM1_CH4,1000+abs(RC_Ctl.rc.ch3-1024)*1.5);//摩擦轮
		}
	}
}

