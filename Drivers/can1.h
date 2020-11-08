#ifndef __CAN1_H
#define __CAN1_H
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "Configuration.h"

#define Hz_Flag (1000/PID_Hz)		//计数次数

typedef struct Motor_data    //电机相关数据结构体
{	
	int16_t ActualSpeed;		//本次速度值
	int16_t last_speed;			//上次速度值
	int16_t NowAngle;				//当前机械角
	int16_t OldAngle;				//上次机械角
	int16_t D_Angle;				//一周期机械角差值
	
	int16_t Intensity;			//测量转矩电流，云台电机数据
}M_Data;

extern M_Data motor_data[7];		//电机数据



//6个电机的位置
enum {M0,M1,M2,M3,YAW,PITCH};

//电机can初始化
void CAN1_Configuration(void);

//电机数据发送
void CAN1_SendCommand_chassis(signed long ESC_201,signed long ESC_202,
							signed long ESC_203,signed long ESC_204);
void CAN1_Send_Msg_gimbal(int16_t control_205,int16_t control_206, int16_t control_2006);

int16_t kalman(int16_t x,uint8_t i);			//机械角卡尔曼
void Angle_deal(int i);				//机械角越界处理


#endif
