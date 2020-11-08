#ifndef __USERAPI_H
#define __USERAPI_H
#include "sys.h"

typedef struct
{
	float Vx;			//前后速度
	float Vy;			//左右速度
	float Wz;			//旋转速度
	
	float Pitch_angle;	//pitch轴角度
	float Yaw_angle;		//yaw
	
}ControlDATA_TypeDef;
extern ControlDATA_TypeDef Control_data;//控制数据

//射击方案
typedef struct
{
	u16 Frequency;	//射频
	float Speed;		//射速
	
}ShootProject_TypeDef;


extern uint8_t status[];

extern u8 TrashyTime_Flag;	//垃圾时间，开局时的垃圾时间
extern u8 Vision_Flag;			//视觉自瞄
extern u8 ShootBuff_Flag;		//打符
extern u8 Shoot_Flag;				//射击
extern u8 Twist_Flag;				//扭腰
extern u8 Shoot_Motor;			//摩擦轮

extern void Chassis_Task(void);	//底盘控制
extern void Stop_Chassis_Task(void);//终止底盘控制
extern void Gimbal_Task(void);		//云台控制

extern void User_Api(void);
extern void Mode_Monitor(void);
extern void Get_Ctrl_Data(void);
extern void MultiFuntion(void);
extern void Shoot_Ctrl(void);
#endif



