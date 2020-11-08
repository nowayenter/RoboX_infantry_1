#ifndef __H_CLASS_H
#define __H_CLASS_H
#include "sys.h"
//视觉识别向上位机发送的指令
#define Vision_Discern_RedArmor			12		//识别红色装甲
#define Vision_Discern_BlueArmor		22		//识别蓝色装甲




void Vision_Gimbal(float Vx,float Vy,float Wz,float pitch_angle,u8 Twist);//自动瞄准
//void arm_open(void); //机械臂伸展
//void arm_close(void); //机械臂收回
//void arm_catch(void);//机械臂夹取
//void arm_release(void);//机械臂放松
//void arm_ctrl(void);//机械臂一键取弹
//void chassis_ctrl(u8 k);//底盘升降控制
//void bullet_ctrl(u8 k);//弹舱控制
//void left_chassis(u8 k);
//void right_chassis(u8 k);
extern void Astrict_Acc(float Vx,float Vy,float Wz);
extern void Matrix_Translate(float *X,float *Y,float angle);
extern float Vx_Lpf,Vy_Lpf,Wz_Lpf;
extern void Vision_Mode(u8 mode);

#endif

