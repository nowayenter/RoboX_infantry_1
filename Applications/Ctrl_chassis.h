#ifndef CHASSIS_H
#define CHASSIS_H
#include "sys.h"


typedef struct Motor_Pid	//位置环pid数据结构
{
	int old_aim;			//旧目标码数值
	int old_err;			//旧差值
	int16_t output;		//输出
}M_pid;

void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed);
void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz);


#endif
