#ifndef CHASSIS_H
#define CHASSIS_H
#include "sys.h"


typedef struct Motor_Pid	//λ�û�pid���ݽṹ
{
	int old_aim;			//��Ŀ������ֵ
	int old_err;			//�ɲ�ֵ
	int16_t output;		//���
}M_pid;

void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed);
void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz);


#endif
