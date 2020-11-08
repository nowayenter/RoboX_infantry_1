#ifndef GIMBAL_H
#define GIMBAL_H

#include "sys.h"
#include "usart.h"
#include "delay.h"

extern float printfBuff1,printfBuff2;
extern float Yaw_EncodeAngle,Pitch_EncodeAngle;//使用码盘计算出来的云台角度

static void Pitch_pid(float Target_angle,float Measure_angle);
//static void Yaw_angle_pid(float Targrt_d_angle);

void Encoder_angle_conversion(void);
//void Gimbal_Ctrl(float pitchAngle,float yawAngle,uint16_t shoot_speed);

#endif
