#ifndef _PID_H_
#define _PID_H_
#include "sys.h"


//滤波器数据
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;
//滤波器参数
typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;


//pid数据结构
typedef struct
{
    u8 Err_Limit_Flag :1;//偏差限幅标志
    u8 Integrate_Limit_Flag :1;//积分限幅标志
    u8 Integrate_Separation_Flag :1;//积分分离标志
    float Err;//偏差
    float Last_Err;//上次偏差
    float Err_Max;//偏差限幅值
    float Integrate_Separation_Err;//积分分离偏差值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float Kp;//控制参数Kp
    float Ki;//控制参数Ki
    float Kd;//控制参数Kd
    float Control_OutPut;//控制器总输出
    float Last_Control_OutPut;//上次控制器总输出
    float Control_OutPut_Limit;//输出限幅
    /***************************************/
    float Last_FeedBack;//上次反馈值
    float Dis_Err;//微分量
    float Dis_Error_History[5];//历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲
}PID_Controler;

extern PID_Controler motor_speed_pid[4];	//电机速度环pid数据
extern PID_Controler Yaw_speed_pid;		//云台yaw轴角速度
//extern PID_Controler Yaw_angle_pid;		//云台yaw轴角度
extern PID_Controler Pitch_angle_pid;	//云台pitch轴角度
extern PID_Controler Pitch_speed_pid;	//云台pitch轴角速度
extern PID_Controler Fire_speed_pid;	//波轮电机速度

void  Total_PID_Init(void);
void  PID_Init(PID_Controler *Controler);

float PID_Control(PID_Controler *Controler,float target,float measure);
float PID_Control_Div_LPF(PID_Controler *Controler,float target,float measure);
float PID_Control_Err_LPF(PID_Controler *Controler,float target,float measure);
void  PID_Integrate_Reset(PID_Controler *Controler);


#endif


