#ifndef _PID_H_
#define _PID_H_
#include "sys.h"


//�˲�������
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;
//�˲�������
typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;


//pid���ݽṹ
typedef struct
{
    u8 Err_Limit_Flag :1;//ƫ���޷���־
    u8 Integrate_Limit_Flag :1;//�����޷���־
    u8 Integrate_Separation_Flag :1;//���ַ����־
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate_Separation_Err;//���ַ���ƫ��ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float Kp;//���Ʋ���Kp
    float Ki;//���Ʋ���Ki
    float Kd;//���Ʋ���Kd
    float Control_OutPut;//�����������
    float Last_Control_OutPut;//�ϴο����������
    float Control_OutPut_Limit;//����޷�
    /***************************************/
    float Last_FeedBack;//�ϴη���ֵ
    float Dis_Err;//΢����
    float Dis_Error_History[5];//��ʷ΢����
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������
}PID_Controler;

extern PID_Controler motor_speed_pid[4];	//����ٶȻ�pid����
extern PID_Controler Yaw_speed_pid;		//��̨yaw����ٶ�
//extern PID_Controler Yaw_angle_pid;		//��̨yaw��Ƕ�
extern PID_Controler Pitch_angle_pid;	//��̨pitch��Ƕ�
extern PID_Controler Pitch_speed_pid;	//��̨pitch����ٶ�
extern PID_Controler Fire_speed_pid;	//���ֵ���ٶ�

void  Total_PID_Init(void);
void  PID_Init(PID_Controler *Controler);

float PID_Control(PID_Controler *Controler,float target,float measure);
float PID_Control_Div_LPF(PID_Controler *Controler,float target,float measure);
float PID_Control_Err_LPF(PID_Controler *Controler,float target,float measure);
void  PID_Integrate_Reset(PID_Controler *Controler);


#endif


