#ifndef __H_CLASS_H
#define __H_CLASS_H
#include "sys.h"
//�Ӿ�ʶ������λ�����͵�ָ��
#define Vision_Discern_RedArmor			12		//ʶ���ɫװ��
#define Vision_Discern_BlueArmor		22		//ʶ����ɫװ��




void Vision_Gimbal(float Vx,float Vy,float Wz,float pitch_angle,u8 Twist);//�Զ���׼
//void arm_open(void); //��е����չ
//void arm_close(void); //��е���ջ�
//void arm_catch(void);//��е�ۼ�ȡ
//void arm_release(void);//��е�۷���
//void arm_ctrl(void);//��е��һ��ȡ��
//void chassis_ctrl(u8 k);//������������
//void bullet_ctrl(u8 k);//���տ���
//void left_chassis(u8 k);
//void right_chassis(u8 k);
extern void Astrict_Acc(float Vx,float Vy,float Wz);
extern void Matrix_Translate(float *X,float *Y,float angle);
extern float Vx_Lpf,Vy_Lpf,Wz_Lpf;
extern void Vision_Mode(u8 mode);

#endif

