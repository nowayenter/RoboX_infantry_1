#ifndef IMUSO3_H
#define IMUSO3_H

#include "sys.h"
#include "MPU6500.h"


extern uint8_t bFilterInit;	//��������������־λ

//У׼ʱ��
#define ACC_CALC_TIME  3000//ms
#define GYRO_CALC_TIME   3000000l	//us

/* Function prototypes */
static float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

void IMUSO3Thread(void);	//imu��ȡ����̬�����ܽӿ�
void IMU_Temp_Ctrl(void);	//imu�¶ȱջ�

#endif

