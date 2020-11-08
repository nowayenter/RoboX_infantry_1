#ifndef PWM_H
#define PWM_H
#include "sys.h"
#include "main.h"

#define PWM1_CH1 11
#define PWM1_CH2 12
#define PWM1_CH3 13
#define PWM1_CH4 14

#define BUZZER			TIM3->CCR1
#define IMU_TEMP_R	TIM3->CCR2	//imuÎÂ¶È²¹³¥µç×è


void TIM12_PWM_Init(void);	//·äÃùÆ÷
void TIM3_PWM_Init(void);		//imuÎÂ¶È²¹³¥
void TIM1_PWM_Init(void);		//Ä¦²ÁÂÖ
void PWM_Write(uint8_t PWM_CH,int16_t PWM_Value);

#endif
