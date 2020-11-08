/*************************************************************
 *����������ӿ�ʵ�ֺ���
 *
 *��������PB4 TIM3_CH1 �ϣ���TIM3_PWM_Init(void)�г�ʼ��
 ************************************************************/
#include "buzzer.h"
#include "User_Api.h"
#include "Configuration.h"

uint8_t buzzer=0;
uint16_t start_time;


//����������ӿ�
//��������������־buzzer����1ʱ������������һ��ʱ��
//ʱ�䳤�̿���config�ļ�������
void Buzzer_Task(void)
{
	if(buzzer==1)
	{
		buzzer = 0;
		start_time = BUZZER_TIME/2;
		TIM3->CCR1 = 300;		//��ʼ����
	}
	
	if(start_time > 0)
	{
		start_time--;
	}
	else if(start_time==0)	TIM3->CCR1 = 0;
	
}


