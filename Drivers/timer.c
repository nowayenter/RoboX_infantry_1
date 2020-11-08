/*
 *ʹ�ö�ʱ������ʱ��Flag
 */

#include "timer.h"

#include "Ctrl_chassis.h"
#include "PID.h"
#include "can1.h"
#include "RemoteControl.h"


//ʱ��������
u8 Hz500_Cnt=0,Hz200_Cnt1=0,Hz100_Cnt1=1,Hz50_Cnt=2,Hz200_Cnt2=3,Hz100_Cnt2=4;		//������flag�ķ���ʱ��

u8 Flag50Hz,Flag100Hz_Thread1,Flag100Hz_Thread2,Flag200Hz_Thread1,Flag200Hz_Thread2,Flag500Hz;	//Hz Flag

void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��

	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	//TIM3��ʱ�ж�����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
		
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx����
}



void TIM4_IRQHandler(void)	
{	
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) //����ж�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //����жϱ�־λ
		
		Hz500_Cnt++;
		Hz200_Cnt1++;
		Hz200_Cnt2++;
		Hz100_Cnt1++;
		Hz100_Cnt2++;
		Hz50_Cnt++;
		
		//500hz�߳�
		if(Hz500_Cnt == 2)//1000/500=2
		{
			Hz500_Cnt = 0;
			Flag500Hz = 1;
			
			//PID_Control(&motor_speed_pid[0],Control_data.RC_ch1*6,motor_data[0].ActualSpeed);
			//CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,0,0,0);
		}
		//200hz�߳�1
		if(Hz200_Cnt1 == 5)//1000/200=5
		{
			Hz200_Cnt1 = 0;
			Flag200Hz_Thread1 = 1;
		}
		//200hz�߳�2
		if(Hz200_Cnt2 == 5)//1000/200=5
		{
			Hz200_Cnt2 = 0;
			Flag200Hz_Thread2 = 1;
		}
		//100hz�߳�1
		if(Hz100_Cnt1 == 10)//1000/100=10
		{
			Hz100_Cnt1 = 0;
			Flag100Hz_Thread1 = 1;
		}
		//100hz�߳�2
		if(Hz100_Cnt2 == 10)//1000/100=10
		{
			Hz100_Cnt2 = 0;
			Flag100Hz_Thread2 = 1;
		}
		//50hz�߳�
		if(Hz50_Cnt == 20)//1000/50=20
		{
			Hz50_Cnt = 0;
			Flag50Hz = 1;
		}
	}
}

