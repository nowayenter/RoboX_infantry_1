/*
 *使用定时器产生时间Flag
 */

#include "timer.h"

#include "Ctrl_chassis.h"
#include "PID.h"
#include "can1.h"
#include "RemoteControl.h"


//时基计数用
u8 Hz500_Cnt=0,Hz200_Cnt1=0,Hz100_Cnt1=1,Hz50_Cnt=2,Hz200_Cnt2=3,Hz100_Cnt2=4;		//错开各个flag的发生时间

u8 Flag50Hz,Flag100Hz_Thread1,Flag100Hz_Thread2,Flag200Hz_Thread1,Flag200Hz_Thread2,Flag500Hz;	//Hz Flag

void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	//TIM3定时中断设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设
}



void TIM4_IRQHandler(void)	
{	
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除中断标志位
		
		Hz500_Cnt++;
		Hz200_Cnt1++;
		Hz200_Cnt2++;
		Hz100_Cnt1++;
		Hz100_Cnt2++;
		Hz50_Cnt++;
		
		//500hz线程
		if(Hz500_Cnt == 2)//1000/500=2
		{
			Hz500_Cnt = 0;
			Flag500Hz = 1;
			
			//PID_Control(&motor_speed_pid[0],Control_data.RC_ch1*6,motor_data[0].ActualSpeed);
			//CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,0,0,0);
		}
		//200hz线程1
		if(Hz200_Cnt1 == 5)//1000/200=5
		{
			Hz200_Cnt1 = 0;
			Flag200Hz_Thread1 = 1;
		}
		//200hz线程2
		if(Hz200_Cnt2 == 5)//1000/200=5
		{
			Hz200_Cnt2 = 0;
			Flag200Hz_Thread2 = 1;
		}
		//100hz线程1
		if(Hz100_Cnt1 == 10)//1000/100=10
		{
			Hz100_Cnt1 = 0;
			Flag100Hz_Thread1 = 1;
		}
		//100hz线程2
		if(Hz100_Cnt2 == 10)//1000/100=10
		{
			Hz100_Cnt2 = 0;
			Flag100Hz_Thread2 = 1;
		}
		//50hz线程
		if(Hz50_Cnt == 20)//1000/50=20
		{
			Hz50_Cnt = 0;
			Flag50Hz = 1;
		}
	}
}

