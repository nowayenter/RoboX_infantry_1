/*************************************************************
 *蜂鸣器任务接口实现函数
 *
 *蜂鸣器在PB4 TIM3_CH1 上，在TIM3_PWM_Init(void)中初始化
 ************************************************************/
#include "buzzer.h"
#include "User_Api.h"
#include "Configuration.h"

uint8_t buzzer=0;
uint16_t start_time;


//蜂鸣器任务接口
//当蜂鸣器工作标志buzzer被置1时，蜂鸣器工作一段时间
//时间长短可在config文件里配置
void Buzzer_Task(void)
{
	if(buzzer==1)
	{
		buzzer = 0;
		start_time = BUZZER_TIME/2;
		TIM3->CCR1 = 300;		//开始工作
	}
	
	if(start_time > 0)
	{
		start_time--;
	}
	else if(start_time==0)	TIM3->CCR1 = 0;
	
}


