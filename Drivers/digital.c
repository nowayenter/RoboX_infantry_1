//功能：用户自定义端口数字输出控制
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "digital.h"
//数字端口初始化
void digital_init()
{
   	GPIO_InitTypeDef GPIO_InitStructure;


		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; 

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	
		GPIO_Init(GPIOC, &GPIO_InitStructure);		//初始化 GPIO
}

