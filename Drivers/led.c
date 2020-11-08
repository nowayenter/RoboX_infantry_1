#include "led.h"
#include "stm32f4xx.h"

//初始化led接口
//LED0-PE7-红色，LED1-PF14-绿色
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能 GPIOE 时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能 GPIOF 时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		//LED0 对应 IO 口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);		//初始化 GPIO
	GPIO_SetBits(GPIOE,GPIO_Pin_7 );			//初始化置高，红灯灭
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//LED1 对应 IO 口
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//初始化 GPIO
	GPIO_SetBits(GPIOF,GPIO_Pin_14 );			//初始化置高，绿灯灭
}

void Laser_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		//PG13                                                                                                                                                                                                                                                                                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);
}

void laser_on(void)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_13);
}
void laser_off(void)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}
