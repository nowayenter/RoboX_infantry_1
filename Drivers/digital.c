//���ܣ��û��Զ���˿������������
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "digital.h"
//���ֶ˿ڳ�ʼ��
void digital_init()
{
   	GPIO_InitTypeDef GPIO_InitStructure;


		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; 

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//��ͨ���ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//����
	
		GPIO_Init(GPIOC, &GPIO_InitStructure);		//��ʼ�� GPIO
}

