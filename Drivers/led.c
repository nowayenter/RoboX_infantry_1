#include "led.h"
#include "stm32f4xx.h"

//��ʼ��led�ӿ�
//LED0-PE7-��ɫ��LED1-PF14-��ɫ
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ�� GPIOE ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ�� GPIOF ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		//LED0 ��Ӧ IO ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//����
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);		//��ʼ�� GPIO
	GPIO_SetBits(GPIOE,GPIO_Pin_7 );			//��ʼ���øߣ������
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//LED1 ��Ӧ IO ��
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//��ʼ�� GPIO
	GPIO_SetBits(GPIOF,GPIO_Pin_14 );			//��ʼ���øߣ��̵���
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
