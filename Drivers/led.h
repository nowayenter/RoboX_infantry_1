#ifndef __LED_H
#define __LED_H
#include "main.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"


//LED �˿ڶ���
#define LED0 PEout(7) // DS0����
#define LED1 PFout(14)// DS1����

#define LASER PGout(13) //LASER0

void LED_Init(void);//��ʼ��
void Laser_Init(void);//��ʼ������
extern void laser_on(void);
extern void laser_off(void);
#endif
