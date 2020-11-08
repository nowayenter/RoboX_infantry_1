#ifndef __LED_H
#define __LED_H
#include "main.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"


//LED 端口定义
#define LED0 PEout(7) // DS0，红
#define LED1 PFout(14)// DS1，绿

#define LASER PGout(13) //LASER0

void LED_Init(void);//初始化
void Laser_Init(void);//初始化激光
extern void laser_on(void);
extern void laser_off(void);
#endif
