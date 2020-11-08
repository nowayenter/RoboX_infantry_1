#ifndef __CAN2_H
#define __CAN2_H
#include "sys.h"
#include "usart.h"
#include "delay.h"



void CAN2_Configuration(void);
void CAN2_Send(u8 Msg);

extern u8 TX2_Data[8];

typedef struct			    //视觉相关数据结构体
{
	u8 mode;				//模式
	int16_t x;			//横轴
	int16_t y;		
	float distance;	//距离
	
}Vision_InitTypeDef;
extern Vision_InitTypeDef Vision_Data;


#endif
