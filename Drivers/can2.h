#ifndef __CAN2_H
#define __CAN2_H
#include "sys.h"
#include "usart.h"
#include "delay.h"



void CAN2_Configuration(void);
void CAN2_Send(u8 Msg);

extern u8 TX2_Data[8];

typedef struct			    //�Ӿ�������ݽṹ��
{
	u8 mode;				//ģʽ
	int16_t x;			//����
	int16_t y;		
	float distance;	//����
	
}Vision_InitTypeDef;
extern Vision_InitTypeDef Vision_Data;


#endif
