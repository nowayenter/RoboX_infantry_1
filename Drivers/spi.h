#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


 	    													  
void SPI5_Init(void);			 //��ʼ��SPI1��
void SPI5_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI5_ReadWriteByte(u8 TxData);
		 
#endif

