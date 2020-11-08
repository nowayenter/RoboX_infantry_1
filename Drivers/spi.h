#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


 	    													  
void SPI5_Init(void);			 //初始化SPI1口
void SPI5_SetSpeed(u8 SpeedSet); //设置SPI1速度   
u8 SPI5_ReadWriteByte(u8 TxData);
		 
#endif

