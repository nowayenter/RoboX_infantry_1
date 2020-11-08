/*
 *与裁判系统通信
 */
#include "Comm_umpire.h"
#include "led.h"
#include "CRC.h"


#define UART_RX_LEN     100
u8 Rx_buffer[UART_RX_LEN];		//接收缓冲
u8 SendBuff[UART_RX_LEN];			//发送数据缓冲区
static u8 Pointer;						//接收缓冲指针

RoboState_TypeDef Umpire_State;					//机器人状态
RoboHurt_TypeDef Umpire_Hurt;						//伤害数据
ShootData_TypeDef Umpire_Shoot;					//实时射击信息
PowerHeatData_TypeDef Umpire_PowerHeat;	//实时功率热量数据
RfidDetect_TypeDef Umpire_Rfid;					//场地交互数据
GameResult_TypeDef Umpire_GameResult;		//比赛胜负数据
BuffMusk_TypeDef Umpire_Buff;						//Buff数据
GameRobotPos_TypeDef Umpire_RobotPos;		//机器人位置朝向信息


/****串口dma初始化，用于接收裁判系统数据****/
/*-----USART6_RX-----PG9----DMA2CH5Stream1&2*/
void Comm_umpire_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* config clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	/* gpio config */
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//Rx，PG9                                                                                                                                                                                                                                                                                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	/* USART6 mode config */
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&USART_InitStructure);
	
	USART_Cmd(USART6,ENABLE);											//使能USART6
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);	//使能串口DMA接收
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);	//使能串口空闲中断
		
	/* 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* DMA配置 */
	DMA_DeInit(DMA2_Stream2);		//重置为缺省值
	DMA_InitStructure.DMA_Channel= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);		//源地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_buffer;					//目的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//数据传输方向为外设到内存
	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;												//设置数据的缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//内存缓冲区地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//工作在常规模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;										//高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								//不使用FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	
//	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//不使用dma中断
  DMA_Cmd(DMA2_Stream2,ENABLE);
}


//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	  
 
///////////////////////////////////////////////////////////////////////////
//串口1
//发送单字节
 void SendByteInfoProc(u8 nSendInfo)
{
	u8 *pBuf = NULL;
	//指向发送缓冲区
	pBuf = SendBuff;
	*pBuf++ = nSendInfo;
 
	DmaSendDataProc(DMA2_Stream7,1); //开始一次DMA传输！	  
}
//发送多字节
void SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
	u16 i = 0;
	u8 *pBuf = NULL;
	//指向发送缓冲区
	pBuf = SendBuff;
 
	for (i=0; i<nSendCount; i++)
	{
		*pBuf++ = pSendInfo[i];
	}
	//DMA发送方式
	DmaSendDataProc(DMA2_Stream7,nSendCount); //开始一次DMA传输！	  
}
 
//发送完成中断
void DMA2_Stream7_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
	{ 
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
	}
}


//#define printf_buff
//串口空闲中断
void USART6_IRQHandler(void)                                
{
	uint16_t Length = 0;//数据长度
	LED1 =! LED1;	//绿色灯闪
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) 
	{ 
		DMA_Cmd(DMA2_Stream2, DISABLE); //关闭DMA,防止处理其间有数据
		Length = USART6->SR; 
		Length = USART6->DR; //清USART_IT_IDLE标志 
		Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);//得到数据长度
		
		#ifdef printf_buff
		printf("%d,%d,%d,%d,%d,%d,%d\r\n",Rx_buffer[0],Rx_buffer[1],Rx_buffer[2],Rx_buffer[3],Rx_buffer[4],Rx_buffer[27],Rx_buffer[28]);
		Rx_buffer[0] = 0;
		Rx_buffer[1] = 0;
		Rx_buffer[2] = 0;
		Rx_buffer[3] = 0;
		Rx_buffer[4] = 0;
		Rx_buffer[27] = 0;
		Rx_buffer[28] = 0;
		printf("%d\r\n",Length);
		#else
		Get_UmpireData(Length);
		#endif
		
		//清除传输完成标志
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		DMA_SetCurrDataCounter(DMA2_Stream2, UART_RX_LEN);
		DMA_Cmd(DMA2_Stream2, ENABLE);//处理完,重开DMA
	}
}



//获取裁判系统数据
void Get_UmpireData(uint16_t Length)
{
	for(Pointer=0;Pointer<Length;Pointer++)
	{
		if(Rx_buffer[Pointer] == 0xA5)//检测到起始标志
		{
			Frame_TypeDef Frame={0};
			Frame.DataLength = Rx_buffer[Pointer+1] | (Rx_buffer[Pointer+2]<<8);//数据段data长度
			Frame.Seq = Rx_buffer[Pointer+3];//包序号
			Frame.CRC8 = Rx_buffer[Pointer+4];//帧头检验crc8
			Frame.CmdID = Rx_buffer[Pointer+5] | (Rx_buffer[Pointer+6]<<8);//命令id
			Frame.FrameTail = Rx_buffer[Pointer+7+Frame.DataLength] | (Rx_buffer[Pointer+8+Frame.DataLength]<<8);//整包检验crc16
			
			//crc校验
			if( (Verify_CRC8_Check_Sum((u8 *)&Rx_buffer[Pointer],5)) && 
				( Verify_CRC16_Check_Sum((u8 *)&Rx_buffer[Pointer],9+Frame.DataLength)) )
			{
				DealDataPack(Frame.CmdID,Frame.DataLength);//解析数据
			}//end crc
		}
	}
}



//处理数据包
void DealDataPack(uint16_t CmdID, uint16_t DataLength)
{
	Pointer += 7;//偏移7，指到数据段第一个字节
	switch(CmdID)
	{
		case 0x0001:	//机器人状态
		{
			u8 *f1 = (u8 *)&Umpire_State;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		case 0x0002:	//伤害数据
		{
			u8 *f1 = (u8 *)&Umpire_Hurt;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		case 0x0003:	//实时射击信息
		{
			u8 *f1 = (u8 *)&Umpire_Shoot;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			printf("%f\r\n",Umpire_Shoot.bulletSpeed);
			break;
		}
		case 0x0004:	//实时功率热量数据
		{
			u8 *f1 = (u8 *)&Umpire_PowerHeat;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		case 0x0005:	//场地交互数据
		{
			u8 *f1 = (u8 *)&Umpire_Rfid;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}case 0x0006:	//比赛胜负数据
		{
			u8 *f1 = (u8 *)&Umpire_GameResult;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		case 0x0007:	//Buff数据
		{
			u8 *f1 = (u8 *)&Umpire_Buff;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		case 0x0008:	//机器人位置朝向信息
		{
			u8 *f1 = (u8 *)&Umpire_RobotPos;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}
		
		default:
			break;
	}
}





