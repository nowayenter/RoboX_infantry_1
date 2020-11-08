/*
 *�����ϵͳͨ��
 */
#include "Comm_umpire.h"
#include "led.h"
#include "CRC.h"


#define UART_RX_LEN     100
u8 Rx_buffer[UART_RX_LEN];		//���ջ���
u8 SendBuff[UART_RX_LEN];			//�������ݻ�����
static u8 Pointer;						//���ջ���ָ��

RoboState_TypeDef Umpire_State;					//������״̬
RoboHurt_TypeDef Umpire_Hurt;						//�˺�����
ShootData_TypeDef Umpire_Shoot;					//ʵʱ�����Ϣ
PowerHeatData_TypeDef Umpire_PowerHeat;	//ʵʱ������������
RfidDetect_TypeDef Umpire_Rfid;					//���ؽ�������
GameResult_TypeDef Umpire_GameResult;		//����ʤ������
BuffMusk_TypeDef Umpire_Buff;						//Buff����
GameRobotPos_TypeDef Umpire_RobotPos;		//������λ�ó�����Ϣ


/****����dma��ʼ�������ڽ��ղ���ϵͳ����****/
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//Rx��PG9                                                                                                                                                                                                                                                                                                          
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
	
	USART_Cmd(USART6,ENABLE);											//ʹ��USART6
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���DMA����
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);	//ʹ�ܴ��ڿ����ж�
		
	/* �ж����� */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* DMA���� */
	DMA_DeInit(DMA2_Stream2);		//����Ϊȱʡֵ
	DMA_InitStructure.DMA_Channel= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);		//Դ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_buffer;					//Ŀ�ĵ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//���ݴ��䷽��Ϊ���赽�ڴ�
	DMA_InitStructure.DMA_BufferSize = UART_RX_LEN;												//�������ݵĻ����С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�ڴ滺������ַ�Լ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//�����ڳ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;										//�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								//��ʹ��FIFOģʽ
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	
//	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//��ʹ��dma�ж�
  DMA_Cmd(DMA2_Stream2,ENABLE);
}


//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	  
 
///////////////////////////////////////////////////////////////////////////
//����1
//���͵��ֽ�
 void SendByteInfoProc(u8 nSendInfo)
{
	u8 *pBuf = NULL;
	//ָ���ͻ�����
	pBuf = SendBuff;
	*pBuf++ = nSendInfo;
 
	DmaSendDataProc(DMA2_Stream7,1); //��ʼһ��DMA���䣡	  
}
//���Ͷ��ֽ�
void SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
	u16 i = 0;
	u8 *pBuf = NULL;
	//ָ���ͻ�����
	pBuf = SendBuff;
 
	for (i=0; i<nSendCount; i++)
	{
		*pBuf++ = pSendInfo[i];
	}
	//DMA���ͷ�ʽ
	DmaSendDataProc(DMA2_Stream7,nSendCount); //��ʼһ��DMA���䣡	  
}
 
//��������ж�
void DMA2_Stream7_IRQHandler(void)
{
	//�����־
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
	{ 
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
	}
}


//#define printf_buff
//���ڿ����ж�
void USART6_IRQHandler(void)                                
{
	uint16_t Length = 0;//���ݳ���
	LED1 =! LED1;	//��ɫ����
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) 
	{ 
		DMA_Cmd(DMA2_Stream2, DISABLE); //�ر�DMA,��ֹ�������������
		Length = USART6->SR; 
		Length = USART6->DR; //��USART_IT_IDLE��־ 
		Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);//�õ����ݳ���
		
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
		
		//���������ɱ�־
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
		DMA_SetCurrDataCounter(DMA2_Stream2, UART_RX_LEN);
		DMA_Cmd(DMA2_Stream2, ENABLE);//������,�ؿ�DMA
	}
}



//��ȡ����ϵͳ����
void Get_UmpireData(uint16_t Length)
{
	for(Pointer=0;Pointer<Length;Pointer++)
	{
		if(Rx_buffer[Pointer] == 0xA5)//��⵽��ʼ��־
		{
			Frame_TypeDef Frame={0};
			Frame.DataLength = Rx_buffer[Pointer+1] | (Rx_buffer[Pointer+2]<<8);//���ݶ�data����
			Frame.Seq = Rx_buffer[Pointer+3];//�����
			Frame.CRC8 = Rx_buffer[Pointer+4];//֡ͷ����crc8
			Frame.CmdID = Rx_buffer[Pointer+5] | (Rx_buffer[Pointer+6]<<8);//����id
			Frame.FrameTail = Rx_buffer[Pointer+7+Frame.DataLength] | (Rx_buffer[Pointer+8+Frame.DataLength]<<8);//��������crc16
			
			//crcУ��
			if( (Verify_CRC8_Check_Sum((u8 *)&Rx_buffer[Pointer],5)) && 
				( Verify_CRC16_Check_Sum((u8 *)&Rx_buffer[Pointer],9+Frame.DataLength)) )
			{
				DealDataPack(Frame.CmdID,Frame.DataLength);//��������
			}//end crc
		}
	}
}



//�������ݰ�
void DealDataPack(uint16_t CmdID, uint16_t DataLength)
{
	Pointer += 7;//ƫ��7��ָ�����ݶε�һ���ֽ�
	switch(CmdID)
	{
		case 0x0001:	//������״̬
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
		case 0x0002:	//�˺�����
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
		case 0x0003:	//ʵʱ�����Ϣ
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
		case 0x0004:	//ʵʱ������������
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
		case 0x0005:	//���ؽ�������
		{
			u8 *f1 = (u8 *)&Umpire_Rfid;
			while(DataLength--)
			{
				*f1 = Rx_buffer[Pointer];
				f1++;
				Pointer++;
			}
			break;
		}case 0x0006:	//����ʤ������
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
		case 0x0007:	//Buff����
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
		case 0x0008:	//������λ�ó�����Ϣ
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





