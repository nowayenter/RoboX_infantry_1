/*
 *��ȡ���ԡ�ң����ԭʼ����
 */
#include "RemoteControl.h"

volatile unsigned char sbus_rx_buffer[25];		//������ջ���

RC_Ctl_t RC_Ctl;								//ң�ؽṹ�嶨��
u8 failsafe_flag = 0;						//���������־


/*-----USART1_RX-----PB7----*/ 
void RC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* config USART1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		//Rx��PB7                                                                                                                                                                                                                                                                                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/* USART3 mode config */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;	//SBUS 100K baudrate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);											//ʹ��USART1
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���DMA����
		
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream5, DISABLE);
  while (DMA2_Stream5->CR & DMA_SxCR_EN);
	
	DMA_DeInit(DMA2_Stream5);		//����Ϊȱʡֵ
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);		//Դ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;			//Ŀ�ĵ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								//���ݴ��䷽��Ϊ���赽�ڴ�
	DMA_InitStructure.DMA_BufferSize = 18;																//�������ݵĻ����С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�ڴ滺������ַ�Լ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												//������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;								//������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream5,&DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
  DMA_Cmd(DMA2_Stream5,ENABLE);
	
}

void DMA2_Stream5_IRQHandler(void)		//ң�����ݵĽ���
{
	//�ж��Ƿ�ΪDMA��������ж�
  if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		Get_RC_Data();				//�����յ������ݱ��浽sbus_rx_buffer��m
	  FailSafe_Check();   //����ʧ�ر���
		
#ifdef USE_DMA_USART_DBUG
		printf("CH0:%d	\n",RC_Ctl.rc.ch0);		//���ڵ���
		printf("CH1:%d	\n",RC_Ctl.rc.ch1);
		printf("CH2:%d	\n",RC_Ctl.rc.ch2);
		printf("CH3:%d	\n",RC_Ctl.rc.ch3);
		printf("S1: %d  \n",RC_Ctl.rc.s1);
		printf("S2: %d  \n",RC_Ctl.rc.s2);
		printf("mouse-x:%d	\n",RC_Ctl.mouse.x);
		printf("mouse-y:%d	\n",RC_Ctl.mouse.y);
		printf("mouse-z:%d	\n",RC_Ctl.mouse.z);
		printf("press_l:%d	\n",RC_Ctl.mouse.press_l);
		printf("press_r:%d	\n",RC_Ctl.mouse.press_r);
		printf("PC-key:%d	\n",RC_Ctl.key.v);
#endif		 
	}
}

//���԰������ݽ���
void KeyDataProcess(Mouse *mouse, Key *key)
{
	//ǰ��
	if(key->buff & KEY_PRESSED_OFFSET_W)   		//Key W
		key->Vertical = 1;
	else if(key->buff & KEY_PRESSED_OFFSET_S) //Key S
		key->Vertical = -1;
	else
		key->Vertical = 0;
	//����
	if(key->buff & KEY_PRESSED_OFFSET_D)   		//Key D
		key->Horizontal = 1;
	else if(key->buff & KEY_PRESSED_OFFSET_A) //Key A
		key->Horizontal = -1;
	else
		key->Horizontal = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_Q)//Key q
		key->Q = 1;
	else key->Q = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_E)//Key e
		key->E = 1;
	else key->E = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_R)//Key r
		key->R = 1;
	else key->R = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_F)//Key f
		key->F = 1;
	else key->F = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_G)//Key g
		key->G = 1;
	else key->G = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_Z)//Key z
		key->Z = 1;
	else key->Z = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_X)//Key x
		key->X = 1;
	else key->X = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_C)//Key c
		key->C = 1;
	else key->C = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_V)//Key v
		key->V = 1;
	else key->V = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_B)//Key b
		key->B = 1;
	else key->B = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_SHIFT)   //Key shitf
		key->Shift = 1;
	else key->Shift = 0;
	
	if(key->buff & KEY_PRESSED_OFFSET_CTRL)   //Key ctrl
		key->Ctrl = 1;
	else key->Ctrl = 0;
}


void Get_RC_Data(void)//��ȡң������
{
	RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; 						//!< Channel 0
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; 		//!< Channel 1
	RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;															//!< Switch left
	RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); 																	//!< Switch right	
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); 											//!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);										  //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); 										//!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; 																					//!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; 																					//!< Mouse Right Is Press ?
	RC_Ctl.key.buff = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); 									//!< KeyBoard value
	failsafe_flag = 0;
	
	KeyDataProcess(&RC_Ctl.mouse,&RC_Ctl.key);//���԰������ݽ���
}

void FailSafe_Check(void)			//ʧ�ر������
{
	if(failsafe_flag < 250)
	{	
		failsafe_flag++;
	}
	if(failsafe_flag >= 10)
	{
		RC_Ctl.rc.s2 = RC_SW_DOWN;
	}
}
