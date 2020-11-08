/*
 *can2初始化、接收、发送程序
 *用于与tx2视觉上位机数据交换
 */

#include "can2.h"
#include "led.h"
#include "Configuration.h"

u8 TX2_Data[8];
Vision_InitTypeDef Vision_Data={0};

void CAN2_Configuration(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef       GPIO_InitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//打开GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);//配置引脚复用功能
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//复用推挽输出
	GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;				//中断配置 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure); 
    
	CAN_DeInit(CAN2);                        //将外围寄存器初始化到它们的默认重置值
	CAN_StructInit(&CAN_InitStructure);      //用它的默认值填充每个CAN_InitStructure成员
	
   /************CAN总线的配置*******************/
	CAN_InitStructure.CAN_TTCM = DISABLE;			//非时间触发通信模式
	CAN_InitStructure.CAN_ABOM = DISABLE;			//软件自动离线管理模式
	CAN_InitStructure.CAN_AWUM = DISABLE;			//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART = DISABLE;			//非自动重传输模式，禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM = DISABLE;			//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
	CAN_InitStructure.CAN_TXFP = ENABLE;			//发送FIFO优先迹，优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//模式设置： mode:0,普通模式;1,回环模式;
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS1_8tq;
	CAN_InitStructure.CAN_Prescaler = 3;   //brp    CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &CAN_InitStructure);

	/******CAN总线的过滤配置(接收配置)********/
	CAN_FilterInitStructure.CAN_FilterNumber=14;                  //过滤器 0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化   

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		//接收中断  启用或禁用指定的CANx中断  CAN_IT_FMP0:等待中断的FIFO 0消息
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//发送中断
}




//can中断服务函数，接受数据
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	LED1 =! LED1;	//绿色灯闪，代表接收到电机数据
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x001://TX2图像数据
			{
				TX2_Data[0] = RxMessage.Data[0];
				TX2_Data[1] = RxMessage.Data[1];
				TX2_Data[2] = RxMessage.Data[2];
				TX2_Data[3] = RxMessage.Data[3];
				TX2_Data[4] = RxMessage.Data[4];
				TX2_Data[5] = RxMessage.Data[5];
				TX2_Data[6] = RxMessage.Data[6];
				TX2_Data[7] = RxMessage.Data[7];
				
				Vision_Data.mode = RxMessage.Data[0];
				Vision_Data.x = ((RxMessage.Data[1]<<8)|RxMessage.Data[2]) - VISION_X_Pixels/2;
				Vision_Data.y = ((RxMessage.Data[3]<<8)|RxMessage.Data[4]) - VISION_Y_Pixels/2;
				Vision_Data.distance = ((RxMessage.Data[5]<<8)|RxMessage.Data[6])*0.01;
				break;
			}
			default:
				break;
		}
	}	
}




void CAN2_Send(u8 Msg)   //can2发送给tx2测试
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//定义一个发送信息的结构体
	
  TxMessage.StdId=0x002;	 // 标准标识符为0
  TxMessage.IDE = CAN_ID_STD;			//指定将要传输的消息的标识符的类型
  TxMessage.RTR = CAN_RTR_DATA;		//指定的帧将被传输的消息的类型   数据帧或远程帧
  TxMessage.DLC = 2;			//指定数据的长度
	TxMessage.Data[0] = 5;
	TxMessage.Data[1] = 6; 
	
  mbox = CAN_Transmit(CAN2, &TxMessage);
	
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
}
