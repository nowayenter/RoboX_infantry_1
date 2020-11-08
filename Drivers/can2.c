/*
 *can2��ʼ�������ա����ͳ���
 *������tx2�Ӿ���λ�����ݽ���
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

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);//�������Ÿ��ù���
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//�����������
	GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;				//�ж����� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//��ռ���ȼ� 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//��Ӧ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure); 
    
	CAN_DeInit(CAN2);                        //����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
	CAN_StructInit(&CAN_InitStructure);      //������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա
	
   /************CAN���ߵ�����*******************/
	CAN_InitStructure.CAN_TTCM = DISABLE;			//��ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_ABOM = DISABLE;			//����Զ����߹���ģʽ
	CAN_InitStructure.CAN_AWUM = DISABLE;			//�Զ�����ģʽ��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART = DISABLE;			//���Զ��ش���ģʽ����ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM = DISABLE;			//����FIFO����ģʽ�����Ĳ�����,�µĸ��Ǿɵ� 
	CAN_InitStructure.CAN_TXFP = ENABLE;			//����FIFO���ȼ������ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS1_8tq;
	CAN_InitStructure.CAN_Prescaler = 3;   //brp    CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &CAN_InitStructure);

	/******CAN���ߵĹ�������(��������)********/
	CAN_FilterInitStructure.CAN_FilterNumber=14;                  //������ 0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��   

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		//�����ж�  ���û����ָ����CANx�ж�  CAN_IT_FMP0:�ȴ��жϵ�FIFO 0��Ϣ
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//�����ж�
}




//can�жϷ���������������
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	LED1 =! LED1;	//��ɫ������������յ��������
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x001://TX2ͼ������
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




void CAN2_Send(u8 Msg)   //can2���͸�tx2����
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//����һ��������Ϣ�Ľṹ��
	
  TxMessage.StdId=0x002;	 // ��׼��ʶ��Ϊ0
  TxMessage.IDE = CAN_ID_STD;			//ָ����Ҫ�������Ϣ�ı�ʶ��������
  TxMessage.RTR = CAN_RTR_DATA;		//ָ����֡�����������Ϣ������   ����֡��Զ��֡
  TxMessage.DLC = 2;			//ָ�����ݵĳ���
	TxMessage.Data[0] = 5;
	TxMessage.Data[1] = 6; 
	
  mbox = CAN_Transmit(CAN2, &TxMessage);
	
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}
