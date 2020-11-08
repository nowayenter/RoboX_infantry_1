#include "can1.h"

/*********�˴���ɹ���********
 *���������ݽṹ��
 *��ʼ��can
 *���忨�����˲�����
 *���յ�����ݲ������˲�
 *���巢�����ݺ���
 *����һ���ڻ�е�ǲ���
 ****************************/


M_Data motor_data[7];			//���������ݽṹ��
uint8_t cnt[5]={0,0,0,0,0};	//������
int16_t old_angle[7]={0};	//�ɽǶ�ֵ


void CAN1_Configuration(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef       GPIO_InitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		//��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);		//�������Ÿ��ù���
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//�����������
	GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;			//�ж����� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//��ռ���ȼ� 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//��Ӧ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN1);                        //����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
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
    CAN_Init(CAN1, &CAN_InitStructure);
	
		/******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;                  //������ 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��   

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//�����ж�  ���û����ָ����CANx�ж�  CAN_IT_FMP0:�ȴ��жϵ�FIFO 0��Ϣ
	//mc.m1=mc.m2=mc.m3=mc.m4=0;
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//�����ж�
}


//can�жϷ���������������
void CAN1_RX0_IRQHandler(void)
{
	//LED1 =! LED1;	//��ɫ������������յ��������
	CanRxMsg RxMessage;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x201:	//���̵��
			{
				motor_data[0].ActualSpeed = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[0].ActualSpeed = kalman(motor_data[0].ActualSpeed,0+4);//������ƽ������
				
				cnt[0]++;
				if(cnt[0]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[0] = 0;
					motor_data[0].OldAngle = motor_data[0].NowAngle;
					motor_data[0].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(0);												//��е��Խ�紦��
					motor_data[0].D_Angle = kalman(motor_data[0].D_Angle,0);	//��е���˲�
				}
				break;
			}
			case 0x202:	//���̵��
			{
				motor_data[1].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[1].ActualSpeed = kalman(motor_data[1].ActualSpeed,1+4);//������ƽ������
				
				cnt[1]++;
				if(cnt[1]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[1] = 0;
					motor_data[1].OldAngle = motor_data[1].NowAngle;
					motor_data[1].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(1);												//��е��Խ�紦��
					motor_data[1].D_Angle = kalman(motor_data[1].D_Angle,1);	//��е���˲�
				}
				break;
			}
			case 0x203:	//���̵��
			{
				motor_data[2].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[2].ActualSpeed = kalman(motor_data[2].ActualSpeed,2+4);//������ƽ������
				
				cnt[2]++;
				if(cnt[2]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[2] = 0;
					motor_data[2].OldAngle = motor_data[2].NowAngle;
					motor_data[2].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(2);												//��е��Խ�紦��
					motor_data[2].D_Angle = kalman(motor_data[2].D_Angle,2);	//��е���˲�
				}
				break;
			}
			case 0x204:	//���̵��
			{
				motor_data[3].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[3].ActualSpeed = kalman(motor_data[3].ActualSpeed,3+4);//������ƽ������
				
				cnt[3]++;
				if(cnt[3]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[3] = 0;
					motor_data[3].OldAngle = motor_data[3].NowAngle;
					motor_data[3].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(3);												//��е��Խ�紦��
					motor_data[3].D_Angle = kalman(motor_data[3].D_Angle,3);	//��е���˲�
				}
				break;
			}
			case 0x205://��̨���yaw��
			{
				motor_data[4].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor_data[4].Intensity = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				break;
			}
			case 0x206://��̨���pitch��
			{
				motor_data[5].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor_data[5].Intensity = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				break;
			}
			case 0x207://�������
			{
				motor_data[6].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[6].ActualSpeed = kalman(motor_data[6].ActualSpeed,4+4);//������ƽ������
				
				cnt[4]++;
				if(cnt[4]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[4] = 0;
					motor_data[6].OldAngle = motor_data[6].NowAngle;
					motor_data[6].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(6);												//��е��Խ�紦��
					motor_data[6].D_Angle = kalman(motor_data[6].D_Angle,9);	//��е���˲�
				}
			}

			default:
				break;
		}
	}	
}

//���͵��̵������
void CAN1_SendCommand_chassis(signed long ESC_201,signed long ESC_202,signed long ESC_203,signed long ESC_204)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;						//����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x200;				//����820r���ñ�ʶ��
	TxMessage.IDE = CAN_ID_STD;				//ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA;			//ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;						//ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	
	mbox=CAN_Transmit(CAN1,&TxMessage);   	//������Ϣ
	
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;	//�ȴ����ͽ���

}

//������̨�������
void CAN1_Send_Msg_gimbal(int16_t control_205,int16_t control_206, int16_t control_2006)   //can���� control_201 Y��  control_202 P��
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//����һ��������Ϣ�Ľṹ��
	
  TxMessage.StdId=0x1ff;	 // ��׼��ʶ��Ϊ0
  TxMessage.IDE = CAN_ID_STD;			//ָ����Ҫ�������Ϣ�ı�ʶ��������
  TxMessage.RTR = CAN_RTR_DATA;		//ָ����֡�����������Ϣ������   ����֡��Զ��֡
  TxMessage.DLC = 6;			//ָ�����ݵĳ���
	TxMessage.Data[0] = control_205>>8;
	TxMessage.Data[1] = control_205;
	TxMessage.Data[2] = control_206>>8;
	TxMessage.Data[3] = control_206;
	TxMessage.Data[4] = control_2006>>8;
	TxMessage.Data[5] = control_2006;
      
  mbox= CAN_Transmit(CAN1, &TxMessage);
	
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}



////////////////////////////////////////////////////////////////////////
/////////////////////////����Ϊ����Ԥ������////////////////////////////
////////////////////////////////////////////////////////////////////////


//������

float Q=600;					//Ԥ�ⷽ��
float R=5000;					//��������
float variance_kalman[10]={100,100,100,100,100,100,100,100,100,100};//����������ֵ�����ʼ��
float old_kalman[10];				//�ɿ���������ֵ

int16_t kalman(int16_t x,uint8_t i)			//�������˲�����x�����ݱ�ʶ
{
	//Ԥ��
	float state_pre = old_kalman[i];					//Ԥ��ֵΪ��һ������ֵ
	float variance_pre = variance_kalman[i] + Q;	//Ԥ��ֵ����Ϊ��һ������ֵ����+Ԥ�ⷽ��
	
	//���㿨��������
	float K = variance_pre / (variance_pre + R);
	
	//��ϲ���ֵ��Ԥ��ֵ����У��
	float state_kalman = state_pre + (K *(x-state_pre));	//����������ֵ����
	
	if(state_pre<0) state_pre = -state_pre;
	variance_kalman[i] = (1 - K) * state_pre;				//����������ֵ�������
	
	old_kalman[i] = state_kalman;							//���¾�����ֵ
	
	return state_kalman;								//�������������ֵ
}




//��е��Խ�紦��
void Angle_deal(int i)		//����iΪ�����ʶ��
{
	motor_data[i].D_Angle = motor_data[i].NowAngle - motor_data[i].OldAngle;
	
	if ((motor_data[i].ActualSpeed<5) && (motor_data[i].ActualSpeed>-5)) motor_data[i].ActualSpeed=0;
	
	if ((motor_data[i].ActualSpeed>=0) && (motor_data[i].D_Angle<-10))
	{
		motor_data[i].D_Angle += 8192;
	}
	else if ((motor_data[i].ActualSpeed<0) && (motor_data[i].D_Angle>10))
	{
		motor_data[i].D_Angle -= 8192; 
	}
	
	//�쳣����
	if ((motor_data[i].D_Angle - old_angle[i])>4000)
		motor_data[i].D_Angle -= 8192;
	else if ((motor_data[i].D_Angle - old_angle[i])<-4000)
		motor_data[i].D_Angle += 8192;
	else
		old_angle[i] = motor_data[i].D_Angle;
}


