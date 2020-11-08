#include "can1.h"

/*********此处完成功能********
 *定义电机数据结构体
 *初始化can
 *定义卡尔曼滤波函数
 *接收电调数据并进行滤波
 *定义发送数据函数
 *计算一周期机械角步长
 ****************************/


M_Data motor_data[7];			//定义电机数据结构体
uint8_t cnt[5]={0,0,0,0,0};	//计数用
int16_t old_angle[7]={0};	//旧角度值


void CAN1_Configuration(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef       GPIO_InitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		//打开GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);		//配置引脚复用功能
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//复用推挽输出
	GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;			//中断配置 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN1);                        //将外围寄存器初始化到它们的默认重置值
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
    CAN_Init(CAN1, &CAN_InitStructure);
	
		/******CAN总线的过滤配置(接收配置)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;                  //过滤器 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化   

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//接收中断  启用或禁用指定的CANx中断  CAN_IT_FMP0:等待中断的FIFO 0消息
	//mc.m1=mc.m2=mc.m3=mc.m4=0;
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//发送中断
}


//can中断服务函数，接受数据
void CAN1_RX0_IRQHandler(void)
{
	//LED1 =! LED1;	//绿色灯闪，代表接收到电机数据
	CanRxMsg RxMessage;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x201:	//底盘电机
			{
				motor_data[0].ActualSpeed = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[0].ActualSpeed = kalman(motor_data[0].ActualSpeed,0+4);//卡尔曼平滑处理
				
				cnt[0]++;
				if(cnt[0]==Hz_Flag)//电调每返回信息n次则计算机械角一次
				{
					cnt[0] = 0;
					motor_data[0].OldAngle = motor_data[0].NowAngle;
					motor_data[0].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(0);												//机械角越界处理
					motor_data[0].D_Angle = kalman(motor_data[0].D_Angle,0);	//机械角滤波
				}
				break;
			}
			case 0x202:	//底盘电机
			{
				motor_data[1].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[1].ActualSpeed = kalman(motor_data[1].ActualSpeed,1+4);//卡尔曼平滑处理
				
				cnt[1]++;
				if(cnt[1]==Hz_Flag)//电调每返回信息n次则计算机械角一次
				{
					cnt[1] = 0;
					motor_data[1].OldAngle = motor_data[1].NowAngle;
					motor_data[1].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(1);												//机械角越界处理
					motor_data[1].D_Angle = kalman(motor_data[1].D_Angle,1);	//机械角滤波
				}
				break;
			}
			case 0x203:	//底盘电机
			{
				motor_data[2].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[2].ActualSpeed = kalman(motor_data[2].ActualSpeed,2+4);//卡尔曼平滑处理
				
				cnt[2]++;
				if(cnt[2]==Hz_Flag)//电调每返回信息n次则计算机械角一次
				{
					cnt[2] = 0;
					motor_data[2].OldAngle = motor_data[2].NowAngle;
					motor_data[2].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(2);												//机械角越界处理
					motor_data[2].D_Angle = kalman(motor_data[2].D_Angle,2);	//机械角滤波
				}
				break;
			}
			case 0x204:	//底盘电机
			{
				motor_data[3].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[3].ActualSpeed = kalman(motor_data[3].ActualSpeed,3+4);//卡尔曼平滑处理
				
				cnt[3]++;
				if(cnt[3]==Hz_Flag)//电调每返回信息n次则计算机械角一次
				{
					cnt[3] = 0;
					motor_data[3].OldAngle = motor_data[3].NowAngle;
					motor_data[3].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(3);												//机械角越界处理
					motor_data[3].D_Angle = kalman(motor_data[3].D_Angle,3);	//机械角滤波
				}
				break;
			}
			case 0x205://云台电机yaw轴
			{
				motor_data[4].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor_data[4].Intensity = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				break;
			}
			case 0x206://云台电机pitch轴
			{
				motor_data[5].NowAngle = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				motor_data[5].Intensity = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				break;
			}
			case 0x207://拨弹电机
			{
				motor_data[6].ActualSpeed=(RxMessage.Data[2]<<8)+RxMessage.Data[3];
				motor_data[6].ActualSpeed = kalman(motor_data[6].ActualSpeed,4+4);//卡尔曼平滑处理
				
				cnt[4]++;
				if(cnt[4]==Hz_Flag)//电调每返回信息n次则计算机械角一次
				{
					cnt[4] = 0;
					motor_data[6].OldAngle = motor_data[6].NowAngle;
					motor_data[6].NowAngle=(RxMessage.Data[0]<<8)+RxMessage.Data[1];
					Angle_deal(6);												//机械角越界处理
					motor_data[6].D_Angle = kalman(motor_data[6].D_Angle,9);	//机械角滤波
				}
			}

			default:
				break;
		}
	}	
}

//发送底盘电机数据
void CAN1_SendCommand_chassis(signed long ESC_201,signed long ESC_202,signed long ESC_203,signed long ESC_204)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;						//定义一个发送信息的结构体
	
	TxMessage.StdId = 0x200;				//根据820r设置标识符
	TxMessage.IDE = CAN_ID_STD;				//指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA;			//指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 8;						//指定数据的长度
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	
	mbox=CAN_Transmit(CAN1,&TxMessage);   	//发送信息
	
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;	//等待发送结束

}

//发送云台电机数据
void CAN1_Send_Msg_gimbal(int16_t control_205,int16_t control_206, int16_t control_2006)   //can发送 control_201 Y轴  control_202 P轴
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;				//定义一个发送信息的结构体
	
  TxMessage.StdId=0x1ff;	 // 标准标识符为0
  TxMessage.IDE = CAN_ID_STD;			//指定将要传输的消息的标识符的类型
  TxMessage.RTR = CAN_RTR_DATA;		//指定的帧将被传输的消息的类型   数据帧或远程帧
  TxMessage.DLC = 6;			//指定数据的长度
	TxMessage.Data[0] = control_205>>8;
	TxMessage.Data[1] = control_205;
	TxMessage.Data[2] = control_206>>8;
	TxMessage.Data[3] = control_206;
	TxMessage.Data[4] = control_2006>>8;
	TxMessage.Data[5] = control_2006;
      
  mbox= CAN_Transmit(CAN1, &TxMessage);
	
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
}



////////////////////////////////////////////////////////////////////////
/////////////////////////以下为数据预处理函数////////////////////////////
////////////////////////////////////////////////////////////////////////


//卡尔曼

float Q=600;					//预测方差
float R=5000;					//测量方差
float variance_kalman[10]={100,100,100,100,100,100,100,100,100,100};//卡尔曼最优值方差初始化
float old_kalman[10];				//旧卡尔曼最优值

int16_t kalman(int16_t x,uint8_t i)			//输入需滤波数据x、数据辨识
{
	//预测
	float state_pre = old_kalman[i];					//预测值为上一次最优值
	float variance_pre = variance_kalman[i] + Q;	//预测值方差为上一次最优值方差+预测方差
	
	//计算卡尔曼增益
	float K = variance_pre / (variance_pre + R);
	
	//结合测量值对预测值进行校正
	float state_kalman = state_pre + (K *(x-state_pre));	//卡尔曼最优值计算
	
	if(state_pre<0) state_pre = -state_pre;
	variance_kalman[i] = (1 - K) * state_pre;				//卡尔曼最优值方差计算
	
	old_kalman[i] = state_kalman;							//更新旧最优值
	
	return state_kalman;								//输出卡尔曼最优值
}




//机械角越界处理
void Angle_deal(int i)		//输入i为电机标识符
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
	
	//异常处理
	if ((motor_data[i].D_Angle - old_angle[i])>4000)
		motor_data[i].D_Angle -= 8192;
	else if ((motor_data[i].D_Angle - old_angle[i])<-4000)
		motor_data[i].D_Angle += 8192;
	else
		old_angle[i] = motor_data[i].D_Angle;
}


