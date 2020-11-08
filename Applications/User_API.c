/********************************************************************
 *�û��ӿڲ�
 *
 *����״̬������
 *ģʽ�л�
 *
 *����������Configuration.h�ļ���
 ********************************************************************/
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "IMU.h"
#include "MPU6500.h"
#include "Configuration.h"
#include "led.h"
#include "buzzer.h"
#include "RemoteControl.h"
#include "Comm_umpire.h"
#include "can1.h"
#include "Higher_Class.h"
#include "PID.h"
#include "pwm.h"
#include "Shoot.h"
#include "fric.h"

ControlDATA_TypeDef Control_data;//��������
u8 View_Flag = 0;
u8 Bullet_Flag = 0;
u8 Chassis_Flag = 0;
u8 Left_Flag = 0;
u8 Right_Flag = 0;
u8 viewcnt;
u8 bulletcnt;
u8 chassiscnt;
u8 armcnt;
u8 leftcnt;
u8 rightcnt;

//״̬��
u8 TrashyTime_Flag = 1;	//����ʱ�䣬����ʱ������ʱ��
u8 Twist_Flag = 0;			//Ť��
u8 Vision_Flag = 0;			//�Ӿ�����
u8 ShootBuff_Flag = 0;	//���
u8 Shoot_Flag = 0;			//���
u8 Shoot_Motor = 0;			//Ħ����
u8 Chassis_mode = 0;		//����ģʽ��0�Զ����У�1��ͷģʽ
u8 TwistCnt,BuffCnt,ShootMotorCnt;//������

//�������(�������)
ShootProject_TypeDef ShootProject_TrashyTime={18,21};	//����ʱ��
ShootProject_TypeDef ShootProject_Level1   = {2,21};	//һ��ģʽ
ShootProject_TypeDef ShootProject_Level2   = {2,21};	//����ģʽ
ShootProject_TypeDef ShootProject_Level3   = {2,21};	//����ģʽ
ShootProject_TypeDef ShootProject_Pillbox  = {10,10};	//�ﱤģʽ����
ShootProject_TypeDef ShootProject_Gate		 = {10,10};	//��ͷ/�ؿ�����
ShootProject_TypeDef SmallBuff						 = {10,10};	//С������
ShootProject_TypeDef BigBuff						   = {10,10};	//�������

u16 Shoot_Frequency;		//��Ƶ

//demo
float old_output[4];
void Chassis_Task(void)//���̿���
{
	Astrict_Acc(Control_data.Vx,Control_data.Vy,Control_data.Wz);//���ٶ�����
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//���̿��ƽӿ�
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
														motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}

void Stop_Chassis_Task(void)
{
	Astrict_Acc(0, 0, 0);//���ٶ�����
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//���̿��ƽӿ�
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
														motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}


/****************************************************************
 *ʵ�ֺ�����Get_Ctrl_Data(void)
 *��    �ܣ���ң���ź�ת��Ϊ�����źţ������źŴ�����Control_data��
 *˵    ��������/ң�����л����Ҽ����ϵ��Կ���
 *          ���� ��Ť����F�������飨������ ��ס���������R��
 *          ң������Ť���������
 ****************************************************************/
void Get_Ctrl_Data(void)
{
	if(RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_UP)//ң���� ���� ���� ���̿���
	{
		Control_data.Vx = -(float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Vx_MAX;
		Control_data.Vy = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Vy_MAX;
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;	//���ٶȣ���λrad/s
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Pitch_Angle;
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
	}
	else if(RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_MID)//ң���� ���� ���� ��̨+����ƽ��
	{
		Control_data.Vx = -(float)(RC_Ctl.rc.ch0 - 1024 + RC_ch0_Offset)/660*REMOTE_Vx_MAX;
		Control_data.Vy = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Vy_MAX;
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;	//���ٶȣ���λrad/s
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Pitch_Angle;
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		
	}
	
	else if (RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_DOWN){//ң���� ���� ���� ���+��̨		
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;
		
//		if (Control_data.Yaw_angle > 30)	Control_data.Yaw_angle = 30;
//		else if (Control_data.Yaw_angle < -30)	Control_data.Yaw_angle = -30;
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Pitch_Angle;
		
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
	}
	
	else if(RC_Ctl.rc.s2 == RC_SW_UP)//����
	{
		if(RC_Ctl.key.Shift == 1){//����
			Control_data.Vx = PC_HIGH_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_HIGH_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * PC_MouseSpeed_X * RC_Ctl.mouse.x;
			Control_data.Pitch_angle -= 0.01 * PC_MouseSpeed_Y * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
			else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		}
		else if(RC_Ctl.key.Ctrl == 1)//����
		{
			Control_data.Vx = PC_LOW_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_LOW_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * PC_MouseSpeed_X * RC_Ctl.mouse.x;
			Control_data.Pitch_angle -= 0.01 * PC_MouseSpeed_Y * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
			else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		}
		else//��ͨ
		{
			Control_data.Vx = PC_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * RC_Ctl.mouse.x;

			Control_data.Pitch_angle -= 0.005 * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 30)	Control_data.Pitch_angle = 30;
			if (Control_data.Pitch_angle < -10)	Control_data.Pitch_angle = -10;
		}
	}

//		//��������
//		if(chassiscnt > 100)  
//		{
//			if(RC_Ctl.key.F == 1)
//			{
//				Chassis_Flag = !Chassis_Flag;
//				chassiscnt = 0;
//			}
//		}else chassiscnt++;
//		
//		//ȡ��ģʽ
//		if(armcnt > 100)  
//		{
//			if(RC_Ctl.key.R == 1)
//			{
//				arm_ctrl();
//				armcnt = 0;
//			}
//		}else armcnt++;
//		
//		//��������
//		if(bulletcnt > 100)  
//		{
//			if(RC_Ctl.key.G == 1)
//			{
//				Bullet_Flag = !Bullet_Flag;
//				bulletcnt = 0;
//			}
//		}else bulletcnt++;
//		
//		if(leftcnt > 100)  
//		{
//			if(RC_Ctl.key.Q == 1)
//			{
//				Left_Flag = !Left_Flag;
//				leftcnt = 0;
//			}
//		}else leftcnt++;
//		
//		if(rightcnt > 100)  
//		{
//			if(RC_Ctl.key.E == 1)
//			{
//				Right_Flag = !Right_Flag;
//				rightcnt = 0;
//			}
//		}else rightcnt++;		
//		
//		if(RC_Ctl.mouse.press_l == 1)
//		{
//			Shoot_Flag = 1;
////			Gimbal_Ctrl2(3,60);
//		}
//		else 
//		{
//			Shoot_Flag = 0;
//		}

//		chassis_ctrl(Chassis_Flag);
//		bullet_ctrl(Bullet_Flag);
//		left_chassis(Left_Flag);
//		right_chassis(Right_Flag);		
	}
	

		
			
//		else
// Ť��ģʽ����		
//		if(modecnt > 100)  
//		{
//			if(RC_Ctl.key.F == 1)
//			{
//				Twist_Flag = !Twist_Flag;
//				modecnt = 0;
//			}
//		}else modecnt++;
//	}
//	
//	
//	//Ť��ʱ��������
//	if(Twist_Flag == 1)
//	{
//		Control_data.Vx *= 0.5f;
//		Control_data.Vy *= 0.5f;
//		Control_data.Wz *= 0.5f;
//	}


/****************************************************************
 *ʵ�ֺ�����Shoot_Ctrl(void)
 *��    �ܣ����١���Ƶ����
 *˵    ��������״̬��������״̬��buff�ӳɵȿ������١���Ƶ
 *         ���յ���Ƶ���ֵ������Shoot_Frequency��
 ****************************************************************/

void Shoot_Ctrl(void)
{
	float ShootSpeed;
	float shooterHeat_Max;//��������
	if(Shoot_Flag == 0)//�������ʱ��
	{
		Shoot_Frequency = 0;
		return;
	}
	/***�����������***/
	if (Umpire_State.robotLevel == 1)
	{
		ShootSpeed = ShootProject_Level1.Speed;
		Shoot_Frequency = ShootProject_Level1.Frequency;
		shooterHeat_Max = 80;
	}
	else if (Umpire_State.robotLevel == 2)
	{
		ShootSpeed = ShootProject_Level2.Speed;
		Shoot_Frequency = ShootProject_Level2.Frequency;
		shooterHeat_Max = 120;
	}
	else if (Umpire_State.robotLevel == 3)
	{
		ShootSpeed = ShootProject_Level3.Speed;
		Shoot_Frequency = ShootProject_Level3.Frequency;
		shooterHeat_Max = 200;
	}
	
	
	/***���ؼӳ�***/
	if(Umpire_Buff.buffMusk&(0x0001<<9) == 1)//�ﱤģʽ
	{
		ShootSpeed += ShootProject_Pillbox.Speed;
		Shoot_Frequency += ShootProject_Pillbox.Frequency;
	}
	else if (Umpire_Buff.buffMusk&(0x0001<<8) == 1)//������ȴ���ؿ�λ�ã�
	{
		ShootSpeed += ShootProject_Gate.Speed;
		Shoot_Frequency += ShootProject_Gate.Frequency;
	}
	/***��С���ӳ�***/
	if (Umpire_Buff.buffMusk&(0x0001<<4) == 1)//��buff
	{
		ShootSpeed += BigBuff.Speed;
		Shoot_Frequency += BigBuff.Frequency;
	}
	else if (Umpire_Buff.buffMusk&(0x0001<<6) == 1)//Сbuff
	{
		ShootSpeed += SmallBuff.Speed;
		Shoot_Frequency += SmallBuff.Frequency;
	}
	
	/***�������***/
	if (ShootSpeed > 16)
		ShootSpeed = 16;//��������
	if ((shooterHeat_Max-Umpire_PowerHeat.shooterHeat0) < (ShootSpeed))
		Shoot_Frequency = 0;//ʣ����������ֹͣ���
	
//	PWM_Write(PWM1_CH2,1400+(ShootSpeed-17)*20.0f);//Ħ����
//	PWM_Write(PWM1_CH1,1400+(ShootSpeed-17)*20.0f);//Ħ����
	
	printfBuff1 = Umpire_Shoot.bulletSpeed;
	printfBuff2 = Umpire_PowerHeat.shooterHeat0;
}


