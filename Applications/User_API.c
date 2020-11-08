/********************************************************************
 *用户接口层
 *
 *整车状态机设置
 *模式切换
 *
 *参数配置在Configuration.h文件内
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

ControlDATA_TypeDef Control_data;//控制数据
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

//状态机
u8 TrashyTime_Flag = 1;	//垃圾时间，开局时的垃圾时间
u8 Twist_Flag = 0;			//扭腰
u8 Vision_Flag = 0;			//视觉自瞄
u8 ShootBuff_Flag = 0;	//打符
u8 Shoot_Flag = 0;			//射击
u8 Shoot_Motor = 0;			//摩擦轮
u8 Chassis_mode = 0;		//底盘模式，0自动回中，1无头模式
u8 TwistCnt,BuffCnt,ShootMotorCnt;//计数用

//射击方案(最大性能)
ShootProject_TypeDef ShootProject_TrashyTime={18,21};	//垃圾时间
ShootProject_TypeDef ShootProject_Level1   = {2,21};	//一级模式
ShootProject_TypeDef ShootProject_Level2   = {2,21};	//二级模式
ShootProject_TypeDef ShootProject_Level3   = {2,21};	//三级模式
ShootProject_TypeDef ShootProject_Pillbox  = {10,10};	//碉堡模式增益
ShootProject_TypeDef ShootProject_Gate		 = {10,10};	//桥头/关口增益
ShootProject_TypeDef SmallBuff						 = {10,10};	//小符增益
ShootProject_TypeDef BigBuff						   = {10,10};	//大符增益

u16 Shoot_Frequency;		//射频

//demo
float old_output[4];
void Chassis_Task(void)//底盘控制
{
	Astrict_Acc(Control_data.Vx,Control_data.Vy,Control_data.Wz);//加速度限制
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//底盘控制接口
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
														motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}

void Stop_Chassis_Task(void)
{
	Astrict_Acc(0, 0, 0);//加速度限制
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//底盘控制接口
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
														motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}


/****************************************************************
 *实现函数：Get_Ctrl_Data(void)
 *功    能：将遥控信号转换为控制信号，控制信号储存在Control_data中
 *说    明：电脑/遥控器切换：右键最上电脑控制
 *          电脑 ：扭腰（F），自瞄（鼠标左键 按住），打符（R）
 *          遥控器：扭腰左键最上
 ****************************************************************/
void Get_Ctrl_Data(void)
{
	if(RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_UP)//遥控器 左上 右中 底盘控制
	{
		Control_data.Vx = -(float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Vx_MAX;
		Control_data.Vy = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Vy_MAX;
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;	//角速度，单位rad/s
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Pitch_Angle;
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
	}
	else if(RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_MID)//遥控器 左中 右中 云台+底盘平移
	{
		Control_data.Vx = -(float)(RC_Ctl.rc.ch0 - 1024 + RC_ch0_Offset)/660*REMOTE_Vx_MAX;
		Control_data.Vy = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Vy_MAX;
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;	//角速度，单位rad/s
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch3 - 1024 + RC_ch3_Offset)/660*REMOTE_Pitch_Angle;
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		
	}
	
	else if (RC_Ctl.rc.s2 == RC_SW_DOWN && RC_Ctl.rc.s1 == RC_SW_DOWN){//遥控器 左下 右中 射击+云台		
		Control_data.Wz = (float)(RC_Ctl.rc.ch2 - 1024 + RC_ch2_Offset)/660*REMOTE_Wz_MAX;
		
//		if (Control_data.Yaw_angle > 30)	Control_data.Yaw_angle = 30;
//		else if (Control_data.Yaw_angle < -30)	Control_data.Yaw_angle = -30;
		
		Control_data.Pitch_angle = (float)(RC_Ctl.rc.ch1 - 1024 + RC_ch1_Offset)/660*REMOTE_Pitch_Angle;
		
		if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
		else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
	}
	
	else if(RC_Ctl.rc.s2 == RC_SW_UP)//电脑
	{
		if(RC_Ctl.key.Shift == 1){//高速
			Control_data.Vx = PC_HIGH_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_HIGH_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * PC_MouseSpeed_X * RC_Ctl.mouse.x;
			Control_data.Pitch_angle -= 0.01 * PC_MouseSpeed_Y * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
			else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		}
		else if(RC_Ctl.key.Ctrl == 1)//低速
		{
			Control_data.Vx = PC_LOW_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_LOW_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * PC_MouseSpeed_X * RC_Ctl.mouse.x;
			Control_data.Pitch_angle -= 0.01 * PC_MouseSpeed_Y * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 25)	Control_data.Pitch_angle = 25;
			else if (Control_data.Pitch_angle < -25)	Control_data.Pitch_angle = -25;
		}
		else//普通
		{
			Control_data.Vx = PC_SPEED_X * RC_Ctl.key.Vertical;
			Control_data.Vy = PC_SPEED_Y * RC_Ctl.key.Horizontal;
			Control_data.Wz = 0.05 * RC_Ctl.mouse.x;

			Control_data.Pitch_angle -= 0.005 * RC_Ctl.mouse.y;
			
			if (Control_data.Pitch_angle > 30)	Control_data.Pitch_angle = 30;
			if (Control_data.Pitch_angle < -10)	Control_data.Pitch_angle = -10;
		}
	}

//		//底盘升降
//		if(chassiscnt > 100)  
//		{
//			if(RC_Ctl.key.F == 1)
//			{
//				Chassis_Flag = !Chassis_Flag;
//				chassiscnt = 0;
//			}
//		}else chassiscnt++;
//		
//		//取弹模式
//		if(armcnt > 100)  
//		{
//			if(RC_Ctl.key.R == 1)
//			{
//				arm_ctrl();
//				armcnt = 0;
//			}
//		}else armcnt++;
//		
//		//步兵补弹
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
// 扭腰模式开启		
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
//	//扭腰时动作变慢
//	if(Twist_Flag == 1)
//	{
//		Control_data.Vx *= 0.5f;
//		Control_data.Vy *= 0.5f;
//		Control_data.Wz *= 0.5f;
//	}


/****************************************************************
 *实现函数：Shoot_Ctrl(void)
 *功    能：射速、射频控制
 *说    明：根据状态机、步兵状态、buff加成等控制射速、射频
 *         最终的射频输出值储存在Shoot_Frequency中
 ****************************************************************/

void Shoot_Ctrl(void)
{
	float ShootSpeed;
	float shooterHeat_Max;//热量上限
	if(Shoot_Flag == 0)//不射击的时候
	{
		Shoot_Frequency = 0;
		return;
	}
	/***决策射击方案***/
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
	
	
	/***场地加成***/
	if(Umpire_Buff.buffMusk&(0x0001<<9) == 1)//碉堡模式
	{
		ShootSpeed += ShootProject_Pillbox.Speed;
		Shoot_Frequency += ShootProject_Pillbox.Frequency;
	}
	else if (Umpire_Buff.buffMusk&(0x0001<<8) == 1)//加速冷却（关口位置）
	{
		ShootSpeed += ShootProject_Gate.Speed;
		Shoot_Frequency += ShootProject_Gate.Frequency;
	}
	/***大小幅加成***/
	if (Umpire_Buff.buffMusk&(0x0001<<4) == 1)//大buff
	{
		ShootSpeed += BigBuff.Speed;
		Shoot_Frequency += BigBuff.Frequency;
	}
	else if (Umpire_Buff.buffMusk&(0x0001<<6) == 1)//小buff
	{
		ShootSpeed += SmallBuff.Speed;
		Shoot_Frequency += SmallBuff.Frequency;
	}
	
	/***输出限制***/
	if (ShootSpeed > 16)
		ShootSpeed = 16;//射速限制
	if ((shooterHeat_Max-Umpire_PowerHeat.shooterHeat0) < (ShootSpeed))
		Shoot_Frequency = 0;//剩余热量不足停止射击
	
//	PWM_Write(PWM1_CH2,1400+(ShootSpeed-17)*20.0f);//摩擦轮
//	PWM_Write(PWM1_CH1,1400+(ShootSpeed-17)*20.0f);//摩擦轮
	
	printfBuff1 = Umpire_Shoot.bulletSpeed;
	printfBuff2 = Umpire_PowerHeat.shooterHeat0;
}


