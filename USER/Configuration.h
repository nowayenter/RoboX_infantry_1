/***********************************
 *全局配置文件
 *英雄车测试版 20180709
 ***********************************/
#ifndef __CONFIG_H
#define __CONFIG_H
//#define PI 	3.1415926f

/********************************************************************************************************************
 **********************************************用户层功能设置*********************************************************/

#define CHASSIS_POSITION_CRTL	//底盘位置环控制，注释掉时则仅使用速度环
#define PITCH_IMU_CTRL	//云台pitch轴的控制使用imu测量的绝对角度数据作反馈，注释掉则使用电机码盘的相对角度作反馈
#define YAW_ENCO_CTRL		//云台yaw轴使用电机码盘作反馈

#define PC_HIGH_SPEED_X		2500	//电脑按住shift控制时，高速模式的速度，前后,mm/s
#define PC_HIGH_SPEED_Y		2500	//电脑按住shift控制时，高速模式的速度，左右,mm/s
#define PC_LOW_SPEED_X		500	//电脑按住CTRL控制时，低速模式的速度，前后,mm/s
#define PC_LOW_SPEED_Y		500	//电脑按住CTRL控制时，低速模式的速度，左右,mm/s
#define PC_SPEED_X				1500	//电脑控制普通速度，前后
#define PC_SPEED_Y				1500	//电脑控制普通速度，左右

#define PC_MouseSpeed_X		1			//鼠标速度倍率
#define PC_MouseSpeed_Y		1			//鼠标速度倍率
#define REMOTE_Vx_MAX			2500	//遥控器控制最大速度，前后
#define REMOTE_Vy_MAX			2500	//遥控器控制最大速度，左右
#define REMOTE_Wz_MAX			PI		//遥控器控制最大速度，航向
#define REMOTE_Yaw_Angle	2050	//遥控器控制最大速度，航向
#define REMOTE_Pitch_Angle	25	//pitch轴最大角度
//20

/********************************************************************************************************************
 ***************************************************配置声明*********************************************************/

//*********************底盘机械参数*********************
#define MOTOR_P		19		//电机减速齿比
#define WHEEL_R		76		//轮子半径
#define CHASSIS_K	445		//K=0.5*(横轴长度+纵轴长度)mm

//******************3510电机极限性能********************
/*带19减速箱情况下实测得到
 ************************/
#define LIM_3510_SPEED	6880		//电调返回最大的速度值
#define LIM_3510_ANGLE	940			//每周期最大码值步长

//******************摇杆中位偏置************************
#define RC_ch0_Offset 0
#define RC_ch1_Offset 0
#define RC_ch2_Offset 0
#define RC_ch3_Offset 0
 
//****************云台电机配置声明********************
#define Yaw_MID					3400		//中点码值
#define Yaw_MAX					5423		//码值范围：中点+-1760
#define Yaw_MIN					1327
#define Pitch_MID				6850		//中点码值
#define Pitch_MAX				7450		//码值范围：中点+-1760
#define Pitch_MIN				6400
#define Yaw_Direction		 -1			//电机安装方向，1或-1
#define Pitch_Direction	 -1			//电机安装方向，1或-1

//****************拨盘电机配置声明********************
#define M2006_Direction	 -1			//电机安装方向，1或-1

//*******************pid频率声明**********************
#define PID_Hz 					500


/********************************************************************************************************************
 ***************************************************性能设置*********************************************************/

//******************最大速度、加速度*********************
#define Vx_MAX 				1890			// mm/s，应小于LIM_3510_27_Vx
//1280
//1890
#define Vy_MAX				1890			// mm/s，应小于LIM_3510_27_Vy
//1890
//1280
#define Wz_MAX 				PI				// rad/s,弧度每秒
#define X_ACC_MAX			1500			//最大加速度，即加速到X_ACC_MAX需要一秒钟
//2000
#define Y_ACC_MAX			1500			//最大加速度，即加速到Y_ACC_MAX需要一秒钟
//2000
//1000
#define Z_ACC_MAX			(4*PI)		//最大加速度，即加速到Z_ACC_MAX需要一秒钟

//****************底盘跟随云台模式相关**********************
#define GIM_FOLLOW_ANGLE			5				//底盘开始跟随云台的角度,正负5度
#define GIM_FOLLOW_ANGLE_D		10				//底盘开始跟随云台的角度补偿，与GIM_FOLLOW_ANGLE构成动态加速区间
																			//动态区间在GIM_FOLLOW_ANGLE+-(GIM_FOLLOW_ANGLE_D*Wz_MAX)内浮动
#define GIM_FOLLOW_BUFFERING	25			//缓冲角度，左右各30度
#define FLLOW_RATE						2.0f		//底盘以FLLOW_RATE倍速度追逐云台的
#define FLLOW_ACC							(2*PI)	//底盘跟随云台的最大加速度，即加速到FLLOW_ACC需要一秒钟

//****************底盘自动回中模式相关**********************
#define GIM_HIGH_SPEED_POS		20			//云台高速运动区间，度
#define GIM_HIGH_SPEED_RATE		2				//高速运动区间时的速度倍率

//*********************扭腰模式相关***********************
#define TWIST_ANGLE						10			//扭腰的角度
#define TWIST_SPEED						PI			//扭腰的速度
#define Twist_Vy_P	         	60      //Since the Twist will affect y axis of velocity, this parameter is used to implement this part


//*********************自动瞄准模式相关（高级项）***********************
#define VISION_X_Pixels				640	//x轴像素
#define VISION_Y_Pixels				480	//y轴像素
#define VISION_X_Offset				0		//x轴中点偏置
#define VISION_y_Offset				0		//y轴中点偏置


/********************************************************************************************************************
 ***************************************************参数设置*********************************************************/

//****************底盘电机pid参数************************
//速度环pid
#define PID_MOTOR_SPEED_KP			(3.8f)
#define PID_MOTOR_SPEED_KI			(0.0f) //0.2
#define PID_MOTOR_SPEED_KD			(3.5f)
#define PID_MOTOR_SPEED_I_MAX		(1500)	//积分最大值
#define PID_MOTOR_SPEED_I_Err		(500)		//积分分离值，差值在此值以内才进行积分
//位置环pid
#define PID_MOTOR_ANGLE_KP			(0.02f)
#define PID_MOTOR_ANGLE_KI			(0.0f)
#define PID_MOTOR_ANGLE_KD			(0.0f)

//****************云台电机pid参数************************
//Pitch轴角速度环
#define PID_PITCH_SPEED_KP		(20.0f)
//100
#define PID_PITCH_SPEED_KI		(0.5f)
//1.0
//0.5
#define PID_PITCH_SPEED_KD		(700.0f)
//1000
#define PID_PITCH_SPEED_I_MAX	(800)	//积分最大值
#define PID_PITCH_SPEED_I_Err	(100)	//积分分离值，差值在此值以内才进行积分
//Pitch轴角度环
#define PID_PITCH_ANGLE_KP		(20.0f)
//20
//17
#define PID_PITCH_ANGLE_KI		(0.0f)
#define PID_PITCH_ANGLE_KD		(0)
#define PID_PITCH_ANGLE_I_MAX	(50)	//积分最大值
#define PID_PITCH_ANGLE_I_Err	(10)	//积分分离值，差值在此值以内才进行积分

//////Yaw轴角速度环
//#define PID_YAW_SPEED_KP			(500.0f)
////150
//#define PID_YAW_SPEED_KI			(0.5f)
////0.5
//#define PID_YAW_SPEED_KD			(1000.0f)
////2000
//#define PID_YAW_SPEED_I_MAX		(400)	//积分最大值
//#define PID_YAW_SPEED_I_Err		(100)	//积分分离值，差值在此值以内才进行积分

//Yaw轴角速度环
#define PID_YAW_SPEED_KP			(16.0f)
//30
//15
#define PID_YAW_SPEED_KI			(0.5f)
//0.1
//0.5
#define PID_YAW_SPEED_KD			(700.0f)
//975
//950
//1000
//900
#define PID_YAW_SPEED_I_MAX		(400)	//积分最大值
#define PID_YAW_SPEED_I_Err		(100)	//积分分离值，差值在此值以内才进行积分

//Yaw轴角度环
#define PID_YAW_ANGLE_KP			(18.0f)
//28
//20
#define PID_YAW_ANGLE_KI			(0.0f)
#define PID_YAW_ANGLE_KD			(0.0f)
#define PID_YAW_ANGLE_I_MAX		(50)	//积分最大值
#define PID_YAW_ANGLE_I_Err		(10)	//积分分离值，差值在此值以内才进行积分


//拨弹电机
#define PID_FIRE_SPEED_KP			(3.8f)
#define PID_FIRE_SPEED_KI			(0.8f)
#define PID_FIRE_SPEED_KD			(2.5f)
#define PID_FIRE_SPEED_I_MAX	(50)	//积分最大值

//自动瞄准
#define VISION_X_KP						(0.01f)
#define VISION_X_KI						(0)
#define VISION_X_KD						(0)
#define VISION_Y_KP						(0)
#define VISION_Y_KI						(0)
#define VISION_Y_KD						(0)


//***********************IMU参数************************
#define IMU_TEMP				9000		//目标温度
#define TEMP_Tolerance	60			//温度误差容忍度
#define TEMP_KP					2.4f
#define TEMP_KI					1
#define TEMP_KD					1

//***********************蜂鸣器参数************************
#define BUZZER_TIME			300			//蜂鸣器工作一次的时间长度，ms



//***********************摩擦轮2312参数************************
#define Snail_Direction 	1
#define Fric_UP   				1400
#define Fric_MID					1300
#define Fric_RC						1000+abs(RC_Ctl.rc.ch3-1024)*0.5f//遥控器
#define Fric_DOWN 				1250
#define Fric_OFF  				1000

#endif

