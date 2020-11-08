//#include "Ctrl_gimbal.h"
//#include "MPU6500.h"
//#include "IMU.h"
//#include "PID.h"
//#include "can1.h"
//#include "Configuration.h"
//#include "User_Api.h"
//#include <math.h>
//#include <stdlib.h>

//float printfBuff1,printfBuff2;

//float Yaw_EncodeAngle,Pitch_EncodeAngle;//使用码盘计算出来的云台角度
//float E_yaw;//未校准的yaw角度值，需要在启动云台控制前赋值E_yaw=imu.yaw


////实现函数：pitch轴电机控制
////输入：目标角度，角度反馈数据
//void Pitch_pid(float Target_angle,float Measure_angle)
//{
//	//57.2957795=180/pi
//	float gryoPitch = -(imu.gyro[1]-imu.gyroOffset[1])*57.2957795f;
//	
//	PID_Control(&Pitch_angle_pid,Target_angle,Measure_angle);
////	if(fabs(Pitch_angle_pid.Err)<0.5) Pitch_angle_pid.Integrate = 0;		//达到目标时积分归零
//	PID_Control(&Pitch_speed_pid,Pitch_angle_pid.Control_OutPut,gryoPitch);
//}



///*********************************************************************
// *实现函数：void virtual_encoders(int16_t aim[4])
// *功    能：计算虚拟码盘
// *输    入：控制器目标值
// *说    明：
// ********************************************************************/
//int16_t Encoders_shoot;		//虚拟码盘值
//void virtual_encoders_shoot(int16_t target)
//{
//	Encoders_shoot += target;
//	if (Encoders_shoot>8191) Encoders_shoot -= 8192;
//	else if (Encoders_shoot<0) Encoders_shoot += 8192;
//}

///*********************************************************************
// *实现函数：int encoders_err(int error)
// *功    能：计算实际目标码盘与目标虚拟码盘差值
// *输    入：电机识别符，周期差值
// *返    回：实际目标码盘与目标虚拟码盘差值
// *说    明：
// ********************************************************************/

//int encoders_err_shoot(int error)
//{
//	int temp;
//	
//	temp = motor_data[6].NowAngle + error;
//	temp = temp%8192;	
//	if (temp<0) temp += 8192;						//实际目标码盘值
//	temp = Encoders_shoot - temp;
//	if (temp<-3000) temp += 8192;
//	else if (temp>3000) temp -= 8192;
//	
//	return temp;
//}

///*********************************************************************
// *实现函数：void motor_angle_pid(int16_t TargetAngle)
// *功    能：电机位置控制
// *输    入：电机每周期的目标码值
// *说    明：使用串级pid，内环为速度环
// ********************************************************************/
//#define ANGLE_KP (0.26f)//角度环参数
//#define ANGLE_KI (0.04f)
//#define ANGLE_KD (1.0f)
//#define SPEED_KP (2.4f)//速度环参数
//#define SPEED_KI (0.2f)
//#define SPEED_KD (2.0f)
//int16_t pid_out;
//static float angle_i=0,speed_i=0;
//static int16_t last_speed = 0;
//static int old_target = 0;
//void motor_angle_pid(int16_t TargetAngle)
//{
//	/**********位置环***********/
//	float P,I,D;									//中间值归零
//	int error;

//	virtual_encoders_shoot(TargetAngle);
//	
//	error = TargetAngle;											//角度差值
//	error += old_target - motor_data[6].D_Angle;	//累加上一次未完成差值
//	error += encoders_err_shoot(error);		//累加实际码盘值与虚拟码盘值的差
//	printfBuff1 = error;
//	
//	old_target = error;				//更新旧目标值
//		
//	P = ANGLE_KP * error;
//	angle_i += error;
//	if(angle_i > 1000) angle_i = 1000;
//	if(angle_i <- 1000) angle_i = -1000;
//	I = ANGLE_KI * angle_i;
//	D = ANGLE_KD * motor_data[6].ActualSpeed;
//	
//	pid_out = P + I - D;
//		
//	if (pid_out > 10000) pid_out = 10000;			//位置环输出(目标转速)限幅
//	else if(pid_out < -10000) pid_out = -10000;
//	
//	if ((pid_out<70) && (pid_out>-70)) pid_out = 0;
//	
//	/**********速度环***********/
//	error = pid_out - motor_data[6].ActualSpeed;		//速度偏差值
//	P = SPEED_KP * error;
//	
//	if((error < 500)&&(error > -500)) speed_i += error;
//	if (speed_i>1000) speed_i=1000;		//积分限幅
//	if (speed_i<-1000) speed_i=-1000;
//	I = SPEED_KI * speed_i;
//	
//	D = SPEED_KD * (motor_data[6].ActualSpeed - last_speed);
//	last_speed = motor_data[6].ActualSpeed;
//	
//	pid_out = P+I-D;
//	pid_out += 0.4 * pid_out;
//	CAN1_Send_Msg_gimbal(0, 0, pid_out);
//}

////实现函数：Yaw_angle_pid(float aim)
////功    能：利用mpu6500控制云台yaw轴角度
////输    入：每周期转动的角度
////注			：如500HZ速度运行，每周期转动0.5度，则1s转动250度
//static float T_yaw;		//目标角度
//static float Yaw_AnglePid_i;//角度环积分项
//void Yaw_angle_pid(float Targrt_d_angle)
//{
//	//57.2957795=180/pi
//	float gryoYaw = (imu.gyro[2]-imu.gyroOffset[2])*57.2957795f;//角速度
//	float angleYaw = imu.yaw - E_yaw;//测量角度，需用上电时的角度校准imu测量角度零点
//	
//	float error=0;
//	float P=0,I=0,D=0;
//	float angle_output=0;
//	
//	T_yaw -= Targrt_d_angle;		//计算目标角度
//	if (T_yaw > 180)
//	{
//		T_yaw = T_yaw - 360;
//	}
//	else if (T_yaw < -180)
//	{
//		T_yaw = T_yaw + 360;
//	}
//	
//	//************外环，角度环pi************
//	error = T_yaw - angleYaw;
//	if (error < -170)
//	{
//		error = error + 360;
//	}
//	else if (error > 170)
//	{
//		error = error - 360;
//	}
//	
//	P = error * PID_YAW_ANGLE_KP;
//	
//	if((error<10)&&(error>-10))
//		Yaw_AnglePid_i += error;
//	//积分限幅
//	if(Yaw_AnglePid_i > 50)	Yaw_AnglePid_i = 50;
//	if(Yaw_AnglePid_i < -50)	Yaw_AnglePid_i = -50;
//	if((error<0.2f) && (error>-0.2f)) Yaw_AnglePid_i = 0;		//达到目标时积分归零
//	I = Yaw_AnglePid_i * PID_YAW_ANGLE_KI;
//	
//	D = PID_YAW_ANGLE_KD * gryoYaw;
//	
//	angle_output = P + I - D;	
//	//************内环，角速度环pid*************
//	PID_Control(&Yaw_speed_pid,angle_output,gryoYaw);
//}


////实现函数：Encoder_angle_conversion(void)
////功    能：云台电机码盘值 转换为以0为中点的角度值
////Yaw_MID,Pitch_MID：码盘值中点
//#define DPI 0.043945f	//码值角度分辨率，360/8192
//void Encoder_angle_conversion(void)
//{
//	float Yaw_encode = motor_data[4].NowAngle;
//	float Pitch_encode = motor_data[5].NowAngle;
//	
//	if (Yaw_encode < (Yaw_MID-Yaw_MAX-100))//超出左极限说明码盘值过界
//		Yaw_encode += 8192;
//	else if (Yaw_encode > (Yaw_MID+Yaw_MAX+100))//超出右极限说明码盘值过界
//		Yaw_encode -= 8192;
//	Yaw_encode -= Yaw_MID;
//	Yaw_encode *= Yaw_Direction;
//	
//	if (Pitch_encode < (Pitch_MID-Pitch_MAX-100))//超出左极限说明码盘值过界
//		Pitch_encode += 8192;
//	else if (Pitch_encode > (Pitch_MID+Pitch_MAX+100))//超出右极限说明码盘值过界
//		Pitch_encode -= 8192;
//	Pitch_encode -= Pitch_MID;
//	Pitch_encode *= Pitch_Direction;//方向设置
//	
//	Yaw_EncodeAngle = Yaw_encode * DPI;
//	Pitch_EncodeAngle = Pitch_encode * DPI;
//}


////射速控制
////输入：发/分钟
//#define ShootRatio		36	//拨弹电机齿比
//#define ShootNumber		6		//波轮齿数（转一圈出多少个弹）
//void shoot_speed(uint16_t speed)
//{
//	int16_t motor_speed = speed/ShootNumber*ShootRatio;
//	
//	PID_Control(&Fire_speed_pid,motor_speed,motor_data[6].ActualSpeed);
//}

///*********************************************************
// *实现函数：Gimbal_Ctrl(void)
// *功    能：云台整体的控制
// *输    入：float pitch，pitch轴的绝对角度值(相对地面坐标)
// *          float yaw_rate，yaw轴的角速度值，单位rad/s
// *					shoot_speed,射速，发/分钟
// *说    明：因为yaw轴没有绝对坐标可供参考，所以选择以角速度的形势输入
// *					实际Yaw_angle_pid()以每周期转过的角度运行
// ********************************************************/
//static int16_t shoot_cnt = 0;

//void Gimbal_Ctrl(float pitch,float yaw_rate,uint16_t shoot_speed)
//{
//	int16_t motor_speed = -shoot_speed/ShootNumber*ShootRatio;//拨弹电机目标速度
//	
//	float d_yaw;								//每周期yaw转过的角度
//	d_yaw = yaw_rate / (2*PI);	//一圈2pi弧度，n圈
//	d_yaw = d_yaw * 360;		//度每秒
//	d_yaw = d_yaw / PID_Hz;	//每周期多少度
//	
//	//pitch轴
//	#ifdef PITCH_IMU_CTRL	//使用imu测量的绝对角度作为pitch轴反馈
//	Pitch_pid(pitch,-imu.pitch);
//	#else	//使用pitch电机码盘角度作反馈
//	Pitch_pid(pitch,Pitch_EncodeAngle);
//	#endif
//	//yaw轴
//	Yaw_angle_pid(d_yaw);
//	//拨弹电机
//	if(shoot_cnt<18)//循环次数,转60度需要36ms
//	{
//		shoot_cnt++;
//		//转动拨轮
//		motor_angle_pid(2048);
//	}
//	else
//	{
//		motor_angle_pid(0);
//		shoot_cnt++;
//		if (shoot_cnt > 500/(shoot_speed/60))
//			shoot_cnt = 0;
//	}
//	
// PID_Control(&Fire_speed_pid,motor_speed,motor_data[6].ActualSpeed);

//// 此处为安全考虑关闭电流
//CAN1_Send_Msg_gimbal( 0,Pitch_Direction*Pitch_speed_pid.Control_OutPut,
//	                    0);
//											//Fire_speed_pid.Control_OutPut);

//	}



