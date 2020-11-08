#include "can1.h"
#include "Configuration.h"
#include "PID.h"
#include "IMU.h"
#include "Ctrl_gimbal.h"
#include "RemoteControl.h"
#include "user_lib.h"
#include "fric.h"
#include "User_API.h"
#include "led.h"
#include "Shoot.h"
#include "user_lib.h"
#include <math.h>
#include "Higher_Class.h"
#include "Ctrl_chassis.h"
#include "Shoot.h"

#define shoot_fric1_on(pwm) fric1_on(pwm) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on(pwm) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��

static Shoot_Motor_t trigger_motor;          //�������

uint16_t targetSpeed;

int16_t pid_out;
float printfBuff1,printfBuff2;

float Yaw_EncodeAngle, Pitch_EncodeAngle;//ʹ�����̼����������̨�Ƕ�
float E_yaw;//δУ׼��yaw�Ƕ�ֵ����Ҫ��������̨����ǰ��ֵE_yaw=imu.yaw
float angleYaw;
uint16_t shootSpeed;

//ʵ�ֺ�����pitch��������
//���룺Ŀ��Ƕȣ��Ƕȷ�������
void Pitch_pid(float Target_angle,float Measure_angle)
{
	//57.2957795=180/pi
	float gryoPitch = (imu.gyro[1]-imu.gyroOffset[1])*57.2957795f;
	PID_Control(&Pitch_angle_pid,Target_angle,Measure_angle);
//	if(fabs(Pitch_angle_pid.Err)<0.5) Pitch_angle_pid.Integrate = 0;		//�ﵽĿ��ʱ���ֹ���
//	PID_Control(&Pitch_speed_pid,Pitch_angle_pid.Control_OutPut,);
	PID_Control(&Pitch_speed_pid,Pitch_angle_pid.Control_OutPut,gryoPitch);
}

//ʵ�ֺ�����yaw��������
//���룺Ŀ��Ƕȣ��Ƕȷ�������
//void Yaw_pid(float Target_angle,float Measure_angle)
//{
//	//57.2957795=180/pi
////	float gryoYaw = (motor_data[4].ActualSpeed)*57.2957795f;
//	PID_Control(&Yaw_angle_pid,Target_angle,Measure_angle); 
////	if(fabs(Pitch_angle_pid.Err)<0.5) Pitch_angle_pid.Integrate = 0;		//�ﵽĿ��ʱ���ֹ���
////	PID_Control(&Pitch_speed_pid,Pitch_angle_pid.Control_OutPut,Pitch_EncodeAngle);
//	PID_Control(&Yaw_speed_pid,Yaw_angle_pid.Control_OutPut,Yaw_EncodeAngle);
//}

/*********************************************************
 *ʵ�ֺ�����Yaw_angle PID (void)
 *��    �ܣ���mpu6500 ������̨yaw��Ƕ�
 *��    �룺ÿһ��������ת�Ƕ�
 *˵    �������500HZ�ٶ����У�ÿ����ת��0.5�ȣ���1sת��250��
 ********************************************************/
static float T_yaw=0;		//Ŀ��Ƕ�
static float Yaw_AnglePid_i;//�ǶȻ�������
void Yaw_angle_pid(float Targrt_d_angle)
{
	float angleYaw = 0;
	float gyroYaw = (imu.gyro[2]-imu.gyroOffset[2])*57.2957795f;//���ٶ�
//	float angleYaw = -(imu.yaw - E_yaw);//�����Ƕȣ������ϵ�ʱ�ĽǶ�У׼imu�����Ƕ����
//	float angleYaw = -imu.yaw;
//	float angleYaw = Yaw_EncodeAngle;
	
	angleYaw = -imu.yaw;

	if(RC_Ctl.key.G){		
		if(Yaw_EncodeAngle > 2){
			T_yaw -= 0.05;
		}
		else if(Yaw_EncodeAngle <-2){
			T_yaw += 0.05;		
		}
		else{
			T_yaw += 0;
		}
	}
	else{
		T_yaw += Targrt_d_angle;
	}
	
//	T_yaw += Targrt_d_angle;		//����Ŀ��Ƕ�
	
	//inital value
	float error=0;
	float P=0,I=0,D=0;
	float angle_output=0;
	
//	T_yaw += Targrt_d_angle;		//����Ŀ��Ƕ�
//	T_yaw += Targrt_d_angle;
	if (T_yaw > 180)
	{
		T_yaw = T_yaw - 360;
	}
	else if (T_yaw < -180)
	{
		T_yaw = T_yaw + 360;
	}
	
	//************�⻷���ǶȻ�pi************
	error = T_yaw - angleYaw;
	if (error < -170)
	{
		error = error + 360;
	}
	else if (error > 170)
	{
		error = error - 360;
	}
	
	P = error * PID_YAW_ANGLE_KP;
	
	if((error<10)&&(error>-10))
		Yaw_AnglePid_i += error;
	//�����޷�
	if(Yaw_AnglePid_i > 50)	Yaw_AnglePid_i = 50;
	if(Yaw_AnglePid_i < -50)	Yaw_AnglePid_i = -50;
	if((error<0.2f) && (error>-0.2f)) Yaw_AnglePid_i = 0;		//�ﵽĿ��ʱ���ֹ���
	I = Yaw_AnglePid_i * PID_YAW_ANGLE_KI;
	
	D = PID_YAW_ANGLE_KD * gyroYaw;
	
	angle_output = P + I - D;	
	//************�ڻ������ٶȻ�pid*************
	PID_Control(&Yaw_speed_pid,angle_output,gyroYaw);
}

/*********************************************************************
 *ʵ�ֺ�����void virtual_encoders(int16_t aim[4])
 *��    �ܣ�������������
 *��    �룺������Ŀ��ֵ
 *˵    ����
 ********************************************************************/
int16_t Encoders_shoot;		//��������ֵ
void virtual_encoders_shoot(int16_t target)
{
	Encoders_shoot += target;
	if (Encoders_shoot>8191) Encoders_shoot -= 8192;
	else if (Encoders_shoot<0) Encoders_shoot += 8192;
}

/*********************************************************************
 *ʵ�ֺ�����int encoders_err(int error)
 *��    �ܣ�����ʵ��Ŀ��������Ŀ���������̲�ֵ
 *��    �룺���ʶ��������ڲ�ֵ
 *��    �أ�ʵ��Ŀ��������Ŀ���������̲�ֵ
 *˵    ����
 ********************************************************************/

int encoders_err_shoot(int error)
{
	int temp;
	
	temp = motor_data[6].NowAngle + error;
	temp = temp%8192;	
	if (temp<0) temp += 8192;						//ʵ��Ŀ������ֵ
	temp = Encoders_shoot - temp;
	if (temp<-3000) temp += 8192;
	else if (temp>3000) temp -= 8192;
	
	return temp;
}


////���ٿ���
////���룺��/����
#define ShootRatio		36	//��������ݱ�
#define ShootNumber		6		//���ֳ�����תһȦ�����ٸ�����
void shoot_speed(uint16_t speed)
{
	int16_t motor_speed = speed/ShootNumber*ShootRatio;
	
	PID_Control(&Fire_speed_pid,motor_speed,motor_data[6].ActualSpeed);
}

////ʵ�ֺ�����Encoder_angle_conversion(void)
////��    �ܣ���̨�������ֵ ת��Ϊ��0Ϊ�е�ĽǶ�ֵ
////Yaw_MID,Pitch_MID������ֵ�е�
#define DPI 0.043945f	//��ֵ�Ƕȷֱ��ʣ�360/8192
void Encoder_angle_conversion(void)
{
	float Yaw_encode = motor_data[4].NowAngle;
	float Pitch_encode = motor_data[5].NowAngle;
	
	if (Yaw_encode < (Yaw_MID-Yaw_MAX-100))//��������˵������ֵ����
		Yaw_encode += 8192;
	else if (Yaw_encode > (Yaw_MID+Yaw_MAX+100))//�����Ҽ���˵������ֵ����
		Yaw_encode -= 8192;
	Yaw_encode -= Yaw_MID;
	Yaw_encode *= Yaw_Direction;
	
	if (Pitch_encode < (Pitch_MID-Pitch_MAX-100))//��������˵������ֵ����
		Pitch_encode += 8192;
	else if (Pitch_encode > (Pitch_MID+Pitch_MAX+100))//�����Ҽ���˵������ֵ����
		Pitch_encode -= 8192;
	Pitch_encode -= Pitch_MID;
	Pitch_encode *= Pitch_Direction;//��������
	
	Yaw_EncodeAngle = Yaw_encode * DPI;
	Pitch_EncodeAngle = Pitch_encode * DPI;
}

/*********************************************************************
 *ʵ�ֺ�����void motor_angle_pid(int16_t TargetAngle)
 *��    �ܣ����λ�ÿ���
 *��    �룺���ÿ���ڵ�Ŀ����ֵ
 *˵    ����ʹ�ô���pid���ڻ�Ϊ�ٶȻ�
 ********************************************************************/
#define ANGLE_KP (0.26f)//�ǶȻ�����
#define ANGLE_KI (0.04f)
#define ANGLE_KD (1.0f)
#define SPEED_KP (2.4f)//�ٶȻ�����
#define SPEED_KI (0.2f)
#define SPEED_KD (2.0f)
static float angle_i=0,speed_i=0;
static int16_t last_speed = 0;
static int old_target = 0;
void motor_angle_pid(int16_t TargetAngle)
{
	/**********λ�û�***********/
	float P,I,D;									//�м�ֵ����
	int error;

	virtual_encoders_shoot(TargetAngle);
	
	error = TargetAngle;											//�ǶȲ�ֵ
	error += old_target - motor_data[6].D_Angle;	//�ۼ���һ��δ��ɲ�ֵ
	error += encoders_err_shoot(error);		//�ۼ�ʵ������ֵ����������ֵ�Ĳ�
	
	old_target = error;				//���¾�Ŀ��ֵ
		
	P = ANGLE_KP * error;
	angle_i += error;
	if(angle_i > 1000) angle_i = 1000;
	if(angle_i <- 1000) angle_i = -1000;
	I = ANGLE_KI * angle_i;	
	pid_out = P + I - D;
		
	if (pid_out > 10000) pid_out = 10000;			//λ�û����(Ŀ��ת��)�޷�
	else if(pid_out < -10000) pid_out = -10000;
	
	if ((pid_out<70) && (pid_out>-70)) pid_out = 0;
	
	/**********�ٶȻ�***********/
	error = pid_out - motor_data[6].ActualSpeed;		//�ٶ�ƫ��ֵ
	P = SPEED_KP * error;
	
	if((error < 500)&&(error > -500)) speed_i += error;
	if (speed_i>1000) speed_i=1000;		//�����޷�
	if (speed_i<-1000) speed_i=-1000;
	I = SPEED_KI * speed_i;
	
	D = SPEED_KD * (motor_data[6].ActualSpeed - last_speed);
	last_speed = motor_data[6].ActualSpeed;
	
	pid_out = P+I-D;
	pid_out += 0.4 * pid_out;
}


/*********************************************************
 *ʵ�ֺ�����Gimbal_Ctrl(void)
 *��    �ܣ���̨����Ŀ���
 *��    �룺float pitch��pitch��ľ��ԽǶ�ֵ(��Ե�������)
 *          float yaw_rate��yaw��Ľ��ٶ�ֵ����λrad/s
 *					shoot_speed,���٣���/����
 *˵    ������Ϊyaw��û�о�������ɹ��ο�������ѡ���Խ��ٶȵ���������
 *					ʵ��Yaw_angle_pid()��ÿ����ת���ĽǶ�����
 ********************************************************/
static int16_t shoot_cnt = 0;
int twist = 0;
void Gimbal_Ctrl(float pitch, float yaw_rate, uint16_t shoot_speed)
{
	int16_t motor_speed = shoot_speed/ShootNumber*ShootRatio;//�������Ŀ���ٶ�
	float d_yaw;								//ÿ����yawת���ĽǶ�
	d_yaw = yaw_rate / (2*PI);	//һȦ2pi���ȣ�nȦ
	d_yaw = d_yaw * 360;		//��ÿ��
	d_yaw = d_yaw / PID_Hz;	//ÿ���ڶ��ٶ�
	
	//pitch��
	#ifdef PITCH_IMU_CTRL	//ʹ��imu�����ľ��ԽǶ���Ϊpitch�ᷴ��
	Pitch_pid(-pitch, imu.roll);
//	#else	//ʹ��pitch������̽Ƕ�������
//	Pitch_pid(-pitchAngle, Pitch_EncodeAngle);
	#endif
	
//	//yaw��
//	#ifdef YAW_ENCO_CTRL	//ʹ��yaw������̽Ƕ�������
//	Yaw_pid(Yaw_MID+yaw_rate, Yaw_EncodeAngle);
//	#endif

	//yaw��
	Yaw_angle_pid(d_yaw);
	
	if(shoot_speed != 0)
		{	
			if(shoot_cnt<18)//ѭ������,ת60����Ҫ36ms
				{
					shoot_cnt++;
					//ת������
					motor_angle_pid(2048);
				}
			else
				{
					motor_angle_pid(0);
					shoot_cnt++;
					if (shoot_cnt > 500/(shoot_speed/60))
					shoot_cnt = 0;
				}
			PID_Control(&Fire_speed_pid,motor_speed,motor_data[6].ActualSpeed);
			//	�˴�Ϊ��ȫ���ǹرյ���
			CAN1_Send_Msg_gimbal(Yaw_Direction*Yaw_speed_pid.Control_OutPut,Pitch_Direction*Pitch_speed_pid.Control_OutPut,Fire_speed_pid.Control_OutPut);
//			CAN1_Send_Msg_gimbal(Yaw_Direction*Yaw_speed_pid.Control_OutPut,Pitch_Direction*Pitch_speed_pid.Control_OutPut,M2006_Direction*pid_out);
		}
	else
		CAN1_Send_Msg_gimbal(Yaw_Direction*Yaw_speed_pid.Control_OutPut,Pitch_Direction*Pitch_speed_pid.Control_OutPut,0);
}

//��̨��ʼ������whileǰ��can��ʼ����һ��ʱ�����
void Gimbal_Ctrl_init(void)
{
	Encoders_shoot = motor_data[6].D_Angle;
}

void shoot_init(void)
{
	ramp_init(&trigger_motor.fric1_ramp, Fric_DOWN, Fric_OFF);
  ramp_init(&trigger_motor.fric2_ramp, Fric_DOWN, Fric_OFF);
}

/**
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_control_loop(uint16_t targetSpeed)//uint16_t fricSpee
{
	shoot_laser_on();       //���⿪��
	
	shoot_fric1_on(targetSpeed);
	shoot_fric2_on(targetSpeed);	
}

/**
  * @brief          ���̸���
  * @param[in]      void
  */
/*********************************************************
 *Function: void Twist
 *input:
 *Comment: It has to be used by timer, as PID_HZ
*********************************************************/
static float Twist_buff = TWIST_SPEED;
void Twist(float Vx, float Vy, float Wz, float pitch_angle, uint16_t shoot_speed)
{
	float Chassis_Wz;
	float Vy_Offset;
	Encoder_angle_conversion();//encoder turn into angle
	Matrix_Translate(&Vx,&Vy,Yaw_EncodeAngle);//by using matrix rotation, make sport target from gimbal to chassis
	
	if(Yaw_EncodeAngle>TWIST_ANGLE)	//��̨�ڵ����ұ�
		Twist_buff = TWIST_SPEED;
	else if(Yaw_EncodeAngle<-TWIST_ANGLE)	//��̨�ڵ������
		Twist_buff = -TWIST_SPEED;
	
	Chassis_Wz = Wz+Twist_buff;//Ť�������Ͽ�����
	Vy_Offset = Wz_Lpf*Twist_Vy_P;//vy�Ჹ��
	Astrict_Acc(Vx,Vy,Chassis_Wz);//���ٶ�����
	
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf-Vy_Offset,Wz_Lpf);//���̿��ƽӿ�
//	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//���̿��ƽӿ�

	Gimbal_Ctrl(pitch_angle,Wz,shoot_speed);//��̨���ƽӿ�
//	Chassis_Task();
}

/*********************************************************
 *ʵ�ֺ�����void Chassis_AutoFollow(float Vx, float Vy, float Wz, float pitch_angle)
 *��    �ܣ���̨����ģʽ(��ͷģʽ)�������Զ�������̨(���Զ�����)
 *��    �룺vx,vx �˶�x���y���ٶȣ���λmm/s
 *					wz,z����ת�ٶȣ�rad/s(���̸�����̨)
 *˵    �����˺�����Ҫ����ʱ���ã���PID_Hz
 *          �˶�˳ʱ��Ϊ������
 ********************************************************/
#define LIM_ANGLE		(GIM_FOLLOW_BUFFERING+GIM_FOLLOW_ANGLE)		//����������
void Chassis_AutoFollow(float Vx, float Vy, float Wz, float pitch_angle, uint16_t shoot_speed)
{
	float Gimbal_Wz;//��̨�ٶ�
	float section_float = Wz*GIM_FOLLOW_ANGLE_D;	//���巶Χ����ֵ����λ��

	Encoder_angle_conversion();//���̽Ƕ�ת��
	Matrix_Translate(&Vx,&Vy,Yaw_EncodeAngle);//����̨Ϊ������˶�Ŀ����ת��������
	
	//��ԽǶ����м䷶Χ�ڣ����̲���
	if((Yaw_EncodeAngle < GIM_FOLLOW_ANGLE+section_float)		//�Ұ����䣬��̨����ʱ�����С
		&&(Yaw_EncodeAngle > -GIM_FOLLOW_ANGLE+section_float))//������䣬��̨����ʱ�����С
	{
		Astrict_Acc(Vx,Vy,0);//���ٶ�����	,���̲���ת
		Gimbal_Wz = Wz*FLLOW_RATE;//����Χ�ڣ���̨ת��*FLLOW_RATE��
	}
	else if((Yaw_EncodeAngle>GIM_FOLLOW_ANGLE+section_float) && (Yaw_EncodeAngle<LIM_ANGLE))	//��̨���һ�����
	{
		if(Wz>0)//��̨�����˶�
		{
			Astrict_Acc(Vx,Vy,Wz*FLLOW_RATE);//���ٶ�����,������FLLOW_RATE����׷����̨
			Gimbal_Wz = Wz*FLLOW_RATE;//��̨�⽵��ת��
		}
		else
		{
			Astrict_Acc(Vx,Vy,0);//���ٶ�����,���̵ȴ�
			Gimbal_Wz = Wz*FLLOW_RATE;//��̨FLLOW_RATE������Χ���˶�
		}
	}
	else if((Yaw_EncodeAngle<-GIM_FOLLOW_ANGLE+section_float)&&(Yaw_EncodeAngle>(-LIM_ANGLE)))	//��̨���󻺳���
	{
		if(Wz<0)//��̨�����˶�
		{
			Astrict_Acc(Vx,Vy,Wz*FLLOW_RATE);//���ٶ�����,������FLLOW_RATE����׷����̨
			Gimbal_Wz = Wz*FLLOW_RATE;//��̨�⽵��ת��
		}
		else
		{
			Astrict_Acc(Vx,Vy,0);//���ٶ�����,���̵ȴ�
			Gimbal_Wz = Wz*FLLOW_RATE;//��̨FLLOW_RATE������Χ���˶�
		}
	}
	else if(Yaw_EncodeAngle>LIM_ANGLE)	//���巶Χ���ұ�
	{
		if(Wz>0)//��̨�����˶�
		{
			Astrict_Acc(Vx,Vy,Wz*FLLOW_RATE);//���ٶ�����,������FLLOW_RATE����׷����̨
			Gimbal_Wz = Wz;//��̨����ת��
		}
		else
		{
			Astrict_Acc(Vx,Vy,0);//���ٶ�����,���̵ȴ�
			Gimbal_Wz = Wz*FLLOW_RATE;//FLLOW_RATE������Χ���˶�
		}
	}
	else if(Yaw_EncodeAngle<-LIM_ANGLE)	//���巶Χ�����
	{
		if(Wz<0)//��̨�����˶�
		{
			Astrict_Acc(Vx,Vy,Wz*FLLOW_RATE);//���ٶ�����,������FLLOW_RATE����׷����̨
			Gimbal_Wz = Wz;//��̨����ת��
		}
		else
		{
			Astrict_Acc(Vx,Vy,0);//���ٶ�����,���̵ȴ�
			Gimbal_Wz = Wz*FLLOW_RATE;//FLLOW_RATE������Χ���˶�
		}
	}
	
	Inverse_Kinematic_Ctrl(Vx_Lpf,Vy_Lpf,Wz_Lpf);//���̿��ƽӿ�
	Gimbal_Ctrl(pitch_angle,Gimbal_Wz,shoot_speed);//��̨���ƽӿ�
}



