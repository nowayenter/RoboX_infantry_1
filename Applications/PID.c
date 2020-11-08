#include "PID.h"
#include "Configuration.h"
#include <math.h>
#include <stdlib.h>
/*
*PID计算、PID数据初始化
*/

PID_Controler motor_speed_pid[4];	//底盘电机速度环pid数据
PID_Controler Yaw_speed_pid={0};	//云台yaw轴角速度
//PID_Controler Yaw_angle_pid={0};//云台yaw角速度
PID_Controler Pitch_speed_pid={0};//云台pitch轴角速度
PID_Controler Pitch_angle_pid={0};//云台pitch轴角度
PID_Controler Fire_speed_pid={0};	//波轮电机速度

void PID_Init(PID_Controler *Controler)
{
	
}

//pid初始化
void Total_PID_Init(void)
{
	u8 i;
	//********底盘电机*******
	for(i=0;i<4;i++)
	{
		motor_speed_pid[i].Kp = PID_MOTOR_SPEED_KP;
		motor_speed_pid[i].Ki = PID_MOTOR_SPEED_KI;
		motor_speed_pid[i].Kd = PID_MOTOR_SPEED_KD;
		motor_speed_pid[i].Integrate_Limit_Flag = 1;//积分限幅
		motor_speed_pid[i].Integrate_Separation_Flag = 1;//积分分离
		motor_speed_pid[i].Integrate_Max = PID_MOTOR_SPEED_I_MAX;//积分幅值
		motor_speed_pid[i].Integrate_Separation_Err = PID_MOTOR_SPEED_I_Err;//积分分离范围
		motor_speed_pid[i].Control_OutPut_Limit = 30000;//最大输出
	}
	//********云台pitch轴*******
	Pitch_speed_pid.Integrate_Limit_Flag = 1;//积分限幅
	Pitch_speed_pid.Integrate_Separation_Flag = 1;//积分分离
	Pitch_speed_pid.Integrate_Max = PID_PITCH_SPEED_I_MAX;//积分幅值
	Pitch_speed_pid.Integrate_Separation_Err = PID_PITCH_SPEED_I_Err;//积分分离范围
	Pitch_speed_pid.Kp = PID_PITCH_SPEED_KP;
	Pitch_speed_pid.Ki = PID_PITCH_SPEED_KI;
	Pitch_speed_pid.Kd = PID_PITCH_SPEED_KD;
	Pitch_speed_pid.Control_OutPut_Limit = 5000;//最大输出
	Pitch_angle_pid.Integrate_Limit_Flag = 1;//积分限幅
	Pitch_angle_pid.Integrate_Separation_Flag = 1;//积分分离
	Pitch_angle_pid.Integrate_Max = PID_PITCH_ANGLE_I_MAX;//积分幅值
	Pitch_angle_pid.Integrate_Separation_Err = PID_PITCH_ANGLE_I_Err;//积分分离范围
	Pitch_angle_pid.Kp = PID_PITCH_ANGLE_KP;
	Pitch_angle_pid.Ki = PID_PITCH_ANGLE_KI;
	Pitch_angle_pid.Kd = PID_PITCH_ANGLE_KD;
	Pitch_angle_pid.Control_OutPut_Limit = 30000;//最大输出
	//********云台yaw轴*******
	Yaw_speed_pid.Integrate_Limit_Flag = 1;//积分限幅
	Yaw_speed_pid.Integrate_Separation_Flag = 1;//积分分离
	Yaw_speed_pid.Integrate_Max = PID_YAW_SPEED_I_MAX;//积分幅值
	Yaw_speed_pid.Integrate_Separation_Err = PID_YAW_SPEED_I_Err;//积分分离范围
	Yaw_speed_pid.Kp = PID_YAW_SPEED_KP;
	Yaw_speed_pid.Ki = PID_YAW_SPEED_KI;
	Yaw_speed_pid.Kd = PID_YAW_SPEED_KD;
	Yaw_speed_pid.Control_OutPut_Limit = 5000;//最大输出
	//////////////////////////////////////////
//	Yaw_angle_pid.Integrate_Limit_Flag = 1;//积分限幅
//	Yaw_angle_pid.Integrate_Separation_Flag = 1;//积分分离
//	Yaw_angle_pid.Integrate_Max = PID_YAW_ANGLE_I_MAX;//积分幅值
//	Yaw_angle_pid.Integrate_Separation_Err = PID_YAW_ANGLE_I_Err;//积分分离范围
//	Yaw_angle_pid.Kp = PID_YAW_ANGLE_KP;
//	Yaw_angle_pid.Ki = PID_YAW_ANGLE_KI;
//	Yaw_angle_pid.Kd = PID_YAW_ANGLE_KD;
//	Yaw_angle_pid.Control_OutPut_Limit = 30000;//最大输出
	//********拨弹电机*******
	Fire_speed_pid.Integrate_Limit_Flag = 1;//积分限幅
	Fire_speed_pid.Integrate_Max = PID_FIRE_SPEED_I_MAX;//积分幅值
	Fire_speed_pid.Kp = PID_FIRE_SPEED_KP;
	Fire_speed_pid.Ki = PID_FIRE_SPEED_KI;
	Fire_speed_pid.Kd = PID_FIRE_SPEED_KD;
	Fire_speed_pid.Control_OutPut_Limit = 30000;
}


/***********************
 *float PID_Control(PID_Controler *Controler,float target,float measure)
 *普通位置式pid
 *输入：*Controler：pid数据结构体地址
 *       target：目标值
 *       measure：测量值
 ***********************/
float PID_Control(PID_Controler *Controler, float target, float measure)
{
	/*******偏差计算*********************/
  Controler->Last_Err = Controler->Err;//保存上次偏差
  Controler->Err = target - measure;//期望减去反馈得到偏差
  if(Controler->Err_Limit_Flag == 1)//偏差限幅度标志位
  {
		if(Controler->Err >= Controler->Err_Max)   Controler->Err = Controler->Err_Max;
		if(Controler->Err <= -Controler->Err_Max)  Controler->Err = -Controler->Err_Max;
  }
	/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag == 1)//积分分离标志位
  {
    if(fabs(Controler->Err) <= Controler->Integrate_Separation_Err)
			Controler->Integrate += Controler->Ki * Controler->Err;
  }
  else
    Controler->Integrate += Controler->Err;
	/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag == 1)//积分限制幅度标志
 {
  if(Controler->Integrate >= Controler->Integrate_Max)
    Controler->Integrate = Controler->Integrate_Max;
  if(Controler->Integrate <= -Controler->Integrate_Max)
    Controler->Integrate = -Controler->Integrate_Max ;
 } 
	/*******总输出计算*********************/
  Controler->Last_Control_OutPut = Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut =  Controler->Kp * Controler->Err//比例
															+Controler->Ki * Controler->Integrate//积分
															+Controler->Kd * (Controler->Err-Controler->Last_Err);//微分
	/*******总输出限幅*********************/
  if(Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
	/*******返回总输出*********************/
  return Controler->Control_OutPut;
}






/* Butterworth滤波 */
float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
	/* 获取最新x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
				+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}


Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};


/***********************
 *PID_Control_Div_LPF(PID_Controler *Controler,float target,float measure)
 *带微分低通滤波的位置式pid
 *输入：*Controler：pid数据结构体地址
 *       target：目标值
 *       measure：测量值
 ***********************/
float PID_Control_Div_LPF(PID_Controler *Controler,float target,float measure)
{
  u16 i=0;
/*******偏差计算*********************/
  Controler->Last_Err = Controler->Err;//保存上次偏差
  Controler->Err = target - measure;//期望减去反馈得到偏差
  Controler->Dis_Err = Controler->Err - Controler->Last_Err;//原始微分
  for(i=4;i>0;i--)//数字低通后微分项保存
		Controler->Dis_Error_History[i] = Controler->Dis_Error_History[i-1];
  Controler->Dis_Error_History[0] = Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  if(Controler->Err_Limit_Flag == 1)//偏差限幅度标志位
  {
		if(Controler->Err >= Controler->Err_Max)   Controler->Err =  Controler->Err_Max;
		if(Controler->Err <= -Controler->Err_Max)  Controler->Err = -Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag == 1)//积分分离标志位
  {
    if(fabs(Controler->Err) <= Controler->Integrate_Separation_Err)
			Controler->Integrate += Controler->Ki * Controler->Err;
  }
  else
  {
    Controler->Integrate += Controler->Ki * Controler->Err;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag == 1)//积分限制幅度标志
 {
  if(Controler->Integrate >= Controler->Integrate_Max)
    Controler->Integrate = Controler->Integrate_Max;
  if(Controler->Integrate <= -Controler->Integrate_Max)
    Controler->Integrate = -Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut = Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut = Controler->Kp * Controler->Err//比例
                         +Controler->Integrate//积分
                         //+Controler->Kd*Controler->Dis_Err;//微分
                         +Controler->Kd * Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
/*******总输出限幅*********************/
  if(Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}



/***********************
 *PID_Control_Err_LPF(PID_Controler *Controler,float target,float measure)
 *带偏差低通滤波的位置式pid
 *输入：*Controler：pid数据结构体地址
 *       target：目标值
 *       measure：测量值
 ***********************/
float PID_Control_Err_LPF(PID_Controler *Controler,float target,float measure)
{
/*******偏差计算*********************/
  Controler->Last_Err = Controler->Err;//保存上次偏差
  Controler->Err = target - measure;//期望减去反馈得到偏差
  Controler->Dis_Err = Controler->Err - Controler->Last_Err;//原始微分

  Controler->Last_Err_LPF = Controler->Err_LPF;
  Controler->Err_LPF = Control_Device_LPF(Controler->Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  Controler->Dis_Err_LPF = Controler->Err_LPF - Controler->Last_Err_LPF;//偏差经过低通后的微分量

  if(Controler->Err_Limit_Flag == 1)//偏差限幅度标志位
  {
  if(Controler->Err_LPF >= Controler->Err_Max)   Controler->Err_LPF = Controler->Err_Max;
  if(Controler->Err_LPF <= -Controler->Err_Max)  Controler->Err_LPF =-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag == 1)//积分分离标志位
  {
    if(fabs(Controler->Err_LPF) <= Controler->Integrate_Separation_Err)
    Controler->Integrate += Controler->Ki * Controler->Err_LPF;
  }
  else
  {
    Controler->Integrate += Controler->Ki * Controler->Err_LPF;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag == 1)//积分限制幅度标志
 {
  if(Controler->Integrate >= Controler->Integrate_Max)
    Controler->Integrate = Controler->Integrate_Max;
  if(Controler->Integrate <= -Controler->Integrate_Max)
    Controler->Integrate = -Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut = Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut = Controler->Kp * Controler->Err_LPF//比例
                         +Controler->Integrate//积分
                          +Controler->Kd * Controler->Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
/*******总输出限幅*********************/
  if(Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
  Controler->Control_OutPut = Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
  Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}


//pid积分复位
void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}

/*滤波器参数表
Butter_Parameter 80HZ_Parameter={
  //200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter 60HZ_Parameter={
  //200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter 51HZ_Parameter={
  //200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157,
};

Butter_Parameter 30HZ_Parameter={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter 20HZ_Parameter={
  //200hz---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter 15HZ_Parameter={
  //200hz---15hz
  1,   -1.348967745253,   0.5139818942197,
  0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter 10HZ_Parameter={
  //200hz---10hz
  1,   -1.561018075801,   0.6413515380576,
  0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter 5HZ_Parameter={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter 2HZ_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/

