#include "MPU6500.h"
#include "spi.h"
#include "delay.h"
#include "usart.h"
#include "filter.h"

uint8_t MPU_id = 0;
imu_t imu = {0}; 	//imu数据

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
	static uint8_t MPU_Tx;

	MPU6500=0;
  
  MPU_Tx = reg&0x7f;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Tx = data;
  SPI5_ReadWriteByte(MPU_Tx);
  
  MPU6500=1;
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500=0;
  
  MPU_Tx = reg|0x80;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Rx=SPI5_ReadWriteByte(reg|0x80);
  
  MPU6500=1;
  return  MPU_Rx;
}
uint8_t MPU6500_Read_Regs(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  MPU_Tx = reg|0x80;
  MPU_Rx=SPI5_ReadWriteByte(MPU_Tx);
  return  MPU_Rx;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  delay_ms(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  delay_ms(7);
}

////////////////////////////////////////////////////////////////////////
/////////////////////////////////读数据/////////////////////////////////
//读acc
void MPU6050AccRead(int16_t *accData)
{
	uint8_t buf[6];
	uint8_t i;

	for(i=0;i<6;i++)
	{
		buf[i] = SPI5_ReadWriteByte(0);
	}
	accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
	accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
	accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

//读gyro
void MPU6050GyroRead(int16_t *gyroData)
{
	uint8_t buf[6];
	uint8_t i;

	for(i=0;i<6;i++)
	{
		buf[i] = SPI5_ReadWriteByte(0);
	}
	gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
	gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
	gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}


/****************************************************************************
 *原函数：void ReadIMUSensorHandle(void)
 *功  能：读取mpu6500六轴ad值，并单位化，再进行低通滤波处理
 ***************************************************************************/
#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to correct at the same time
#define SENSOR_MAX_W 1000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)
#define CONSTANTS_ONE_G	9.80665f	//重力加速度* m/s^2
void ReadIMUSensorHandle(void)
{
	uint8_t i;

	//read raw
	MPU6500=0;	//选通器件
	SPI5_ReadWriteByte(MPU6500_ACCEL_XOUT_H|0x80);//写地址、读命令
	MPU6050AccRead(imu.accADC);		//接收数据
	imu.temperature = SPI5_ReadWriteByte(0)<<8;				//两位温度数据
	imu.temperature |= SPI5_ReadWriteByte(0);
	MPU6050GyroRead(imu.gyroADC);
	MPU6500=1;	//禁止器件
	
	//tutn to physical
	for(i=0; i<3; i++)
	{
		imu.accRaw[i]= (float)imu.accADC[i] *ACC_SCALE * CONSTANTS_ONE_G ;
		imu.gyroRaw[i]=(float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F /180.f;		//deg/s
	}

	imu.accb[0]=LPF2pApply_1(imu.accRaw[0]-imu.accOffset[0]);
	imu.accb[1]=LPF2pApply_2(imu.accRaw[1]-imu.accOffset[1]);
	imu.accb[2]=LPF2pApply_3(imu.accRaw[2]-imu.accOffset[2]);

	imu.gyro[0]=LPF2pApply_4(imu.gyroRaw[0]);
	imu.gyro[1]=LPF2pApply_5(imu.gyroRaw[1]);
	imu.gyro[2]=LPF2pApply_6(imu.gyroRaw[2]);
}



////////////////////////////////////////////////////////////////////////
/////////////////////////////////初始化/////////////////////////////////
//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  delay_ms(100);
  return 0;
}

//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
  uint8_t i = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x10},      // +-1000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
  delay_ms(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(i = 0; i < 10; i++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    delay_ms(1);
  }

  return 0;
}

//函数名：IMU_Init(void)
//描  述：初始化滤波器
#define IMU_FILTER_CUTOFF_FREQ	30.0f		//截止频率
#define IMU_SAMPLE_RATE 		500.0f			//1000.0f/(float)DMP_CALC_PRD
void IMU_Init(void)
{
	imu.ready=0;
	
	//filter rate
	LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);		//30Hz
	LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
	LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
	LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
	LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
	LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
}
