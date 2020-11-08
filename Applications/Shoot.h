#include <stdint.h>
#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"
#include "user_lib.h"

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80

typedef struct
{
    ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    bool_t move_flag;
    uint32_t cmd_time;
    uint32_t run_time;
    bool_t key;
    uint16_t key_time;
    bool_t shoot_done;
    uint8_t shoot_done_time;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
} Shoot_Motor_t;

extern uint16_t targetSpeed;
extern uint16_t shootSpeed;

//extern uint16_t fricSpeed;
extern void motor_angle_pid(int16_t TargetAngle);
void Gimbal_Ctrl(float pitchAngle, float yawAngle, uint16_t shoot_speed);
extern void Encoder_angle_conversion(void);
extern void Gimbal_Ctrl_init(void);
extern void shoot_speed(uint16_t speed);
extern void Shoot_Control(void);
//extern void Yaw_pid(float Target_angle,float Measure_angle);
extern void shoot_init(void);
extern void shoot_control_loop(uint16_t targetSpeed);
extern void Twist(float Vx, float Vy, float Wz, float pitch_angle, uint16_t shoot_speed);
extern void Chassis_AutoFollow(float Vx, float Vy, float Wz, float pitch_angle, uint16_t shoot_speed);
extern void Yaw_angle_pid(float Targrt_d_angle);

#endif

