/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "referee.h"


//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_CAN_ADD_VALUE    1000.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_X
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_Z

#define TARGET_BULLET_SPEED					15.7f
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             500
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.03976829217f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  10
//拨弹速度
#define TRIGGER_SPEED               12.0f
#define FASTER_TRIGGER_SPEED				72.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.5f
#define BLOCK_TIME                  450
#define REVERSE_TIME                400
#define REVERSE_SPEED_LIMIT         50.0f

#define REVERSE_ECD									7854

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
#define PI_EIGHT                    0.39269908169872415480783042290994f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        270.0f
#define TRIGGER_ANGLE_PID_KI        4.0f
#define TRIGGER_ANGLE_PID_KD        25.0f

#define TRIGGER_FAST_SPEED_PID_KP		480.0f
#define TRIGGER_FAST_SPEED_PID_KI		6.7f
#define TRIGGER_FAST_SPEED_PID_KD		100.0f

#define TRIIGER_ECD_REVERSE_PID_KP	0.65f
#define TRIIGER_ECD_REVERSE_PID_KI	0.2f
#define TRIIGER_ECD_REVERSE_PID_KD	0.07f

#define TRIGGER_BULLET_PID_MAX_OUT  16000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_READY_PID_MAX_OUT   16000.0f
#define TRIGGER_READY_PID_MAX_IOUT  3000.0f

//摩擦轮PID
#define FRIC_ANGLE_PID_KP 40.0f
#define FRIC_ANGLE_PID_KI 1.0f
#define FRIC_ANGLE_PID_KD 0.0f

#define FRIC_PID_MAX_OUT 16000.0f
#define FRIC_PID_MAX_IOUT 5000.0f
#define FRIC_PID_MAX_IOUT_LEFT 3500.0f


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,					//fric on
		SHOOT_BULLET,					//trigger activated(shoot on bullet)
		SHOOT_ZERO_FORCE,			//trigger zero force
		SHOOT_CONTINUE_BULLET,//not applied for hero
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    const motor_measure_t *fric_motor_measure[2];
    ramp_function_source_t fric1_ramp;
    uint16_t fric_can1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_can2;
    pid_type_def trigger_motor_pid;
		pid_type_def trigger_motor_ecd_pid;//for a reverse with a fixed angle 
    pid_type_def fric_motor_pid[2];
		int16_t firc_speed[2];
    
    fp32 speed;
    fp32 speed_set;
    int32_t sum_ecd;
		int32_t sum_ecd_set;
		int32_t sum_ecd_reverse;//target reverse position
	  int32_t ecd_count;
    fp32 set_angle;
    int16_t given_current;

    bool_t press_l;
    bool_t last_press_l;
    uint16_t press_l_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    uint16_t heat_limit;
		uint16_t heat;
    ext_game_robot_state_t *shoot_state;
		uint8_t fric_error_count;
		
		fp32 last_bullet_speed;
		fp32 bullet_speed;
		uint8_t adp_flag;
		uint8_t adp_tolerant;
		pid_type_def bullet_speed_pid;
		
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
shoot_control_t *get_shoot_point(void);
void bullet_speed_adapt(void);
uint8_t get_shoot_mode(void);
int16_t get_mean_fric_rpm(void);
#endif
