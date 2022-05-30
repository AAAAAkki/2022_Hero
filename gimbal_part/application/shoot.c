/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
extern ext_game_robot_state_t robot_state;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

shoot_control_t shoot_control; //射击数据
fp32 trigger_speed = 0;
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid0[3] = {40, 0.2, 0};
		static const fp32 Fric_speed_pid1[3] = {40, 1, 0};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    shoot_control.shoot_state = get_robot_status_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric_motor_measure[0] = get_fric_motor_measure_point(1);
    shoot_control.fric_motor_measure[1] = get_fric_motor_measure_point(2);
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_pid[0], PID_POSITION, Fric_speed_pid0, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_pid[1], PID_POSITION, Fric_speed_pid1, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
	//	shoot_control.fric_motor_pid[0].proportion_output_filter_coefficient = exp(-300*1E-3);
	//	shoot_control.fric_motor_pid[1].proportion_output_filter_coefficient = exp(-300*1E-3);
//		shoot_control.trigger_motor_pid.derivative_output_filter_coefficient = exp(-0.05*1E-3);
		shoot_control.trigger_motor_pid.proportion_output_filter_coefficient = exp(-1000*1E-3);
		
		shoot_control.ecd_count=0;
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_15, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_15, FRIC_OFF);
    shoot_control.fric_can1 = FRIC_OFF;
    shoot_control.fric_can2 = FRIC_OFF;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		shoot_control.sum_ecd_set=shoot_control.sum_ecd;
}

/**
  * @brief          射击循环q2
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
		trigger_speed = 2;
		
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
		else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = trigger_speed;
        trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
     
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
				if(gimbal_behaviour != GIMBAL_ZERO_FORCE){
						PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
						shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
				}
				else
						shoot_control.given_current = 0;
				shoot_laser_off();
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        shoot_control.fric1_ramp.out = 0;
        shoot_control.fric2_ramp.out = 0;
    }
    else
    {
        shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
				PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
    
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
    }

    shoot_control.fric_can1 = (int16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_can2 = (int16_t)(shoot_control.fric2_ramp.out);
    PID_calc(&shoot_control.fric_motor_pid[0], shoot_control.fric_motor_measure[0]->speed_rpm, -shoot_control.fric_can1);
    PID_calc(&shoot_control.fric_motor_pid[1], shoot_control.fric_motor_measure[1]->speed_rpm, shoot_control.fric_can1);
		
    CAN_CMD_FRIC((int16_t)shoot_control.fric_motor_pid[0].out, (int16_t)shoot_control.fric_motor_pid[1].out);
		
		return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
    static uint8_t fric_state = 0;
    static uint16_t press_time = 0;
    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    if ((switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_mid(last_s) && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    if (shoot_control.shoot_mode == SHOOT_READY)
    {
        if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
        {
						static int count=0;
						count++;
						if(count>5000)
						{
							shoot_control.shoot_mode = SHOOT_BULLET;
							count=0;
						}
        }
    }

    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_mode == SHOOT_READY||shoot_control.shoot_mode == SHOOT_BULLET))
    {

        if ((shoot_control.press_l && shoot_control.last_press_l == 0) )
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    
				if(shoot_control.press_l_time==PRESS_LONG_TIME)
				{
	         shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
				}
    }
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        if (!shoot_control.press_l)
        {
            if (shoot_control.shoot_mode != SHOOT_BULLET)
            {
                shoot_control.shoot_mode = SHOOT_READY;
            }
        }
    }

    get_shoot_heat1_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
   /* if (!toe_is_error(REFEREE_TOE))
    {
        if ((shoot_control.heat + 10 >= shoot_control.heat_limit) && !(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C))
        {
						if(shoot_control.shoot_mode==SHOOT_CONTINUE_BULLET)
							{
									shoot_control.shoot_mode = SHOOT_READY;
							}
        }
    }*/
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

	
	
	
		shoot_control.firc_speed[0]=shoot_control.fric_motor_measure[0]->speed_rpm;
		shoot_control.firc_speed [1]=-shoot_control.fric_motor_measure[1]->speed_rpm;
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

//    //二阶低通滤波
//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = speed_fliter_3;
//    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
//    shoot_control.speed = speed_fliter_3;
		
		
    speed_fliter_3 = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    shoot_control.speed = speed_fliter_3;
    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 20圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }
		shoot_control.sum_ecd= shoot_control.ecd_count*8191+ shoot_control.shoot_motor_measure->ecd;
    //鼠标按键
		shoot_control.last_press_l = shoot_control.press_l;
		shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
		if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }
		
		
		if(shoot_control.shoot_mode==SHOOT_STOP)
		{
				shoot_control.sum_ecd_set=shoot_control.sum_ecd;
		}
		
		
		
    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    if (!toe_is_error(REFEREE_TOE))
    {
        if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 15)
        {
            shoot_control.fric1_ramp.max_value = FRIC_15;
            shoot_control.fric2_ramp.max_value = FRIC_15;
        }
        else if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 18)
        {
            shoot_control.fric1_ramp.max_value = FRIC_18;
            shoot_control.fric2_ramp.max_value = FRIC_18;
        }
        else if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 30)
        {
            shoot_control.fric1_ramp.max_value = FRIC_30;
            shoot_control.fric2_ramp.max_value = FRIC_30;
        }
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_15;
        shoot_control.fric2_ramp.max_value = FRIC_15;
    }
		
		
}

static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.sum_ecd_set=shoot_control.sum_ecd_set+31459;
        shoot_control.move_flag = 1;
    }
    //到达角度判断
    if (shoot_control.sum_ecd_set - shoot_control.sum_ecd > 629)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = trigger_speed;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY;
    }
}
shoot_control_t *get_shoot_point(void)
{
    return &shoot_control;
}