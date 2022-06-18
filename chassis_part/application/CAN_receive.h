/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"


/* CAN send and receive ID */
#define CAN_Communication hcan2
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
		CAN_CAP = 0x211,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

		CAN_FRIC_LIFT_MOTOR_ID = 0x201,
		CAN_FRIC_RIGHT_MOTOR_ID = 0x202,
		
		
} can_msg_id_e;

/*Addtional Define*/
//can be added to enum
//#define gimbal_board_Id 0x210
//#define chassis_board_Id 0x211
#define chassis_motor_cmd_id 0x301
#define ui_info_id 0x302
#define heat_data_id 0x401
#define shoot_data_id 0x402
#define state_data_id 0x403

#define transform_key 1000

typedef struct
{		
		fp32 speed_set[3];
		uint8_t chassis_mode;
		uint8_t shoot_mode;  // 射击模式: 0-摩擦轮、拨弹轮都不动; 1-摩擦轮动拨弹轮不动; 2-都动
		uint8_t swing_flag;  // 1开小陀螺 0不开
		fp32 pitch_angel_degree;
		fp32 yaw_absolute_angel;
		fp32 yaw_relative_angel;  // 视为 body_angel
}gimbal_data_t;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct 
{
	 fp32 InputVot;
	 fp32 CapVot;
	 fp32 TestCurrent;
	 fp32 Target_Power;
} cap_measure_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
/**
  * @brief          return the fric 3508 motor data point
  * @param[in]      i: motor number,range [1,2]
  * @retval         motor data point
  */
/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[1,2]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_fric_motor_measure_point(uint8_t i);

/**
  * @brief          send fric current
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送摩擦轮电机电流
  * @param[in]      none
  * @retval         none
  */
 extern void CAN_CMD_FRIC(int16_t motor1, int16_t motor2);
void CAN_CMD_CAP(float power, float buffer);

//feedback not necessary
//void CAN_motor_feedback_send(int16_t chassis_motor1, int16_t chassis_motor2, int16_t chassis_motor3, int16_t chassis_motor4);

void CAN_heat_data_send(uint16_t shooter_heat, uint16_t shoot_heat_limit, uint8_t power_state);

void CAN_shoot_data_send(uint8_t	bullet_type, uint8_t	bullet_freq, float	bullet_speed);

void get_shoot_mode(uint8_t * shoot_mode);

void get_swing_mode(uint8_t * swing_flag);

void get_pitch_angel(fp32 * pitch_angel);

void get_yaw_absolute_angel(fp32 * yaw_absolute);

void get_yaw_relative_angel(fp32 * yaw_relative);

void get_cap_proportion(fp32 *cap_proportion);

gimbal_data_t *get_gimbal_data(void);
/*Addtional Functions*/

#endif
