/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"

typedef struct
{
  const uint8_t *RC_buf;           
  const gimbal_motor_t *usb_yaw_motor;   
  const gimbal_motor_t *usb_pitch_motor;
  const motor_measure_t *usb_trigger_motor_measure;
  const motor_measure_t *usb_fric1_motor_measure;
  const motor_measure_t *usb_fric2_motor_measure;
	const fp32 *usb_INS_angle;
  const fp32 *usb_INS_accel;
  const fp32 *usb_INS_gyro;
  const fp32 *usb_INS_mag;
	const fp32 *usb_INS_quat;
	const fp32 *usb_bullet_speed;
  chassis_motor_t motor_chassis[4];             
} usb_debug_e;

union refree_4_byte_t
{
		float f;
		unsigned char buf[4];
};
extern void usb_task(void const * argument);

#endif
