/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb����������?1?7
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "Can_receive.h"
#include "remote_control.h"

#include "detect_task.h"
#include "voltage_task.h"
#include "referee.h"
usb_debug_e usb_debug;

static void usb_printf(const char *fmt, ...);

uint8_t usb_txbuf[63];
const error_t *error_list_usb_local;
static void get_motormeasure(uint8_t chassis_number, uint8_t array);
static void get_triggermeasure(uint8_t array);
static void get_fric1measure(uint8_t array);
static void get_fric2measure(uint8_t array);
static void get_INS_accel_data(uint8_t array);
static void get_INS_gyro_data(uint8_t array);
static void get_INS_angle_data(uint8_t array);
static void get_INS_mag_data(uint8_t array);
static void get_pitchmeasure(uint8_t array);
static void get_yawmeasure(uint8_t array);
static void get_remote_control(uint8_t array);
static void usb_taskinit(usb_debug_e *usb_debug_init);
static void get_debugdata(void);
static void get_INS_quat_data(uint8_t array);
extern USBD_HandleTypeDef hUsbDeviceFS;
static void get_bullet_data(uint8_t array);
int32_t usb_count=0;


void usb_task(void const *argument)
{
    
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();
    usb_taskinit(&usb_debug);
    osDelay(1000);
    while (1)
    {
        get_debugdata();
        usb_count++;
        vTaskDelay(1);
    }
}

uint32_t system_time;
uint16_t a = 20;
static void get_debugdata(void)
{
    system_time = xTaskGetTickCount();
    usb_txbuf[0] = 0xF1;
    usb_txbuf[1] = system_time >> 24;
    usb_txbuf[2] = system_time >> 16;
    usb_txbuf[3] = system_time >> 8;
    usb_txbuf[4] = system_time;
		get_bullet_data(5);
    get_INS_gyro_data(9);
	  get_INS_accel_data(21);
    get_INS_mag_data(33);
    get_INS_angle_data(45);
		
    CDC_Transmit_FS(usb_txbuf, sizeof(usb_txbuf));
}


static void get_motormeasure(uint8_t chassis_number, uint8_t array)
{
    usb_txbuf[array] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.motor_chassis[chassis_number].chassis_motor_measure->temperate;
}
static void get_pitchmeasure(uint8_t array)
{
    usb_txbuf[array] = usb_debug.usb_pitch_motor->gimbal_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.usb_pitch_motor->gimbal_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.usb_pitch_motor->gimbal_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.usb_pitch_motor->gimbal_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.usb_pitch_motor->gimbal_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.usb_pitch_motor->gimbal_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.usb_pitch_motor->gimbal_motor_measure->temperate;
}
static void get_yawmeasure(uint8_t array)
{
    usb_txbuf[array] = usb_debug.usb_yaw_motor->gimbal_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.usb_yaw_motor->gimbal_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.usb_yaw_motor->gimbal_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.usb_yaw_motor->gimbal_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.usb_yaw_motor->gimbal_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.usb_yaw_motor->gimbal_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.usb_yaw_motor->gimbal_motor_measure->temperate;
}
static void get_triggermeasure(uint8_t array)
{
    usb_txbuf[array] = usb_debug.usb_trigger_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.usb_trigger_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.usb_trigger_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.usb_trigger_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.usb_trigger_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.usb_trigger_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.usb_trigger_motor_measure->temperate;
}
static void get_fric1measure(uint8_t array)
{
    usb_txbuf[array] = usb_debug.usb_fric1_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.usb_fric1_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.usb_fric1_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.usb_fric1_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.usb_fric1_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.usb_fric1_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.usb_fric1_motor_measure->temperate;
}
static void get_fric2measure(uint8_t array)
{
    usb_txbuf[array] = usb_debug.usb_fric2_motor_measure->ecd >> 8;
    usb_txbuf[array + 1] = usb_debug.usb_fric2_motor_measure->ecd;
    usb_txbuf[array + 2] = usb_debug.usb_fric2_motor_measure->speed_rpm >> 8;
    usb_txbuf[array + 3] = usb_debug.usb_fric2_motor_measure->speed_rpm;
    usb_txbuf[array + 4] = usb_debug.usb_fric2_motor_measure->given_current >> 8;
    usb_txbuf[array + 5] = usb_debug.usb_fric2_motor_measure->given_current;
    usb_txbuf[array + 6] = usb_debug.usb_fric2_motor_measure->temperate;
}

static void get_INS_angle_data(uint8_t array)
{
    union refree_4_byte_t INS_data[3];
    INS_data[0].f = *usb_debug.usb_INS_angle;
    INS_data[1].f = *(usb_debug.usb_INS_angle + 1);
    INS_data[2].f = *(usb_debug.usb_INS_angle + 2);

    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];
}
static void get_INS_accel_data(uint8_t array)
{
    union refree_4_byte_t INS_data[3];
    INS_data[0].f = *usb_debug.usb_INS_accel;
    INS_data[1].f = *(usb_debug.usb_INS_accel + 1);
    INS_data[2].f = *(usb_debug.usb_INS_accel + 2);

    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];
}
static void get_INS_mag_data(uint8_t array)
{
    union refree_4_byte_t INS_data[3];
    INS_data[0].f = *usb_debug.usb_INS_mag;
    INS_data[1].f = *(usb_debug.usb_INS_mag + 1);
    INS_data[2].f = *(usb_debug.usb_INS_mag + 2);

    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[2];
}
static void get_INS_gyro_data(uint8_t array)
{
    union refree_4_byte_t INS_data[3];
    INS_data[0].f = *usb_debug.usb_INS_gyro;
    INS_data[1].f = *(usb_debug.usb_INS_gyro + 1);
    INS_data[2].f = *(usb_debug.usb_INS_gyro + 2);

    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];
}

static void get_INS_quat_data(uint8_t array)
{
    union refree_4_byte_t INS_data[4];
    INS_data[0].f = *usb_debug.usb_INS_gyro;
    INS_data[1].f = *(usb_debug.usb_INS_gyro + 1);
    INS_data[2].f = *(usb_debug.usb_INS_gyro + 2);
		INS_data[4].f = *(usb_debug.usb_INS_gyro + 3);
	
	
    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];
	
	
	  usb_txbuf[array + 12] = INS_data[3].buf[0];
    usb_txbuf[array + 13] = INS_data[3].buf[1];
    usb_txbuf[array + 14] = INS_data[3].buf[2];
    usb_txbuf[array + 15] = INS_data[3].buf[3];
}
static void get_bullet_data(uint8_t array)
{
    union refree_4_byte_t INS_data;
    //INS_data.f = get_bullet_speed();
		INS_data.f = 27;
    usb_txbuf[array] = INS_data.buf[0];
    usb_txbuf[array + 1] = INS_data.buf[1];
    usb_txbuf[array + 2] = INS_data.buf[2];
    usb_txbuf[array + 3] = INS_data.buf[3];
}
static void get_remote_control(uint8_t array) //��ַ����ң��������˫��������ַ��
{
    uint8_t i = 0;
    for (i = 0; i < 18; i++)
    {
        usb_txbuf[array + i] = *(usb_debug.RC_buf + i);
    }
}

static void usb_taskinit(usb_debug_e *usb_debug_init)
{
    uint8_t i;
    //��ȡң����ָ��
    //get gyro sensor euler angle point

    //��ȡ��̨�������ָ��?1?7
    usb_debug_init->usb_yaw_motor = get_yaw_motor_point();
    usb_debug_init->usb_pitch_motor = get_pitch_motor_point();
    usb_debug_init->usb_trigger_motor_measure = get_trigger_motor_measure_point();
    usb_debug_init->usb_fric1_motor_measure = get_fric_motor_measure_point(1);
    usb_debug_init->usb_fric2_motor_measure = get_fric_motor_measure_point(2);
    usb_debug_init->usb_INS_accel = get_accel_data_point();
    usb_debug_init->usb_INS_angle = get_INS_angle_point();
    usb_debug_init->usb_INS_mag = get_mag_data_point();
    usb_debug_init->usb_INS_gyro = get_gyro_data_point();
    usb_debug_init->RC_buf = get_remote_buff_point();
		usb_debug_init->usb_INS_quat = get_INS_quat_point();
    //get chassis motor data point,  initialize motor speed PID
    //��ȡ���̵������ָ�룬��ʼ��PID
    for (i = 0; i < 4; i++)
    {
        usb_debug_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    }
}
