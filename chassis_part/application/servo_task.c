/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500

#define SERVO_INFAN3_OPEN 2150
#define SERVO_INFAN3_CLOSE 1150
#define SERVO_INFAN4_OPEN 2150
#define SERVO_INFAN4_CLOSE 1150

#ifdef INFAN3
int SERVO_OPEN = SERVO_INFAN3_OPEN;
int SERVO_CLOSE = SERVO_INFAN3_CLOSE;
#elif INFAN4
#define SERVO_OPEN SERVO_INFAN4_OPEN
#define SERVO_CLOSE SERVO_INFAN4_CLOSE
#else
#error please define INFAN NO for servo
#endif
  

#define PWM_DETAL_VALUE 10

const RC_ctrl_t *servo_rc;
volatile uint16_t pwm_set;
uint8_t open_flag = 0;
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const *argument)
{
    servo_rc = get_remote_control_point();

    uint16_t time = 0;
    pwm_set = SERVO_CLOSE;
    while (1)
    {
        if (time)
        {
            time--;
        }
        if (!time)
        {
//            if ( servo_rc->rc.ch[4] > 100 &&servo_rc->rc.ch[4] <=660)
//            {
//                time = 100;
//                open_flag = 1;
//                pwm_set = SERVO_OPEN;
//            }
//            else if ( servo_rc->rc.ch[4] < -100&&servo_rc->rc.ch[4] >=-660)
//            {
//                time = 100;
//                open_flag = 0;
//                pwm_set = SERVO_CLOSE;
//            }
//						
					if(servo_rc->key.v & KEY_PRESSED_OFFSET_V )
					{
								time = 100;
                open_flag = 0;
                pwm_set = SERVO_OPEN;	
							
					}			
					else if(servo_rc->key.v & KEY_PRESSED_OFFSET_B )
					{						 
								time = 100;
                open_flag = 1;
                pwm_set = SERVO_CLOSE;	
          }
     }
		servo_pwm_set(pwm_set);
		osDelay(10);
	}
}
