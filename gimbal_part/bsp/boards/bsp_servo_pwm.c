#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void servo_pwm_set(uint16_t pwm)
{

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
}
