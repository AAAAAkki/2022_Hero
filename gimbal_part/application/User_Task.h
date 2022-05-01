#ifndef USER_TASK_H
#define USER_TASK_H

#include "remote_control.h"
#include "INS_task.h"
#include "CAN_Receive.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"
#include "vision.h"
typedef struct{
	const RC_ctrl_t *ui_rc_ctrl;
    gimbal_control_t *ui_gimbal_control;
    chassis_move_t *ui_chassis_move;
    shoot_control_t *ui_shoot_control;
    ext_robot_hurt_t *ui_robot_hurt;
    ext_game_robot_state_t *ui_robot_status;
    vision_control_t *ui_vision;

} UI_show_t;

typedef struct
{
	uint8_t ID[6];
	uint16_t sender_ID[6];
	uint16_t receiver_ID[6];
} id_data_t;
void UserTask(void const *pvParameters);



#endif
