//
// Created by Miao on 2022/3/11
//

#ifndef UIGROUP_UI_LABEL_H
#define UIGROUP_UI_LABEL_H

#include <stdint.h>
#include "vision.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"


typedef struct {
    const RC_ctrl_t *ui_rc_ctrl;
    gimbal_control_t *ui_gimbal_control;
    gimbal_data_t *ui_gimbal_data;
    chassis_move_t *ui_chassis_move;
    shoot_control_t *ui_shoot_control;
    ext_robot_hurt_t *ui_robot_hurt;
    ext_game_robot_state_t *ui_robot_status;
} UI_show_t;

void UI_label_static();

void UI_label_change();

void UI_label_cache_reset();

#endif
