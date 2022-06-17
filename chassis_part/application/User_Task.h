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


typedef struct
{
	uint8_t ID[10];
	uint16_t sender_ID[10];
	uint16_t receiver_ID[10];
} id_data_t;
void UserTask(void const *pvParameters);


#endif
