/**
  ****************************辽宁科技大学COD****************************
  * @file       user_task.c/h-雨落长安
  * @brief      一个普通程序，请大家友善对待，谢谢大家，写的demo，未来得及封装，凑合看吧
  ==============================================================================
  @endverbatim
  ****************************辽宁科技大学COD****************************
  */

#include "User_Task.h"
#include "main.h"
#include "math.h"
#include "RM_Cilent_UI.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"
#include "cmsis_os.h"
#include "string.h"
#include <stdio.h>
#include "detect_task.h"
extern vision_control_t vision_control;
#include "vision.h"
#include "CAN_Receive.h" //加载摩擦轮发射闭环真实转速
#define PI 3.1415936
void UI_send_init(UI_show_t *init);
extern uint16_t Robot_number;
extern uint16_t Robot_cline_number;
id_data_t id_data;
Graph_Data G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11, G12, G13, F2;
String_Data F1;
String_Data CH_FLRB;
String_Data CH_PITCH;
String_Data CH_FRIC;
String_Data CH_ON;
String_Data CH_OFF;
String_Data CH_OFFLINE;
String_Data CH_VISION;
String_Data CH_TRIGGER;
String_Data CH_TOP;
String_Data CH_PITCH_DATA;
Graph_Data vision_state;
String_Data CH_VISION_MODE;
UI_show_t UI_show;
char flrb_arr[4] = "FRBL";
char fric[4] = "FRIC";
char vision[6] = "VISION";
char trigger[7] = "TRIGGER";
char offline[7] = "OFFLINE";
char on[2] = "ON";
char off[3] = "OFF";
char top[3] = "TOP";
char pitch_angle[11] = "PITCH";
char windmill[8] = "WINDMILL";
char armour[6] = "ARMOUR";
char pitch_error[11] = "PITCH ERROR";
void ui_char_refresh(UI_show_t *ui_char);
void robot_id_data_init(void);
void robot_id_select(void);
void aiming_line_refresh(UI_show_t *aming_line);
void ui_name_refresh(void);
void robot_images_refresh(void);
void UserTask(void const *pvParameters)
{

	UI_send_init(&UI_show);
	
	while (1)
	{
		robot_id_select(); //保证热插拔，每次任务都选择一次ID
		ui_name_refresh();
		ui_char_refresh(&UI_show);
		aiming_line_refresh(&UI_show);
		robot_images_refresh();
		vTaskDelay(50);
	}
}
void UI_send_init(UI_show_t *init)
{
	init->ui_gimbal_control = get_gimbal_point();
	init->ui_rc_ctrl = get_remote_control_point();
	init->ui_chassis_move = get_chassis_point();
	init->ui_shoot_control = get_shoot_point();
	init->ui_robot_hurt = get_hurt_point();
	init->ui_vision = get_vision_data();
	init->ui_robot_status = get_robot_status_point();
	vTaskDelay(100);
	robot_id_data_init();
	robot_id_select();
	char pitch_angle_value[32];
	memset(&G1, 0, sizeof(G1));				//中心垂线
	memset(&G2, 0, sizeof(G2));				//上击打线
	memset(&G3, 0, sizeof(G3));				//中心水平线
	memset(&G4, 0, sizeof(G4));				//枪管轴心线
	memset(&G5, 0, sizeof(G5));				//下击打线
	memset(&G6, 0, sizeof(G6));				//远距离击打线
	memset(&G7, 0, sizeof(G7));				//摩擦轮状态
	memset(&G8, 0, sizeof(G8));				//前装甲板状态
	memset(&G9, 0, sizeof(G9));				//左装甲板状态
	memset(&G10, 0, sizeof(G10));			//右装甲板状态
	memset(&G11, 0, sizeof(G11));			//后装甲板状态
	memset(&CH_PITCH, 0, sizeof(CH_PITCH)); //PITCH-角度
	memset(&CH_FLRB, 0, sizeof(CH_FLRB));	//装甲板标识
	memset(&CH_FRIC, 0, sizeof(CH_FRIC));
	memset(&CH_OFF, 0, sizeof(CH_OFF));
	memset(&CH_ON, 0, sizeof(CH_ON));
	memset(&CH_VISION, 0, sizeof(CH_VISION));
	memset(&CH_TOP, 0, sizeof(CH_TOP));
	memset(&CH_OFFLINE, 0, sizeof(CH_OFFLINE));
	memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
	memset(&CH_VISION_MODE, 0, sizeof(CH_VISION_MODE));

	memset(&pitch_angle_value, 0, sizeof(pitch_angle_value));

	fp32 pitch_angle = init->ui_gimbal_control->gimbal_pitch_motor.absolute_angle;
	sprintf(pitch_angle_value, "%f", pitch_angle);
	Char_Draw(&CH_PITCH_DATA, "022", UI_Graph_ADD, 8, UI_Color_Yellow, 26, 10, 4, 1600, 540, &pitch_angle_value[0]);
	Char_ReFresh(CH_PITCH_DATA);

	Char_Draw(&CH_OFF, "062", UI_Graph_ADD, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 860, &off[0]);
	Char_ReFresh(CH_OFF);
	Char_Draw(&CH_OFFLINE, "061", UI_Graph_ADD, 8, UI_Color_Purplish_red, 26, 7, 4, 280, 800, &offline[0]);
	Char_ReFresh(CH_OFFLINE);
	Char_Draw(&CH_OFF, "060", UI_Graph_ADD, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 740, &off[0]);
	Char_ReFresh(CH_OFF);
	Char_Draw(&CH_OFF, "059", UI_Graph_ADD, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 680, &off[0]);
	Char_ReFresh(CH_OFF);

}
void ui_name_refresh(void)
{
	static uint8_t time;

	if (time )
	{
		time--;
	}
	if(!time)
{
	Char_Draw(&CH_FRIC, "067", UI_Graph_ADD, 8, UI_Color_Yellow, 26, 4, 4, 60, 860, &fric[0]);
	Char_Draw(&CH_VISION, "066", UI_Graph_ADD, 8, UI_Color_Yellow, 26, 6, 4, 60, 800, &vision[0]);
	Char_Draw(&CH_TRIGGER, "065", UI_Graph_ADD, 8, UI_Color_Yellow, 26, 7, 4, 60, 740, &trigger[0]);
	Char_Draw(&CH_TOP, "064", UI_Graph_ADD, 8, UI_Color_Yellow, 26, 3, 4, 60, 680, &top[0]);

	Char_ReFresh(CH_FRIC);
	Char_ReFresh(CH_VISION);
	Char_ReFresh(CH_TRIGGER);
	Char_ReFresh(CH_TOP);
	time=5;
}

}
void ui_char_refresh(UI_show_t *ui_char)
{

		static uint8_t time;

	if (time )
	{
		time--;
	}
	if(!time)
{
	if (toe_is_error(FRIC_RIGHT_MOTOR_TOE) && toe_is_error(FRIC_LEFT_MOTOR_TOE))
	{
		Char_Draw(&CH_OFFLINE, "062", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 2, 4, 280, 860, &offline[0]);
		Char_ReFresh(CH_OFFLINE);
	}
	else
	{
		if (ui_char->ui_shoot_control->shoot_mode == SHOOT_READY)
		{
			Char_Draw(&CH_ON, "062", UI_Graph_Change, 8, UI_Color_Green, 26, 2, 4, 280, 860, &on[0]);
			Char_ReFresh(CH_ON);
		}
		else if (ui_char->ui_shoot_control->shoot_mode == SHOOT_STOP)
		{
			Char_Draw(&CH_OFF, "062", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 860, &off[0]);
			Char_ReFresh(CH_OFF);
		}
		else if (ui_char->ui_shoot_control->shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			Char_Draw(&CH_ON, "062", UI_Graph_Change, 8, UI_Color_Green, 30, 2, 4, 280, 860, &on[0]);
			Char_ReFresh(CH_ON);
		}
	}

	if (toe_is_error(TRIGGER_MOTOR_TOE))
	{
		Char_Draw(&CH_OFFLINE, "060", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 740, &offline[0]);
		Char_ReFresh(CH_OFFLINE);
	}
	else
	{
		if (ui_char->ui_shoot_control->shoot_mode == SHOOT_READY)
		{
			Char_Draw(&CH_OFF, "060", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 740, &off[0]);
			Char_ReFresh(CH_OFF);
		}
		else if (ui_char->ui_shoot_control->shoot_mode == SHOOT_STOP)
		{
			Char_Draw(&CH_OFF, "060", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 740, &off[0]);
			Char_ReFresh(CH_OFF);
		}
		else if (ui_char->ui_shoot_control->shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			Char_Draw(&CH_ON, "060", UI_Graph_Change, 8, UI_Color_Green, 26, 2, 4, 280, 740, &on[0]);
			Char_ReFresh(CH_ON);
		}
	}

	if (ui_char->ui_chassis_move->swing_flag == 1)
	{
		Char_Draw(&CH_ON, "059", UI_Graph_Change, 8, UI_Color_Green, 26, 3, 4, 280, 680, &on[0]);
		Char_ReFresh(CH_ON);
	}
	else
	{
		Char_Draw(&CH_OFF, "059", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 3, 4, 280, 680, &off[0]);
		Char_ReFresh(CH_OFF);
	}

	if (toe_is_error(VISION))
	{
		Char_Draw(&CH_VISION_MODE, "061", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 7, 4, 280, 800, &offline[0]);
		Char_ReFresh(CH_VISION_MODE);
	}
	else if (ui_char->ui_vision->vision_mode == 0)
	{
		Char_Draw(&CH_VISION_MODE, "061", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 7, 4, 280, 800, &armour[0]);
		Char_ReFresh(CH_VISION_MODE);
	}
	else if (ui_char->ui_vision->vision_mode == 1)
	{
		Char_Draw(&CH_VISION_MODE, "061", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 7, 4, 280, 800, &windmill[0]);
		Char_ReFresh(CH_VISION_MODE);
	}

	fp32 pitch_angle = ui_char->ui_gimbal_control->gimbal_pitch_motor.absolute_angle;
	if (!toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
	{
		char pitch_angle_value[12];
		sprintf(pitch_angle_value, "%f", pitch_angle);
		Char_Draw(&CH_PITCH_DATA, "022", UI_Graph_Change, 8, UI_Color_Yellow, 26, 10, 4, 1600, 540, &pitch_angle_value[0]);
		Char_ReFresh(CH_PITCH_DATA);
	}
	else
	{
		Char_Draw(&CH_PITCH_DATA, "022", UI_Graph_Change, 8, UI_Color_Purplish_red, 26, 11, 4, 1600, 540, &pitch_error[0]);
		Char_ReFresh(CH_PITCH_DATA);
	}
		time=5;
}
}
void armour_refresh(UI_show_t *armour)
{

	double angle = (double)armour->ui_gimbal_control->gimbal_yaw_motor.relative_angle;
	Circle_Draw(&G8, "081", UI_Graph_ADD, 8, UI_Color_Green, 3, 960 + (int)340 * sin(angle), 540 + (int)340 * cos(angle), 50);
	Circle_Draw(&G9, "082", UI_Graph_ADD, 8, UI_Color_Green, 3, 960 + (int)340 * sin(angle + PI / 2), 540 + (int)340 * cos(angle + PI / 2), 50);
	Circle_Draw(&G10, "083", UI_Graph_ADD, 8, UI_Color_Green, 3, 960 + (int)340 * sin(angle + PI), 540 + (int)340 * cos(angle + PI), 50);
	Circle_Draw(&G11, "084", UI_Graph_ADD, 8, UI_Color_Green, 3, 960 + (int)340 * sin(angle + 3 * PI / 2), 540 + (int)340 * cos(angle + 3 * PI / 2), 50);
	UI_ReFresh(5, G7, G8, G9, G10, G11); //G7缺省

	Char_Draw(&CH_FLRB, "077", UI_Graph_ADD, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle)*2 * PI / 360.0), 540 + (int)340 * cos((angle)*2 * PI / 360.0), &flrb_arr[0]);
	Char_ReFresh(CH_FLRB);
	Char_Draw(&CH_FLRB, "076", UI_Graph_ADD, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 90) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 90) * 2 * PI / 360.0), &flrb_arr[1]);
	Char_ReFresh(CH_FLRB);
	Char_Draw(&CH_FLRB, "075", UI_Graph_ADD, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 180) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 180) * 2 * PI / 360.0), &flrb_arr[2]);
	Char_ReFresh(CH_FLRB);
	Char_Draw(&CH_FLRB, "074", UI_Graph_ADD, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 270) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 270) * 2 * PI / 360.0), &flrb_arr[3]);
	Char_ReFresh(CH_FLRB);
}

void aiming_line_refresh(UI_show_t *aming_line)
{
	static uint8_t time;

	if (time < 2)
	{
		time++;
	}
	else
	{
		if (aming_line->ui_robot_status->shooter_id1_17mm_speed_limit == 15)
		{
			Line_Draw(&G1, "091", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 960, 200, 960, 500);
			Line_Draw(&G2, "092", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 940, 460, 980, 460);
			Line_Draw(&G3, "093", UI_Graph_ADD, 9, UI_Color_Green, 2, 940, 430, 980, 430);
			Line_Draw(&G4, "094", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 920, 405, 1000, 405);
			Line_Draw(&G5, "095", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 880, 340, 1040, 340);
			//Line_Draw(&G6, "096", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 860, 220, 1020, 220);
			Line_Draw(&G7, "097", UI_Graph_ADD, 9, UI_Color_Green, 2, 920, 370, 1000, 370);
			UI_ReFresh(7, G1, G2, G3, G4, G5, G6, G7);

		time=0;
			
		}
		else if (aming_line->ui_robot_status->shooter_id1_17mm_speed_limit == 18)
		{
			Line_Draw(&G1, "091", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 960, 200, 960, 500);
			Line_Draw(&G2, "092", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 940, 460, 980, 460);
			Line_Draw(&G3, "093", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 940, 430, 980, 430);
			Line_Draw(&G4, "094", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 920, 400, 1000, 400);
			Line_Draw(&G5, "095", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 900, 370, 1020, 370);
			Line_Draw(&G6, "096", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 880, 340, 1040, 340);
			UI_ReFresh(5, G1, G2, G3, G4, G5);
			UI_ReFresh(1,G6);
					time=0;
		}
		else if (aming_line->ui_robot_status->shooter_id1_17mm_speed_limit == 30)
		{
			Line_Draw(&G1, "091", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 960, 200, 960, 500);
			Line_Draw(&G2, "092", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 940, 460, 980, 460);
			//Line_Draw(&G3, "093", UI_Graph_ADD, 9, UI_Color_Green, 3, 940, 430, 980, 430);
			Line_Draw(&G4, "094", UI_Graph_ADD, 9, UI_Color_Green, 2, 920, 405, 1000, 405);
			//Line_Draw(&G4, "094", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 920, 405, 1000, 405);
			Line_Draw(&G5, "095", UI_Graph_ADD, 9, UI_Color_Purplish_red, 2, 880, 340, 1040, 340);
			//Line_Draw(&G6, "096", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 860, 220, 1020, 220);
			//Line_Draw(&G7, "097", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 920, 370, 1000, 370);
			UI_ReFresh(2, G1, G2);
			UI_ReFresh(2, G4, G5);
		}
		time = 0;
	}
}
void robot_id_data_init(void)
{
	id_data.ID[0] = 3;
	id_data.ID[1] = 4;
	id_data.ID[2] = 5;

	id_data.ID[3] = 103;
	id_data.ID[4] = 104;
	id_data.ID[5] = 105;

	id_data.sender_ID[0] = 3;
	id_data.sender_ID[1] = 4;
	id_data.sender_ID[2] = 5;

	id_data.sender_ID[3] = 103;
	id_data.sender_ID[4] = 104;
	id_data.sender_ID[5] = 105;

	id_data.receiver_ID[0] = 0x103;
	id_data.receiver_ID[1] = 0x104;
	id_data.receiver_ID[2] = 0x105;

	id_data.receiver_ID[3] = 0x167;
	id_data.receiver_ID[4] = 0x168;
	id_data.receiver_ID[5] = 0x169;
}

void robot_id_select(void)
{
	Uint8_t i = 0;
	Robot_number = get_robot_id();
	for (i = 0; i <= 5; i++)
	{
		if (Robot_number == id_data.ID[i])
		{
			Robot_cline_number = id_data.receiver_ID[i];
		}
	}
}

void robot_images_refresh(void)
{
	if(toe_is_error(VISION))
	{
	Circle_Draw(&vision_state,"001",UI_Graph_ADD,2,UI_Color_Orange,5,1000,680,5);
	UI_ReFresh(1,vision_state);
	}
	else if(vision_control.vision_data.rx[0].target_flag&&!UI_show.ui_rc_ctrl->mouse.press_r)
	{
	Circle_Draw(&vision_state,"001",UI_Graph_ADD,2,UI_Color_Cyan,5,1000,680,5);
	UI_ReFresh(1,vision_state);
	}
	else if(vision_control.vision_data.rx[0].target_flag&&UI_show.ui_rc_ctrl->mouse.press_r)
	{
	Circle_Draw(&vision_state,"001",UI_Graph_ADD,2,UI_Color_Green,5,1000,680,5);
	UI_ReFresh(1,vision_state);
	}
}
