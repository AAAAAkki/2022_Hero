/**
  ****************************(C) COPYRIGHT FREE********************************
  * @file       referee_receive.c/h
  * @brief      裁判系统数据处理，通过USART3串口收发裁判系统数据，利用DMA传输方式节
	*							约CPU资源，利用串口空闲中断来拉起处理函数。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            	Author          Modification
  *  V1.0.0     July-10-2019     Ziqi Yang          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT FREE********************************
  */

#include "referee_send.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "main.h"
#include "referee.h"
#include "detect_task.h"
#include "bsp_usart.h"

#include "cmsis_os.h"
#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"
#include "crc8_crc16.h"

//裁判系统串口初始化

ui_send_t ui_send;


void robot_id_data_init(void);
void robot_id_init(void);

graphic_add_t graphic_add;

void referee_task(void const *pvParameters)
{
	robot_id_data_init();
//	while (toe_is_error(REFEREE_TOE))
//	{
//	}
	robot_id_init();
	UI_init();

	while (1)
	{
		robot_id_init();
		UI_init();
		UI_show();
		vTaskDelay(100);
	}
}

void referee_to_usart6(char *sbus, uint16_t size)
{

	usart6_tx_dma_enable((uint8_t *)sbus, size);
}

//	uint8_t graphic_name[3]; //图形名,在删除，修改等操作中，作为客户端的索引。
//	uint32_t operate_tpye:3; //图形操作：0：空操作；1：增加；2：修改；3：删除；
//	uint32_t graphic_tpye:3; //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；
//	uint32_t layer:4; //图层数，0~9
//	uint32_t color:4; //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
//	uint32_t start_angle:9; //：起始角度，单位：°，范围[0,360]；
//	uint32_t end_angle:9; //终止角度，单位：°，范围[0,360]。
//	uint32_t width:10; //线宽；
//	uint32_t start_x:11; //起点x坐标；
//	uint32_t start_y:11; //起点y坐标。
//	uint32_t radius:10; //字体大小或者半径；
//	uint32_t end_x:11; //终点x坐标；
//	uint32_t end_y:11; //终点y坐标。
void UI_init(void)
{
	graphic_add.frame_header_struct_t.SOF = 0xA5;
	graphic_add.frame_header_struct_t.data_length = 111;
	graphic_add.frame_header_struct_t.seq = 1;
	append_CRC8_check_sum((uint8_t *)&graphic_add.frame_header_struct_t, sizeof(graphic_add.frame_header_struct_t));
	//////////////////////////////////////////////
	graphic_add.cmd_id = 0x0301;
	//////////////////////////////////////////////
	graphic_add.data_cmd_id = 0x0104;
	graphic_add.sender_ID = ui_send.robot_sender_ID;
	graphic_add.receiver_ID = ui_send.robot_receiver_ID;
	//////////////////////////////////////////////
	graphic_add.grap[0].graphic_name[0] = 0;
	graphic_add.grap[0].graphic_name[1] = 0;
	graphic_add.grap[0].graphic_name[2] = 1;

	graphic_add.grap[0].operate_tpye = 1;
	graphic_add.grap[0].graphic_tpye = 0;

	graphic_add.grap[0].layer = 1;
	graphic_add.grap[0].color = 3;
	graphic_add.grap[0].width = 4;

	graphic_add.grap[0].start_x = 805;
	graphic_add.grap[0].start_y = 100;
	graphic_add.grap[0].end_x = 805;
	graphic_add.grap[0].end_y = 800;
	//////////////////////////////////////////////
	graphic_add.grap[1].graphic_name[0] = 0;
	graphic_add.grap[1].graphic_name[1] = 0;
	graphic_add.grap[1].graphic_name[2] = 2;

	graphic_add.grap[1].operate_tpye = 1;
	graphic_add.grap[1].graphic_tpye = 0;

	graphic_add.grap[1].layer = 1;
	graphic_add.grap[1].color = 3;
	graphic_add.grap[1].width = 4;

	graphic_add.grap[1].start_x = 805;
	graphic_add.grap[1].start_y = 200;
	graphic_add.grap[1].end_x = 850;
	graphic_add.grap[1].end_y = 200;
	//////////////////////////////////////////////
	graphic_add.grap[2].graphic_name[0] = 0;
	graphic_add.grap[2].graphic_name[1] = 0;
	graphic_add.grap[2].graphic_name[2] = 3;

	graphic_add.grap[2].operate_tpye = 1;
	graphic_add.grap[2].graphic_tpye = 0;

	graphic_add.grap[2].layer = 1;
	graphic_add.grap[2].color = 8;
	graphic_add.grap[2].width = 4;

	graphic_add.grap[2].start_x = 805;
	graphic_add.grap[2].start_y = 300;
	graphic_add.grap[2].end_x = 850;
	graphic_add.grap[2].end_y = 300;
	//////////////////////////////////////////////
	graphic_add.grap[3].graphic_name[0] = 0;
	graphic_add.grap[3].graphic_name[1] = 0;
	graphic_add.grap[3].graphic_name[2] = 4;

	graphic_add.grap[3].operate_tpye = 1;
	graphic_add.grap[3].graphic_tpye = 0;

	graphic_add.grap[3].layer = 1;
	graphic_add.grap[3].color = 3;
	graphic_add.grap[3].width = 4;

	graphic_add.grap[3].start_x = 805;
	graphic_add.grap[3].start_y = 400;
	graphic_add.grap[3].end_x = 850;
	graphic_add.grap[3].end_y = 400;

	//////////////////////////////////////////////
	graphic_add.grap[4].graphic_name[0] = 0;
	graphic_add.grap[4].graphic_name[1] = 0;
	graphic_add.grap[4].graphic_name[2] = 5;

	graphic_add.grap[4].operate_tpye = 1;
	graphic_add.grap[4].graphic_tpye = 0;

	graphic_add.grap[4].layer = 1;
	graphic_add.grap[4].color = 8;
	graphic_add.grap[4].width = 4;

	graphic_add.grap[4].start_x = 805;
	graphic_add.grap[4].start_y = 500;
	graphic_add.grap[4].end_x = 850;
	graphic_add.grap[4].end_y = 500;
	//////////////////////////////////////////////
	graphic_add.grap[5].graphic_name[0] = 0;
	graphic_add.grap[5].graphic_name[1] = 0;
	graphic_add.grap[5].graphic_name[2] = 6;

	graphic_add.grap[5].operate_tpye = 1;
	graphic_add.grap[5].graphic_tpye = 0;

	graphic_add.grap[5].layer = 1;
	graphic_add.grap[5].color = 3;
	graphic_add.grap[5].width = 4;

	graphic_add.grap[5].start_x = 805;
	graphic_add.grap[5].start_y = 600;
	graphic_add.grap[5].end_x = 850;
	graphic_add.grap[5].end_y = 600;
	//////////////////////////////////////////////
	graphic_add.grap[6].graphic_name[0] = 0;
	graphic_add.grap[6].graphic_name[1] = 0;
	graphic_add.grap[6].graphic_name[2] = 7;

	graphic_add.grap[6].operate_tpye = 1;
	graphic_add.grap[6].graphic_tpye = 0;

	graphic_add.grap[6].layer = 1;
	graphic_add.grap[6].color = 8;
	graphic_add.grap[6].width = 4;

	graphic_add.grap[6].start_x = 805;
	graphic_add.grap[6].start_y = 700;
	graphic_add.grap[6].end_x = 850;
	graphic_add.grap[6].end_y = 700;

	//graphic_add.crc16=get_CRC16_check_sum((uint8_t*)&graphic_add, sizeof(graphic_add) - 2,0xffff);
}

void UI_show(void)
{
	append_CRC16_check_sum((uint8_t *)&graphic_add, sizeof(graphic_add));
	usart6_tx_dma_enable((uint8_t *)&graphic_add, sizeof(graphic_add));
}

void UI_send_init(void)
{
}

void robot_id_data_init(void)
{
	ui_send.id_data.ID[0] = 3;
	ui_send.id_data.ID[1] = 4;
	ui_send.id_data.ID[2] = 5;

	ui_send.id_data.ID[3] = 103;
	ui_send.id_data.ID[4] = 104;
	ui_send.id_data.ID[5] = 105;

	ui_send.id_data.sender_ID[0] = 3;
	ui_send.id_data.sender_ID[1] = 4;
	ui_send.id_data.sender_ID[2] = 5;

	ui_send.id_data.sender_ID[3] = 103;
	ui_send.id_data.sender_ID[4] = 104;
	ui_send.id_data.sender_ID[5] = 105;

	ui_send.id_data.receiver_ID[0] = 0x103;
	ui_send.id_data.receiver_ID[1] = 0x104;
	ui_send.id_data.receiver_ID[2] = 0x105;

	ui_send.id_data.receiver_ID[3] = 0x167;
	ui_send.id_data.receiver_ID[4] = 0x168;
	ui_send.id_data.receiver_ID[5] = 0x169;
}

void robot_id_init(void)
{
	uint8_t i = 0;
	ui_send.robot_id = get_robot_id();
	for (i = 0; i <= 5; i++)
	{
		if (ui_send.robot_id == ui_send.id_data.ID[i])
		{
			ui_send.robot_receiver_ID = ui_send.id_data.receiver_ID[i];
			ui_send.robot_sender_ID = ui_send.id_data.sender_ID[i];
			break;
		}
	}
}
