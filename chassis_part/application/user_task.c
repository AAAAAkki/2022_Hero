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
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"
#include "cmsis_os.h"
#include "string.h"
#include <stdio.h>
#include "detect_task.h"
#include "RM_Cilent_UI.h"
#include "UI_ProgressBar.h"
#include "UI_car.h"
#include "UI_label.h"

#include "voltage_task.h"

extern vision_control_t vision_control;

#include "vision.h"
#include "CAN_receive.h" //加载摩擦轮发射闭环真实转速

#define PI 3.1415936
#define cbc car.basic_config

#define set_name(d, s1, s2, s3) do{\
                                    (d)[0] = (uint8_t) (s1); \
                                    (d)[1] = (uint8_t) (s2); \
                                    (d)[2] = (uint8_t) (s3); \
                                } while(0)


void UI_send_init();

void UI_car_init();

void UI_car_change();

extern uint16_t Robot_number;
extern uint16_t Robot_cline_number;
extern UI_show_t ui;


car_handle car;

progress_bar_data bar;

id_data_t id_data;

void ui_char_refresh(void);

void robot_id_data_init(void);

void robot_id_select(void);

void ui_refresh(void);

void robot_images_refresh(void);

void UI_aimline();

void UI_car_static(void);

void UserTask(void const *pvParameters) {
    static uint8_t time = 0;
    int16_t batter_percentage = 0;

    memset(&bar, 0, sizeof(bar));
    memset(&car, 0, sizeof(car));

    UI_send_init();
    UI_label_static();
    UI_car_init();
    UI_ProgressBar_static(&bar);
    UI_car_static();
    UI_aimline();
    while (1) {
//        batter_percentage = get_cap_percent();
//        bar.progress_bar_data_change = (batter_percentage > 0) ? batter_percentage : 0;


        // 刷新
        time = (time + 1) % 64;
        if (time == 0) {
            UI_label_static();  // 重新加载数据表格
            UI_ProgressBar_static(&bar);  // 重新加载超级电容显示
            UI_aimline();  // 重新绘制瞄准线
            UI_car_static();
        } else {
            UI_label_change();
            UI_ProgressBar_change(&bar);
            UI_car_change();
        }


        robot_images_refresh();
        robot_id_select(); //保证热插拔，每次任务都选择一次ID

        vTaskDelay(50);
    }
}

void UI_car_init() {
    /*** 小车 ***/
/*小车定位*/
    cbc.central_x = 1700;
    cbc.central_y = 700;
/*其他*/
    cbc.full_radius = 110;
    cbc.body_half_length = 70;
    cbc.body_half_width = 60;
    cbc.rear_half_width = 30;
    cbc.head_radius = 35;

    cbc.drawing_width = 2;
    cbc.normal_colour_code = UI_Color_Yellow;
    cbc.attacked_colour_code = UI_Color_Orange;
    cbc.head_layer = 1;
    cbc.body_layer = 2;

    set_name(cbc.body_name_front, '3', '0', '0');
    set_name(cbc.body_name_back, '3', '0', '1');
    set_name(cbc.body_name_left, '3', '0', '2');
    set_name(cbc.body_name_right, '3', '0', '3');
    set_name(cbc.head_name_line, '3', '0', '4');
    set_name(cbc.head_name_circle, '3', '0', '5');
    set_name(cbc.rear_name_back, '3', '0', '6');
    set_name(cbc.rear_name_left, '3', '0', '7');
    set_name(cbc.rear_name_right, '3', '0', '8');

//    car.body_degree = get_body_degree() + 180;
    car.head_degree = 180;
    car.front_armor_showing_attacked = 0;  // 初始状态是不被击打
    car.back_armor_showing_attacked = 0;
    car.left_armor_showing_attacked = 0;
    car.right_armor_showing_attacked = 0;
}

void UI_car_static() {
    car_init_by_handle(&car);
}

//void UI_car_change() {
//    car_rotate_body(&car, get_body_degree() + 180);
//    car_front_armor_showing_attacked(&car, get_front_amour_attacked());
//    car_back_armor_showing_attacked(&car, get_back_amour_attackek());
//    car_left_armor_showing_attacked(&car, get_left_amour_attacked());
//    car_right_armor_showing_attacked(&car, get_right_amour_attackek());
//}

void UI_send_init() {

/*** 数据赋值 ***/
    robot_id_data_init();
    robot_id_select();

}


//瞄准辅助线
void UI_aimline() {

    Graph_Data line_1;
    memset(&line_1, 0, sizeof(line_1));
    Line_Draw(&line_1, "901", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 870, 450, 1050, 450);


    Graph_Data line_2;
    memset(&line_2, 0, sizeof(line_2));
    Line_Draw(&line_2, "902", UI_Graph_ADD, 1, UI_Color_Orange, 2, 870, 454, 1050, 454);

    Graph_Data line_3;
    memset(&line_3, 0, sizeof(line_3));
    Line_Draw(&line_3, "903", UI_Graph_ADD, 1, UI_Color_Orange, 2, 900, 467, 1020, 467);

    Graph_Data line_4;
    memset(&line_4, 0, sizeof(line_4));
    Line_Draw(&line_4, "904", UI_Graph_ADD, 1, UI_Color_Orange, 2, 930, 473, 990, 473);

    Graph_Data line_5;
    memset(&line_5, 0, sizeof(line_5));
    Line_Draw(&line_5, "905", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 870, 421, 1050, 421);

    Graph_Data line_6;
    memset(&line_6, 0, sizeof(line_6));
    Line_Draw(&line_6, "906", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 930, 448, 990, 448);

    Graph_Data line_7;
    memset(&line_7, 0, sizeof(line_7));
    Line_Draw(&line_7, "907", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 900, 483, 1020, 483);

    UI_ReFresh(7, line_1, line_2, line_3, line_4, line_5, line_6, line_7);


    Graph_Data line_8;
    memset(&line_8, 0, sizeof(line_8));
    Line_Draw(&line_8, "908", UI_Graph_ADD, 1, UI_Color_Pink, 2, 870, 405, 1050, 405);

    Graph_Data line_9;
    memset(&line_9, 0, sizeof(line_9));
    Line_Draw(&line_9, "909", UI_Graph_ADD, 1, UI_Color_Pink, 2, 900, 432, 1020, 432);

    Graph_Data line_10;
    memset(&line_10, 0, sizeof(line_10));
    Line_Draw(&line_10, "910", UI_Graph_ADD, 1, UI_Color_Pink, 2, 930, 473, 990, 473);

    Graph_Data line_11;
    memset(&line_11, 0, sizeof(line_11));
    Line_Draw(&line_11, "911", UI_Graph_ADD, 1, UI_Color_Green, 2, 870, 462, 1050, 462);

    Graph_Data line_12;
    memset(&line_12, 0, sizeof(line_12));
    Line_Draw(&line_12, "912", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 870, 442, 1050, 442);

//    Graph_Data line_13;
//    memset(&line_13, 0, sizeof(line_13));
//    Line_Draw(&line_13, "913", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 900, 300, 1020, 300);

//    Graph_Data line_14;
//    memset(&line_14, 0, sizeof(line_14));
//    Line_Draw(&line_14, "914", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 930, 280, 990, 280);

    UI_ReFresh(5, line_8, line_9, line_10, line_11, line_12);

//    Graph_Data line_15;
//    memset(&line_15, 0, sizeof(line_15));
//    Line_Draw(&line_15, "915", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 960, 560, 960, 260);
//    UI_ReFresh(1, line_15);

    //画个刹车线
    Graph_Data line_16;
    Graph_Data line_17;
    memset(&line_16, 0, sizeof(line_16));
    memset(&line_17, 0, sizeof(line_17));
    Line_Draw(&line_16, "916", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 490, 100, 840, 400);
    Line_Draw(&line_17, "917", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 1410, 100, 1060, 400);

    UI_ReFresh(2, line_16, line_17);
}



void robot_id_data_init(void) {
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

void robot_id_select(void) {
    Uint8_t i = 0;
    Robot_number = get_robot_id();
    for (i = 0; i <= 5; i++) {
        if (Robot_number == id_data.ID[i]) {
            Robot_cline_number = id_data.receiver_ID[i];
        }
    }
}

void robot_images_refresh(void) {
}
