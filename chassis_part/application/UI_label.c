//Miao 
/*使用说明：
 * 标签控件用于显示文字类信息。
 * 使用 UI_label_static 初始化
 * 初始化生成文字基本格式
 * 使用 UI_label_change 更新状态
 * 状态来源为“视觉”数据输入 对数据进行判断后 将储存于结构体 UI_label_data
*/

#include "UI_label.h"
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

#include "gimbal_task.h"
#include "vision.h"
#include "servo_task.h"

UI_show_t ui;

String_Data image_1, image_2, image_3, image_4, image_5, image_6; // 这是信息表的提示栏
String_Data vision_0, vision_1, vision_4;  // 这是信息表的数据栏


void label_draw(uint8_t optional) {

    //更新数据
    ui.ui_vision = get_vision_data();
    ui.ui_chassis_move = get_chassis_point();
		ui.ui_gimbal_data = get_gimbal_data();
    ui.ui_gimbal_control = get_gimbal_point();
    ui.ui_shoot_control = get_shoot_point();
    ui.ui_robot_hurt = get_hurt_point();
    ui.ui_robot_status = get_robot_status_point();

    //  !Form: 拨弹轮
    String_Data vision_5;
    switch (get_trigger_state()) {
        case 0:  // OFF
            memset(&vision_5, 0, sizeof(vision_5));
            Char_Draw(&vision_5, "203", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 860, "OFF");
            Char_ReFresh(vision_5);
            break;
        case 1:  // ON
            memset(&vision_5, 0, sizeof(vision_5));
            Char_Draw(&vision_5, "203", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 860, "ON");
            Char_ReFresh(vision_5);
            break;
        case 3:  // 离线
            memset(&vision_5, 0, sizeof(vision_5));
            Char_Draw(&vision_5, "203", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 860, "off-line");
            Char_ReFresh(vision_5);
            break;
        default:  // 理论上不会遇到
            break;
    }

    // !Form: 摩擦轮
    String_Data vision_6;
    switch (get_fric_state()) {
        case 0:  // OFF
            memset(&vision_6, 0, sizeof(vision_6));
            Char_Draw(&vision_6, "204", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 830, "OFF");
            Char_ReFresh(vision_6);
            break;
        case 1:  // ON
            memset(&vision_6, 0, sizeof(vision_6));
            Char_Draw(&vision_6, "204", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 830, "ON");
            Char_ReFresh(vision_6);
            break;
        case 3:  // 离线
            memset(&vision_6, 0, sizeof(vision_6));
            Char_Draw(&vision_6, "204", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 830, "off-line");
            Char_ReFresh(vision_6);
            break;
        default:  // 理论上不会遇到
            break;
    }

    //  !Form: Pitch轴数据
    fp32 pitch_angle = ui.ui_gimbal_data->pitch_angel_degree;
    if (!toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) {
        char pitch_angle_value[12];
        String_Data CH_PITCH_DATA;
        memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
        sprintf(pitch_angle_value, "%.4f", pitch_angle);
        Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Yellow, 15, 6, 4, 280, 800, &pitch_angle_value[0]);
        Char_ReFresh(CH_PITCH_DATA);
    } else {
        String_Data CH_PITCH_DATA;
        memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
        Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Cyan, 15, 5, 4, 280, 800, "ERROR");
        Char_ReFresh(CH_PITCH_DATA);
    }

    // !Form: 小陀螺状态
    if (ui.ui_gimbal_data->swing_flag == 1) {  // 小陀螺启动  FIXME: 只有键盘事件才会响应，待更新
        String_Data vision_2;
        memset(&vision_2, 0, sizeof(vision_2));
        Char_Draw(&vision_2, "201", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 770, "ON");
        Char_ReFresh(vision_2);
    } else {
        String_Data vision_3;
        memset(&vision_3, 0, sizeof(vision_3));
        Char_Draw(&vision_3, "201", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 770, "OFF");
        Char_ReFresh(vision_3);
    }

    //  !Form:  弹舱盖数据判断
    if (get_cover_state() == 1) {  // 弹仓关
        memset(&vision_4, 0, sizeof(vision_4));
        Char_Draw(&vision_4, "202", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 740, "OFF");
        Char_ReFresh(vision_4);
    } else {  // 弹舱开
        memset(&vision_4, 0, sizeof(vision_4));
        Char_Draw(&vision_4, "202", optional, 1, UI_Color_Yellow, 15, 4, 4, 280, 740, "ON");
        Char_ReFresh(vision_4);
    }

    //  !Form: 视觉数据判断  FIXME: 没有无识别状态
    if (ui.ui_vision->vision_mode == 0) {  // 识别到装甲板  FIXME: 根据枚举值，0应该是风车
        memset(&vision_0, 0, sizeof(vision_0));
        Char_Draw(&vision_0, "200", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 710, "ARMOUR");
        Char_ReFresh(vision_0);
    } else if (ui.ui_vision->vision_mode == 1) {  // 识别到风车  FIXME: 根据枚举值，1应该是装甲板
        memset(&vision_1, 0, sizeof(vision_1));
        Char_Draw(&vision_1, "200", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 710, "WINDMILL");
        Char_ReFresh(vision_1);
    } else {
        memset(&vision_4, 0, sizeof(vision_4));
        Char_Draw(&vision_4, "200", optional, 1, UI_Color_Cyan, 15, 8, 4, 280, 710, "OFFLine");
        Char_ReFresh(vision_4);
    }
}


//初始化
void UI_label_static() {
    memset(&image_3, 0, sizeof(image_3));
    Char_Draw(&image_3, "103", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 860, "TRIGGER");
    Char_ReFresh(image_3);

    memset(&image_4, 0, sizeof(image_4));
    Char_Draw(&image_4, "104", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 830, "FIRC");
    Char_ReFresh(image_4);

    memset(&image_5, 0, sizeof(image_5));
    Char_Draw(&image_5, "105", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 800, "PITCH");
    Char_ReFresh(image_5);

    memset(&image_6, 0, sizeof(image_6));
    Char_Draw(&image_6, "106", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 4, 4, 60, 770, "SPIN");
    Char_ReFresh(image_6);

    memset(&image_1, 0, sizeof(image_1));
    Char_Draw(&image_1, "101", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 740, "VISION");
    Char_ReFresh(image_1);

    memset(&image_2, 0, sizeof(image_2));
    Char_Draw(&image_2, "102", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 710, "COVER");
    Char_ReFresh(image_2);

    label_draw(UI_Graph_ADD);
}


//状态更新
void UI_label_change() {
    label_draw(UI_Graph_Change);
}