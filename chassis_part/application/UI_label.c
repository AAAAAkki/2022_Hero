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


UI_show_t ui;

String_Data image_0, image_1, image_2, image_3; // 这是信息表的提示栏
String_Data vision_0, vision_1;  // 这是信息表的数据栏

uint8_t ui_cache_trigger_state = UINT8_MAX;
uint8_t ui_cache_fric_state = UINT8_MAX;
uint8_t ui_cache_spin_state = UINT8_MAX;
fp32 ui_cache_pitch_angle = UINT8_MAX;


void label_draw(uint8_t optional) {

    uint8_t temp;

    //  !Form: 拨弹轮
    temp = ui.ui_gimbal_data->shoot_mode == 2;
    if (temp != ui_cache_trigger_state) {
        ui_cache_trigger_state = temp;
        switch (ui_cache_trigger_state) {
            case 0:  // OFF
                memset(&vision_0, 0, sizeof(vision_0));
                Char_Draw(&vision_0, "203", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 860, "OFF");
                Char_ReFresh(vision_0);
                break;
            case 1:  // ON
                memset(&vision_0, 0, sizeof(vision_0));
                Char_Draw(&vision_0, "203", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 860, "ON");
                Char_ReFresh(vision_0);
                break;
            case 3:  // 离线
                memset(&vision_0, 0, sizeof(vision_0));
                Char_Draw(&vision_0, "203", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 860, "off-line");
                Char_ReFresh(vision_0);
                break;
            default:  // 理论上不会遇到
                break;
        }
    }

    // !Form: 摩擦轮
    temp = ui.ui_gimbal_data->shoot_mode != 0;
    if (temp != ui_cache_fric_state) {
        ui_cache_trigger_state = temp;
        switch (ui_cache_fric_state) {
            case 0:  // OFF
                memset(&vision_1, 0, sizeof(vision_1));
                Char_Draw(&vision_1, "204", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 830, "OFF");
                Char_ReFresh(vision_1);
                break;
            case 1:  // ON
                memset(&vision_1, 0, sizeof(vision_1));
                Char_Draw(&vision_1, "204", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 830, "ON");
                Char_ReFresh(vision_1);
                break;
            case 3:  // 离线
                memset(&vision_1, 0, sizeof(vision_1));
                Char_Draw(&vision_1, "204", optional, 1, UI_Color_Yellow, 15, 8, 4, 280, 830, "off-line");
                Char_ReFresh(vision_1);
                break;
            default:  // 理论上不会遇到
                break;
        }
    }

    //  !Form: Pitch轴数据
    fp32 pitch_angle = ui.ui_gimbal_data->pitch_angel_degree;
    if (fabsf(ui_cache_pitch_angle - pitch_angle) >= 0.001) {
        ui_cache_pitch_angle = pitch_angle;
        char pitch_angle_value[12];
        String_Data CH_PITCH_DATA;
        memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
        sprintf(pitch_angle_value, "%.3f", pitch_angle);
        Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Yellow, 15, 6, 4, 280, 800, &pitch_angle_value[0]);
        Char_ReFresh(CH_PITCH_DATA);
    }

    // !Form: 小陀螺状态
    temp = ui.ui_gimbal_data->swing_flag;
    if (temp != ui_cache_spin_state) {
        ui_cache_spin_state = temp;
        if (ui_cache_spin_state == 1) {  // 小陀螺启动
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
    }
}


//初始化
void UI_label_static() {
    memset(&image_3, 0, sizeof(image_3));
    Char_Draw(&image_3, "103", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 860, "TRIGGER");  // 拨弹轮
    Char_ReFresh(image_3);

    memset(&image_0, 0, sizeof(image_0));
    Char_Draw(&image_0, "104", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 830, "FIRC");  // 摩擦轮
    Char_ReFresh(image_0);

    memset(&image_1, 0, sizeof(image_1));
    Char_Draw(&image_1, "105", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 800, "PITCH");
    Char_ReFresh(image_1);

    memset(&image_2, 0, sizeof(image_2));
    Char_Draw(&image_2, "106", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 4, 4, 60, 770, "SPIN");  // 小陀螺
    Char_ReFresh(image_2);

    //更新数据
    ui.ui_chassis_move = get_chassis_point();  // 获取底盘数据
    ui.ui_gimbal_data = get_gimbal_data();  // 云台角度、射击模式、小陀螺模式
    ui.ui_gimbal_control = get_gimbal_point();
    ui.ui_shoot_control = get_shoot_point();
    ui.ui_robot_hurt = get_hurt_point();
    ui.ui_robot_status = get_robot_status_point();

    label_draw(UI_Graph_ADD);
}


//状态更新
void UI_label_change() {
    label_draw(UI_Graph_Change);
}