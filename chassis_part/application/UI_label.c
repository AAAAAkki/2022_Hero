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

#include "CAN_receive.h"


uint8_t get_colour() {
    static uint8_t colour = 0;
    colour = (colour + 1) % 9;
    return colour;
}


UI_show_t ui;

String_Data image_0, image_1, image_2, image_3; // 这是信息表的提示栏
String_Data vision_0, vision_1, vision_3;  // 这是信息表的数据栏

uint8_t ui_cache_trigger_state = UINT8_MAX;
uint8_t ui_cache_fric_state = UINT8_MAX;
uint8_t ui_cache_spin_state = UINT8_MAX;
fp32 ui_cache_cap_state = UINT8_MAX;
fp32 ui_cache_pitch_angle = UINT8_MAX;


void label_draw(uint8_t optional) {

    uint8_t u8_temp;
    fp32 fp32_temp;

    //  !Form: 拨弹轮
    u8_temp = ui.ui_gimbal_data->shoot_mode == 2;
    if (u8_temp != ui_cache_trigger_state) {
        ui_cache_trigger_state = u8_temp;
        memset(&vision_0, 0, sizeof(vision_0));
        if (ui_cache_trigger_state) {  // ON
            Char_Draw(&vision_0, "203", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 860, "ON");
        } else {  // OFF
            Char_Draw(&vision_0, "203", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 860, "OFF");
        }
        Char_ReFresh(vision_0);
    }

    // !Form: 摩擦轮
    u8_temp = (ui.ui_gimbal_data->shoot_mode != 0 && ui.ui_gimbal_data->shoot_mode !=3);
    if (u8_temp != ui_cache_fric_state) {
        ui_cache_fric_state = u8_temp;
        memset(&vision_1, 0, sizeof(vision_1));
        if (ui_cache_fric_state) {
            Char_Draw(&vision_1, "204", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 830, "ON");
        } else {  // OFF
            Char_Draw(&vision_1, "204", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 830, "OFF");
        }
        Char_ReFresh(vision_1);
    }

    //  !Form: Pitch轴数据
//    fp32_temp = ui.ui_gimbal_data->pitch_angel_degree;
//    if (fabsf(ui_cache_pitch_angle - fp32_temp) >= 0.001) {
//        ui_cache_pitch_angle = fp32_temp;
//        char pitch_angle_value[12];
//        String_Data CH_PITCH_DATA;
//        memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
//        sprintf(pitch_angle_value, "%.1f", fp32_temp);
//        Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Yellow, 15, 6, 4, 280, 800, &pitch_angle_value[0]);
//        Char_ReFresh(CH_PITCH_DATA);
//    }

    // !Form: 小陀螺状态
//    u8_temp = ui.ui_gimbal_data->swing_flag;  // XXX: 有时候刷新错误  @Juntong(2022-7-15): 我真的不记得什么错误
//    if (u8_temp != ui_cache_spin_state) {
//        ui_cache_spin_state = u8_temp;
//        memset(&vision_3, 0, sizeof(vision_3));
//        if (ui_cache_spin_state == 1) {  // 小陀螺启动
//            Char_Draw(&vision_3, "201", optional, 1, UI_Color_Yellow, 15, 2, 4, 280, 770, "ON");
//        } else {
//            Char_Draw(&vision_3, "201", optional, 1, UI_Color_Yellow, 15, 3, 4, 280, 770, "OFF");
//        }
//        Char_ReFresh(vision_3);
//    }

    // !Form: 电容显示
    char string[8];
    get_cap_proportion(&fp32_temp);
    if (fabsf(fp32_temp - ui_cache_cap_state) >= 0.01) {  // 电容显示的精度是 0.01
        ui_cache_cap_state = fp32_temp;
        memset(&vision_3, 0, sizeof(vision_3));
        memset(string, 0, sizeof(char) * 8);
        sprintf(string, "%d", abs((int) (ui_cache_cap_state * 100)));
        Char_Draw(&vision_3, "205", optional, 1,
                  ui_cache_cap_state > 0 ? UI_Color_Cyan : UI_Color_Purplish_red,
                  28, 3, 4, 910, 200, string);
        Char_ReFresh(vision_3);
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

//    memset(&image_1, 0, sizeof(image_1));
//    Char_Draw(&image_1, "105", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 800, "PITCH");
//    Char_ReFresh(image_1);
//
//    memset(&image_2, 0, sizeof(image_2));
//    Char_Draw(&image_2, "106", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 4, 4, 60, 770, "SPIN");  // 小陀螺
//    Char_ReFresh(image_2);

    //更新数据
    ui.ui_chassis_move = get_chassis_point();  // 获取底盘数据
    ui.ui_gimbal_data = get_gimbal_data();  // 云台角度、射击模式、小陀螺模式
    ui.ui_gimbal_control = get_gimbal_point();
    ui.ui_shoot_control = get_shoot_point();
    ui.ui_robot_hurt = get_hurt_point();
    ui.ui_robot_status = get_robot_status_point();

    UI_label_cache_reset();

    label_draw(UI_Graph_ADD);
}


//状态更新
void UI_label_change() {
    label_draw(UI_Graph_Change);
}


void UI_label_cache_reset() {
    ui_cache_cap_state = UINT8_MAX;
    ui_cache_trigger_state = UINT8_MAX;
    ui_cache_pitch_angle = UINT8_MAX;
    ui_cache_fric_state = UINT8_MAX;
    ui_cache_spin_state = UINT8_MAX;
}
