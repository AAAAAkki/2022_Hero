//
// Created by Jessi on 2022/3/8.
//

#ifndef UIGROUP_UI_CAR_H
#define UIGROUP_UI_CAR_H
#endif //UIGROUP_UI_CAR_H

#include "stdint.h"

#include "RM_Cilent_UI.h"

#define GRAPHIC_NAME_LENGTH 3

#define TIMER_MAX 1000  // 被攻击后刷新计时器，该计时器时间大于TIME_MAX时，装甲板始终标记为未受击打。此数字不应超过 UINT16_MAX

typedef struct {
    // 这些内容需要手动设置，除自动设置，程序不会检查数字是否合理
    uint16_t full_radius;  // 整个控件的半径，同时是云台炮管的长度
    uint16_t body_half_length;  // 车辆部分的长度的一半
    uint16_t body_half_width;  // 车身宽度的一半
    uint16_t rear_half_width;  // 车尾部分的宽度的一半
    uint16_t head_radius;  // 云台半径
    uint16_t central_x;  // 整个车的中心的坐标横轴值
    uint16_t central_y;  // 整个车的中心的坐标纵轴值

    uint8_t drawing_width;  // 绘制用的线条宽度
    uint8_t normal_colour_code;  // 常规情况下车辆线条的颜色代码，代码含义见大疆官方手册
    uint8_t attacked_colour_code;  // 被攻击时显示的颜色
    uint8_t head_layer;  // 云台所在图层
    uint8_t body_layer;  // 车体所在图层
    char body_name_front[GRAPHIC_NAME_LENGTH];  // 车头线条的名字
    char body_name_left[GRAPHIC_NAME_LENGTH];  // 车体左侧线条的名字
    char body_name_right[GRAPHIC_NAME_LENGTH];  // 车体右边线条的名字
    char body_name_back[GRAPHIC_NAME_LENGTH];  // 车体后方线条的名字
    char rear_name_left[GRAPHIC_NAME_LENGTH];  // 车尾左侧线条的名字
    char rear_name_right[GRAPHIC_NAME_LENGTH];  // 车尾右侧线条的名字
    char rear_name_back[GRAPHIC_NAME_LENGTH];  // 车尾后方线条的名字
    char head_name_circle[GRAPHIC_NAME_LENGTH]; // 云台圆环的名字
    char head_name_line[GRAPHIC_NAME_LENGTH];  // 云台炮管的名字
} car_basic_config;

typedef struct {
    car_basic_config basic_config;  // 需要手动填写
    uint16_t body_degree;  // 车身旋转度数，范围[0, 359]，顺时针为正方向
    uint16_t head_degree;  // 云台旋转度数，范围[0, 359]，顺时针为正方向
    uint8_t left_armor_showing_attacked;  // 车身左侧装甲板是否显示受击打
    uint8_t right_armor_showing_attacked;  // 车身右侧装甲板是否显示受击打
    uint8_t front_armor_showing_attacked;  // 车身头部装甲板是否显示受击打
    uint8_t back_armor_showing_attacked;  // 车身背部装甲板是否显示受击打
    uint32_t front_armor_attacked_timer;  // 受击倒计时，倒计时清零时所有装甲板显示为未受击打
    uint32_t right_armor_attacked_timer;
    uint32_t back_armor_attacked_timer;
    uint32_t left_armor_attacked_timer;
    Graph_Data body_front_data;  // 车头线条的数据包
    Graph_Data body_left_data;  // 车体左侧线条的数据包
    Graph_Data body_right_data;  // 车体右边线条的数据包
    Graph_Data body_back_data;  // 车体后方线条的数据包
    Graph_Data rear_left_data;  // 车尾左侧线条的数据包
    Graph_Data rear_right_data;  // 车尾右侧线条的数据包
    Graph_Data rear_back_data;  // 车尾后方线条的数据包
    Graph_Data head_circle_data; // 云台圆环的数据包
    Graph_Data head_line_data;  // 云台炮管的数据包
} car_handle;


int car_init_by_handle(car_handle *);
int car_rotate_head(car_handle *, uint16_t);
int car_rotate_body(car_handle *, uint16_t);
int car_left_armor_showing_attacked(car_handle *, uint8_t);
int car_right_armor_showing_attacked(car_handle *, uint8_t);
int car_front_armor_showing_attacked(car_handle *, uint8_t);
int car_back_armor_showing_attacked(car_handle *, uint8_t);
void car_reset_front_armor_timer(car_handle *);  // 重置倒计时
void car_reset_right_armor_timer(car_handle *);
void car_reset_back_armor_timer(car_handle *);
void car_reset_left_armor_timer(car_handle *);
