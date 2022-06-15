//
// Created by Juntong on 2022/3/8.
//
/*使用说明：
 *  首先需要手动初始化`car_handle`结构体，再将其传入 car_init_by_handle 中。
 *  设计上，只需初始化结构体，之后一切不需要动了。
 *
 */
#include "UI_car.h"

#include "math.h"
#include "string.h"
#include "RM_Cilent_UI.h"

#define basic_cfg self->basic_config
#define PI 3.141592653589793
#define hrad PI * self->head_degree / 180.0
#define brad PI * self->body_degree / 180.0
#define get_colour(attr) self->attr ? basic_cfg.attacked_colour_code : basic_cfg.normal_colour_code

int car_init_head(car_handle *);

int car_draw_head_line(car_handle *, uint8_t);

int car_draw_body(car_handle *, uint8_t);

int car_init_by_handle(car_handle *self) {
    /* 使用传入的句柄绘制基础图像
     * 返回0即成功
     */

    car_draw_body(self, UI_Graph_ADD);

    car_init_head(self);

    return 0;
}

int car_init_head(car_handle *self) {
    Circle_Draw(&self->head_circle_data, basic_cfg.head_name_circle, UI_Graph_ADD, basic_cfg.head_layer,
                basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x, basic_cfg.central_y,
                basic_cfg.head_radius);
    car_draw_head_line(self, UI_Graph_ADD);
    UI_ReFresh(2, self->head_circle_data, self->head_line_data);
//    UI_ReFresh(1, self->head_circle_data);
    return 0;
}

int car_draw_head_line(car_handle *self, uint8_t operate) {

    int x = basic_cfg.full_radius * sin(hrad) + basic_cfg.central_x;
    int y = basic_cfg.central_y - basic_cfg.full_radius * cos(hrad);
    Line_Draw(&self->head_line_data, basic_cfg.head_name_line, operate, basic_cfg.head_layer,
              basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x, basic_cfg.central_y, x, y);
    return 0;
}

int car_draw_body(car_handle *self, uint8_t operate) {

    double s = sin(self->body_degree);
    double c = cos(self->body_degree);

    int Ax = (int) (s * basic_cfg.body_half_length - c * basic_cfg.body_half_width);
    int Ay = (int) (c * basic_cfg.body_half_length + s * basic_cfg.body_half_width);
    int Bx = (int) (s * basic_cfg.body_half_length + c * basic_cfg.body_half_width);
    int By = (int) (c * basic_cfg.body_half_length - s * basic_cfg.body_half_width);
    int Ex = (int) (c * basic_cfg.rear_half_width - s * basic_cfg.full_radius);
    int Ey = (int) (-c * basic_cfg.full_radius - s * basic_cfg.rear_half_width);
    int Fx = (int) (-s * basic_cfg.full_radius - c * basic_cfg.rear_half_width);
    int Fy = (int) (s * basic_cfg.rear_half_width - c * basic_cfg.full_radius);

    Line_Draw(&self->body_front_data, basic_cfg.body_name_front, operate, basic_cfg.body_layer,
              get_colour(front_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x + Ax,
              basic_cfg.central_y - Ay, basic_cfg.central_x + Bx, basic_cfg.central_y - By);
    // BC; C(-Ax, -Ay)
    Line_Draw(&self->body_left_data, basic_cfg.body_name_left, operate, basic_cfg.body_layer,
              get_colour(left_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x - Bx,
              basic_cfg.central_y + By, basic_cfg.central_x + Ax, basic_cfg.central_y - Ay);
    // CD; D(-Bx, -By)
    Line_Draw(&self->body_back_data, basic_cfg.body_name_back, operate, basic_cfg.body_layer,
              basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x - Ax, basic_cfg.central_y + Ay,
              basic_cfg.central_x - Bx, basic_cfg.central_y + By);
    // DA
    Line_Draw(&self->body_right_data, basic_cfg.body_name_right, operate, basic_cfg.body_layer,
              basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x + Bx, basic_cfg.central_y - By,
              basic_cfg.central_x - Ax, basic_cfg.central_y + Ay);
    // CE; E(Gx + R * sin, Gy - R * cos)
    Line_Draw(&self->rear_left_data, basic_cfg.rear_name_left, operate, basic_cfg.body_layer,
              basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x - Ax, basic_cfg.central_y + Ay,
              basic_cfg.central_x + Ex, basic_cfg.central_y - Ey);
    // EF; F(Gx - R * sin, Gy + R * cos)
    Line_Draw(&self->rear_back_data, basic_cfg.rear_name_back, operate, basic_cfg.body_layer,
              get_colour(back_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x + Ex,
              basic_cfg.central_y - Ey, basic_cfg.central_x + Fx, basic_cfg.central_y - Fy);
    // FD
    Line_Draw(&self->rear_right_data, basic_cfg.rear_name_right, operate, basic_cfg.body_layer,
              basic_cfg.normal_colour_code, basic_cfg.drawing_width, basic_cfg.central_x + Fx, basic_cfg.central_y - Fy,
              basic_cfg.central_x - Bx, basic_cfg.central_y + By);
    UI_ReFresh(7, self->body_front_data, self->body_left_data, self->body_right_data, self->body_back_data,
               self->rear_back_data, self->rear_left_data, self->rear_right_data);
    return 0;
}

int car_rotate_head(car_handle *self, uint16_t degree) {
    if (self->head_degree != degree) {
        self->head_degree = degree;
        car_draw_head_line(self, UI_Graph_Change);
        UI_ReFresh(1, self->head_line_data);
    }
    return 0;
}

int car_rotate_body(car_handle *self, fp32 degree) {
    if (self->body_degree != degree) {
        self->body_degree = degree;
        car_draw_body(self, UI_Graph_Change);
    }
    return 0;
}

int car_left_armor_showing_attacked(car_handle *self, uint8_t attacked) {
    if (self->left_armor_showing_attacked != attacked) {
        double s = sin(brad);
        double c = cos(brad);

        int Ax = (int) (s * basic_cfg.body_half_length - c * basic_cfg.body_half_width);
        int Ay = (int) (c * basic_cfg.body_half_length + s * basic_cfg.body_half_width);
        int Bx = (int) (s * basic_cfg.body_half_length + c * basic_cfg.body_half_width);
        int By = (int) (c * basic_cfg.body_half_length - s * basic_cfg.body_half_width);
        self->left_armor_showing_attacked = attacked;
        Line_Draw(&self->body_left_data, basic_cfg.body_name_left, UI_Graph_Change, basic_cfg.body_layer,
                  get_colour(left_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x - Bx,
                  basic_cfg.central_y + By, basic_cfg.central_x + Ax, basic_cfg.central_y - Ay);
        UI_ReFresh(1, self->body_left_data);
    }
    return 0;
}

int car_right_armor_showing_attacked(car_handle *self, uint8_t attacked) {
    if (self->right_armor_showing_attacked != attacked) {
        double s = sin(brad);
        double c = cos(brad);

        int Ax = (int) (s * basic_cfg.body_half_length - c * basic_cfg.body_half_width);
        int Ay = (int) (c * basic_cfg.body_half_length + s * basic_cfg.body_half_width);
        int Bx = (int) (s * basic_cfg.body_half_length + c * basic_cfg.body_half_width);
        int By = (int) (c * basic_cfg.body_half_length - s * basic_cfg.body_half_width);
        self->right_armor_showing_attacked = attacked;
        Line_Draw(&self->body_right_data, basic_cfg.body_name_right, UI_Graph_Change, basic_cfg.body_layer,
                  get_colour(right_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x + Bx,
                  basic_cfg.central_y - By, basic_cfg.central_x - Ax, basic_cfg.central_y + Ay);
        UI_ReFresh(1, self->body_right_data);
    }
    return 0;
}

int car_front_armor_showing_attacked(car_handle *self, uint8_t attacked) {
    if (self->front_armor_showing_attacked != attacked) {
        double s = sin(brad);
        double c = cos(brad);

        int Ax = (int) (s * basic_cfg.body_half_length - c * basic_cfg.body_half_width);
        int Ay = (int) (c * basic_cfg.body_half_length + s * basic_cfg.body_half_width);
        int Bx = (int) (s * basic_cfg.body_half_length + c * basic_cfg.body_half_width);
        int By = (int) (c * basic_cfg.body_half_length - s * basic_cfg.body_half_width);
        self->front_armor_showing_attacked = attacked;
        Line_Draw(&self->body_front_data, basic_cfg.body_name_front, UI_Graph_Change, basic_cfg.body_layer,
                  get_colour(front_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x + Ax,
                  basic_cfg.central_y - Ay, basic_cfg.central_x - Bx, basic_cfg.central_y + By);
        UI_ReFresh(1, self->body_front_data);
    }
    return 0;
}

int car_back_armor_showing_attacked(car_handle *self, uint8_t attacked) {
    if (self->back_armor_showing_attacked != attacked) {
        double s = sin(brad);
        double c = cos(brad);

        int Ex = (int) (c * basic_cfg.rear_half_width - s * basic_cfg.full_radius);
        int Ey = (int) (-c * basic_cfg.full_radius - s * basic_cfg.rear_half_width);
        int Fx = (int) (-s * basic_cfg.full_radius - c * basic_cfg.rear_half_width);
        int Fy = (int) (s * basic_cfg.rear_half_width - c * basic_cfg.full_radius);
        self->back_armor_showing_attacked = attacked;
        // EF; F(Gx - R * sin, Gy + R * cos)
        Line_Draw(&self->rear_back_data, basic_cfg.rear_name_back, UI_Graph_Change, basic_cfg.body_layer,
                  get_colour(back_armor_showing_attacked), basic_cfg.drawing_width, basic_cfg.central_x + Ex,
                  basic_cfg.central_y - Ey, basic_cfg.central_x + Fx, basic_cfg.central_y - Fy);
        UI_ReFresh(1, self->rear_back_data);
    }
    return 0;
}
