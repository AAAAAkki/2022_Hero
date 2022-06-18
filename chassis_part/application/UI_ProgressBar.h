#ifndef UI_PROGRESSBAR_H
#define UI_PROGRESSBAR_H


#endif
/** 说明
 * 一个进度条所需的所有数据
 *  *   
 * border进度条边框   
 * ProgressFilling进展填充
 * percent目前剩余百分比(数字)
 * 
 *  * graphic_name_array图形编号
 * 
 * start_x,start_y开始坐标
 * end_x，end_y结束坐标
 * layer所在图层
 * color_code颜色代码
 * ***0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
 * line_width宽度（数字=像素点）
 * **/

#include <stdint.h>

typedef struct {
/*目前百分比 进度条当前大小：0到100间的整数*/
    int percent_x;
    int percent_y;
    int percent_font_size;
    int percent_color_code;
    int percent_line_width;
/*** 进度条表示的量的数据导入 ***/
    uint16_t progress_bar_data_change;                //进度条表示的量的实时数据
//    float progress_bar_data_large;                //进度条表示的量的最大值
} progress_bar_data;


void UI_ProgressBar_static(progress_bar_data *bar);

void UI_ProgressBar_change(progress_bar_data *bar);
