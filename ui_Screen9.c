// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen9_screen_init(void)
{
ui_Screen9 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen9, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen9, lv_color_hex(0xFFDEF6), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen9, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Colorwheel2 = lv_colorwheel_create(ui_Screen9,true);
lv_obj_set_width( ui_Colorwheel2, 92);
lv_obj_set_height( ui_Colorwheel2, 94);
lv_obj_set_x( ui_Colorwheel2, -12 );
lv_obj_set_y( ui_Colorwheel2, -87 );
lv_obj_set_align( ui_Colorwheel2, LV_ALIGN_CENTER );


}