// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen10_screen_init(void)
{
ui_Screen10 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Colorwheel1 = lv_colorwheel_create(ui_Screen10,true);
lv_obj_set_width( ui_Colorwheel1, 150);
lv_obj_set_height( ui_Colorwheel1, 150);
lv_obj_set_x( ui_Colorwheel1, -308 );
lv_obj_set_y( ui_Colorwheel1, 72 );
lv_obj_set_align( ui_Colorwheel1, LV_ALIGN_CENTER );


}