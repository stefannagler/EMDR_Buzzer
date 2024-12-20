// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen5_screen_init(void)
{
ui_Screen5 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Screen5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Screen5, lv_color_hex(0x8F9ABF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen5, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Screen5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_Screen5, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel5 = lv_obj_create(ui_Screen5);
lv_obj_set_width( ui_Panel5, 240);
lv_obj_set_height( ui_Panel5, 89);
lv_obj_set_x( ui_Panel5, -2 );
lv_obj_set_y( ui_Panel5, 217 );
lv_obj_set_align( ui_Panel5, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Panel5, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0xACB2CD), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel5, 100, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_Panel5, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button7 = lv_btn_create(ui_Screen5);
lv_obj_set_width( ui_Button7, 64);
lv_obj_set_height( ui_Button7, 74);
lv_obj_set_x( ui_Button7, -77 );
lv_obj_set_y( ui_Button7, 217 );
lv_obj_set_align( ui_Button7, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button7, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button7, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Button10 = lv_btn_create(ui_Screen5);
lv_obj_set_width( ui_Button10, 64);
lv_obj_set_height( ui_Button10, 74);
lv_obj_set_x( ui_Button10, 70 );
lv_obj_set_y( ui_Button10, 217 );
lv_obj_set_align( ui_Button10, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button10, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label6 = lv_label_create(ui_Screen5);
lv_obj_set_width( ui_Label6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label6, 108 );
lv_obj_set_y( ui_Label6, -256 );
lv_obj_set_align( ui_Label6, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label6,"5");
lv_obj_set_style_text_color(ui_Label6, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label6, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label9 = lv_label_create(ui_Screen5);
lv_obj_set_width( ui_Label9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label9, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label9, 594 );
lv_obj_set_y( ui_Label9, -173 );
lv_obj_set_align( ui_Label9, LV_ALIGN_CENTER );

ui_Image5 = lv_img_create(ui_Screen5);
lv_img_set_src(ui_Image5, &ui_img_1567796331);
lv_obj_set_width( ui_Image5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Image5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Image5, -4 );
lv_obj_set_y( ui_Image5, 219 );
lv_obj_set_align( ui_Image5, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image5, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_Image5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_img_set_zoom(ui_Image5,500);

ui_SoundDropdownContainer = lv_obj_create(ui_Screen5);
lv_obj_remove_style_all(ui_SoundDropdownContainer);
lv_obj_set_width( ui_SoundDropdownContainer, 210);
lv_obj_set_height( ui_SoundDropdownContainer, 230);
lv_obj_set_x( ui_SoundDropdownContainer, 0 );
lv_obj_set_y( ui_SoundDropdownContainer, -73 );
lv_obj_set_align( ui_SoundDropdownContainer, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_SoundDropdownContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_SoundDropdownContainer, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SoundDropdownContainer, lv_color_hex(0x3959F6), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SoundDropdownContainer, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_SoundDropdownContainer, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_SoundDropdownContainer, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SoundVolume = lv_dropdown_create(ui_SoundDropdownContainer);
lv_dropdown_set_options( ui_SoundVolume, "Vol 1\nVol 2\nVol 3\nVol 4" );
lv_obj_set_width( ui_SoundVolume, 180);
lv_obj_set_height( ui_SoundVolume, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SoundVolume, -1 );
lv_obj_set_y( ui_SoundVolume, -74 );
lv_obj_set_align( ui_SoundVolume, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_SoundVolume, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_set_style_text_color(ui_SoundVolume, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SoundVolume, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SoundVolume, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SoundVolume, lv_color_hex(0x5BC398), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SoundVolume, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_SoundVolume, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_SoundVolume, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SoundVolume, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SoundVolume, &lv_font_montserrat_20, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(lv_dropdown_get_list(ui_SoundVolume), lv_color_hex(0xFFFFFF),  LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(lv_dropdown_get_list(ui_SoundVolume), 255,  LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_SoundVolume), &lv_font_montserrat_30,  LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SoundLenght = lv_dropdown_create(ui_SoundDropdownContainer);
lv_dropdown_set_options( ui_SoundLenght, "Lenght 1\nLength 2\nLength 3\nLength 4" );
lv_obj_set_width( ui_SoundLenght, 180);
lv_obj_set_height( ui_SoundLenght, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SoundLenght, 1 );
lv_obj_set_y( ui_SoundLenght, 53 );
lv_obj_set_align( ui_SoundLenght, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_SoundLenght, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_set_style_text_color(ui_SoundLenght, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SoundLenght, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SoundLenght, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SoundLenght, lv_color_hex(0x5BC398), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SoundLenght, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_SoundLenght, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(lv_dropdown_get_list(ui_SoundLenght), lv_color_hex(0xFFFFFF),  LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(lv_dropdown_get_list(ui_SoundLenght), 255,  LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_SoundLenght), &lv_font_montserrat_30,  LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label12 = lv_label_create(ui_Screen5);
lv_obj_set_width( ui_Label12, LV_SIZE_CONTENT);  /// 192
lv_obj_set_height( ui_Label12, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_Label12, -9 );
lv_obj_set_y( ui_Label12, -222 );
lv_obj_set_align( ui_Label12, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label12,"Sound");
lv_obj_set_style_text_color(ui_Label12, lv_color_hex(0x080101), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label12, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_Label12, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label12, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Panel5, ui_event_Panel5, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button7, ui_event_Button7, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button10, ui_event_Button10, LV_EVENT_ALL, NULL);

}
