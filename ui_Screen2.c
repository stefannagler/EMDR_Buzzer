// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen2_screen_init(void)
{
ui_Screen2 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x8F9ABF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Screen2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_Screen2, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel2 = lv_obj_create(ui_Screen2);
lv_obj_set_width( ui_Panel2, 233);
lv_obj_set_height( ui_Panel2, 89);
lv_obj_set_x( ui_Panel2, -2 );
lv_obj_set_y( ui_Panel2, 217 );
lv_obj_set_align( ui_Panel2, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Panel2, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0xACB2CD), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel2, 100, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_Panel2, LV_GRAD_DIR_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_Panel2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SpeedSlider = lv_slider_create(ui_Screen2);
lv_slider_set_range(ui_SpeedSlider, 0,50);
lv_slider_set_value( ui_SpeedSlider, 30, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_SpeedSlider)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_SpeedSlider, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_SpeedSlider, 168);
lv_obj_set_height( ui_SpeedSlider, 50);
lv_obj_set_x( ui_SpeedSlider, -4 );
lv_obj_set_y( ui_SpeedSlider, -9 );
lv_obj_set_align( ui_SpeedSlider, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_SpeedSlider, lv_color_hex(0xF2B018), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SpeedSlider, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_SpeedSlider, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_SpeedSlider, LV_GRAD_DIR_HOR, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_SpeedSlider, lv_color_hex(0x050505), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_SpeedSlider, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_SpeedSlider, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_SpeedSlider, 2, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_SpeedSlider, lv_color_hex(0xF2B018), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SpeedSlider, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_SpeedSlider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_SpeedSlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_SpeedSlider, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SpeedSlider, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_SpeedSlider, lv_color_hex(0xF2B018), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_SpeedSlider, LV_GRAD_DIR_HOR, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_SpeedSlider, lv_color_hex(0xF5F5F5), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_SpeedSlider, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_SpeedSlider, 10, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_SpeedSlider, 10, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_SpeedSlider, 10, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_SpeedSlider, 10, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Button2 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button2, 58);
lv_obj_set_height( ui_Button2, 69);
lv_obj_set_x( ui_Button2, -85 );
lv_obj_set_y( ui_Button2, 226 );
lv_obj_set_align( ui_Button2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button2, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Button2, lv_color_hex(0xD7CDCD), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Button2, lv_color_hex(0xA47AA3), LV_PART_MAIN | LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_Button2, 255, LV_PART_MAIN| LV_STATE_PRESSED);
lv_obj_set_style_bg_grad_color(ui_Button2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_PRESSED );
lv_obj_set_style_bg_grad_dir(ui_Button2, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_PRESSED);
lv_obj_set_style_shadow_color(ui_Button2, lv_color_hex(0xA47AA3), LV_PART_MAIN | LV_STATE_PRESSED );
lv_obj_set_style_shadow_opa(ui_Button2, 255, LV_PART_MAIN| LV_STATE_PRESSED);
lv_obj_set_style_shadow_width(ui_Button2, 50, LV_PART_MAIN| LV_STATE_PRESSED);
lv_obj_set_style_shadow_spread(ui_Button2, 0, LV_PART_MAIN| LV_STATE_PRESSED);

ui_Button3 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button3, 56);
lv_obj_set_height( ui_Button3, 71);
lv_obj_set_x( ui_Button3, 84 );
lv_obj_set_y( ui_Button3, 224 );
lv_obj_set_align( ui_Button3, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button3, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label2 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label2, 112 );
lv_obj_set_y( ui_Label2, -256 );
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"2");
lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TapperOnOffBtn = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_TapperOnOffBtn, 195);
lv_obj_set_height( ui_TapperOnOffBtn, 157);
lv_obj_set_x( ui_TapperOnOffBtn, -1 );
lv_obj_set_y( ui_TapperOnOffBtn, -165 );
lv_obj_set_align( ui_TapperOnOffBtn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_TapperOnOffBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_TapperOnOffBtn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_TapperOnOffBtn, 50, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0xDAB215), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_TapperOnOffBtn, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_TapperOnOffBtn, lv_color_hex(0x32FBF1), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_TapperOnOffBtn, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_TapperOnOffBtn, LV_TEXT_ALIGN_AUTO, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TapperOnOffBtn, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TapperOnOffLabel = lv_label_create(ui_TapperOnOffBtn);
lv_obj_set_width( ui_TapperOnOffLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TapperOnOffLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_TapperOnOffLabel, -2 );
lv_obj_set_y( ui_TapperOnOffLabel, 0 );
lv_obj_set_align( ui_TapperOnOffLabel, LV_ALIGN_CENTER );
lv_label_set_long_mode(ui_TapperOnOffLabel,LV_LABEL_LONG_SCROLL_CIRCULAR);
lv_label_set_text(ui_TapperOnOffLabel,"BLS On/Off");
lv_obj_set_style_text_color(ui_TapperOnOffLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TapperOnOffLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Container1 = lv_obj_create(ui_Screen2);
lv_obj_remove_style_all(ui_Container1);
lv_obj_set_width( ui_Container1, 240);
lv_obj_set_height( ui_Container1, 21);
lv_obj_set_x( ui_Container1, 0 );
lv_obj_set_y( ui_Container1, -256 );
lv_obj_set_align( ui_Container1, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_BatLabel = lv_label_create(ui_Container1);
lv_obj_set_width( ui_BatLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_BatLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_BatLabel, -9 );
lv_obj_set_y( ui_BatLabel, -3 );
lv_obj_set_align( ui_BatLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_BatLabel,"BatLabel");

ui_BatLeftLabel = lv_label_create(ui_Container1);
lv_obj_set_width( ui_BatLeftLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_BatLeftLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_BatLeftLabel, -96 );
lv_obj_set_y( ui_BatLeftLabel, 1 );
lv_obj_set_align( ui_BatLeftLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_BatLeftLabel,"BatLeft");

ui_BatRightLabel = lv_label_create(ui_Container1);
lv_obj_set_width( ui_BatRightLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_BatRightLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_BatRightLabel, 93 );
lv_obj_set_y( ui_BatRightLabel, 3 );
lv_obj_set_align( ui_BatRightLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_BatRightLabel,"BatRight");

ui_Image2 = lv_img_create(ui_Screen2);
lv_img_set_src(ui_Image2, &ui_img_1567796331);
lv_obj_set_width( ui_Image2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Image2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Image2, -4 );
lv_obj_set_y( ui_Image2, 219 );
lv_obj_set_align( ui_Image2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_Image2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_img_set_zoom(ui_Image2,500);

ui_SpeedLabel = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_SpeedLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SpeedLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SpeedLabel, -30 );
lv_obj_set_y( ui_SpeedLabel, 41 );
lv_obj_set_align( ui_SpeedLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SpeedLabel,"Speed");
lv_obj_set_style_text_color(ui_SpeedLabel, lv_color_hex(0x0A0101), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SpeedLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SpeedLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SliderValueLabel = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_SliderValueLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SliderValueLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SliderValueLabel, 41 );
lv_obj_set_y( ui_SliderValueLabel, 43 );
lv_obj_set_align( ui_SliderValueLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SliderValueLabel,"30");
lv_obj_set_style_text_color(ui_SliderValueLabel, lv_color_hex(0x020000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SliderValueLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SliderValueLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ConnectBuzzersLabel1 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_ConnectBuzzersLabel1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_ConnectBuzzersLabel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_ConnectBuzzersLabel1, 1 );
lv_obj_set_y( ui_ConnectBuzzersLabel1, -118 );
lv_obj_set_align( ui_ConnectBuzzersLabel1, LV_ALIGN_CENTER );
lv_label_set_text(ui_ConnectBuzzersLabel1,"ConnectBuzzersLabel");
lv_obj_set_style_text_color(ui_ConnectBuzzersLabel1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_ConnectBuzzersLabel1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_isCheckBoxLabel = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_isCheckBoxLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_isCheckBoxLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_isCheckBoxLabel, 2 );
lv_obj_set_y( ui_isCheckBoxLabel, -205 );
lv_obj_set_align( ui_isCheckBoxLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_isCheckBoxLabel,"Select Tapper Mode");
lv_obj_set_style_text_color(ui_isCheckBoxLabel, lv_color_hex(0x040404), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_isCheckBoxLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Panel2, ui_event_Panel2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_SpeedSlider, ui_event_SpeedSlider, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button2, ui_event_Button2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button3, ui_event_Button3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_TapperOnOffBtn, ui_event_TapperOnOffBtn, LV_EVENT_ALL, NULL);

}
