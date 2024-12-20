// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen7_screen_init(void)
{
ui_Screen7 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen7, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen7, lv_color_hex(0x8F9ABF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen7, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Screen7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_Screen7, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_outline_color(ui_Screen7, lv_color_hex(0x000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_Screen7, 255, LV_PART_SCROLLBAR| LV_STATE_DEFAULT);

ui_Panel7 = lv_obj_create(ui_Screen7);
lv_obj_set_width( ui_Panel7, 240);
lv_obj_set_height( ui_Panel7, 89);
lv_obj_set_x( ui_Panel7, 0 );
lv_obj_set_y( ui_Panel7, 216 );
lv_obj_set_align( ui_Panel7, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Panel7, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0xACB2CD), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel7, 100, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_Panel7, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_Panel7, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_border_color(ui_Panel7, lv_color_hex(0x000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_Panel7, 0, LV_PART_SCROLLBAR| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_Panel7, lv_color_hex(0x000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_Panel7, 0, LV_PART_SCROLLBAR| LV_STATE_DEFAULT);

ui_Button13 = lv_btn_create(ui_Screen7);
lv_obj_set_width( ui_Button13, 64);
lv_obj_set_height( ui_Button13, 74);
lv_obj_set_x( ui_Button13, -77 );
lv_obj_set_y( ui_Button13, 217 );
lv_obj_set_align( ui_Button13, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button13, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button13, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Button14 = lv_btn_create(ui_Screen7);
lv_obj_set_width( ui_Button14, 64);
lv_obj_set_height( ui_Button14, 74);
lv_obj_set_x( ui_Button14, 70 );
lv_obj_set_y( ui_Button14, 217 );
lv_obj_set_align( ui_Button14, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button14, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button14, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Image7 = lv_img_create(ui_Screen7);
lv_img_set_src(ui_Image7, &ui_img_1567796331);
lv_obj_set_width( ui_Image7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Image7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Image7, -4 );
lv_obj_set_y( ui_Image7, 219 );
lv_obj_set_align( ui_Image7, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image7, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_Image7, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_img_set_zoom(ui_Image7,500);

ui_Label16 = lv_label_create(ui_Screen7);
lv_obj_set_width( ui_Label16, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label16, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label16, 108 );
lv_obj_set_y( ui_Label16, -256 );
lv_obj_set_align( ui_Label16, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label16,"7");
lv_obj_set_style_text_color(ui_Label16, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label16, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PressureDropdownContainer = lv_obj_create(ui_Screen7);
lv_obj_remove_style_all(ui_PressureDropdownContainer);
lv_obj_set_width( ui_PressureDropdownContainer, 210);
lv_obj_set_height( ui_PressureDropdownContainer, 138);
lv_obj_set_x( ui_PressureDropdownContainer, -2 );
lv_obj_set_y( ui_PressureDropdownContainer, 78 );
lv_obj_set_align( ui_PressureDropdownContainer, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_PressureDropdownContainer, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_PressureDropdownContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_PressureDropdownContainer, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_PressureDropdownContainer, lv_color_hex(0xACE6C5), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureDropdownContainer, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_PressureDropdownContainer, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureDropdownContainer, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PressureDropdown_2 = lv_dropdown_create(ui_PressureDropdownContainer);
lv_dropdown_set_options( ui_PressureDropdown_2, "Sens 1\n2\n3\n4" );
lv_obj_set_width( ui_PressureDropdown_2, 194);
lv_obj_set_height( ui_PressureDropdown_2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PressureDropdown_2, 1 );
lv_obj_set_y( ui_PressureDropdown_2, 42 );
lv_obj_set_align( ui_PressureDropdown_2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_PressureDropdown_2, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_set_style_text_color(ui_PressureDropdown_2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PressureDropdown_2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_PressureDropdown_2, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_PressureDropdown_2, lv_color_hex(0x5BC398), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureDropdown_2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_PressureDropdown_2, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_font(lv_dropdown_get_list(ui_PressureDropdown_2), &lv_font_montserrat_30,  LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(lv_dropdown_get_list(ui_PressureDropdown_2), lv_color_hex(0xFFFFFF),  LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(lv_dropdown_get_list(ui_PressureDropdown_2), 255,  LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_PressureDropdown_2), &lv_font_montserrat_30,  LV_PART_SELECTED| LV_STATE_DEFAULT);

ui_PressureDropdown_1 = lv_dropdown_create(ui_PressureDropdownContainer);
lv_dropdown_set_options( ui_PressureDropdown_1, "Intensity 1\nIntensity 2\nIntensity 3\nIntensity 4" );
lv_obj_set_width( ui_PressureDropdown_1, 200);
lv_obj_set_height( ui_PressureDropdown_1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PressureDropdown_1, 1 );
lv_obj_set_y( ui_PressureDropdown_1, -40 );
lv_obj_set_align( ui_PressureDropdown_1, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_PressureDropdown_1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_set_style_text_color(ui_PressureDropdown_1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PressureDropdown_1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_PressureDropdown_1, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_PressureDropdown_1, lv_color_hex(0x5BC398), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureDropdown_1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_PressureDropdown_1, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(lv_dropdown_get_list(ui_PressureDropdown_1), lv_color_hex(0xFBF9F9),  LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(lv_dropdown_get_list(ui_PressureDropdown_1), 255,  LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_PressureDropdown_1), &lv_font_montserrat_30,  LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PressureCheck = lv_checkbox_create(ui_Screen7);
lv_checkbox_set_text(ui_PressureCheck,"Pressure");
lv_obj_set_width( ui_PressureCheck, 218);
lv_obj_set_height( ui_PressureCheck, 106);
lv_obj_set_x( ui_PressureCheck, -2 );
lv_obj_set_y( ui_PressureCheck, -197 );
lv_obj_set_align( ui_PressureCheck, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_PressureCheck,LV_FLEX_FLOW_ROW_WRAP);
lv_obj_set_flex_align(ui_PressureCheck, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
lv_obj_add_flag( ui_PressureCheck, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_set_style_text_color(ui_PressureCheck, lv_color_hex(0x040000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PressureCheck, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_PressureCheck, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_PressureCheck, 20, LV_PART_INDICATOR| LV_STATE_DEFAULT);
ui_object_set_themeable_style_property(ui_PressureCheck, LV_PART_INDICATOR| LV_STATE_DEFAULT, LV_STYLE_BG_COLOR, _ui_theme_color_BackgroundGreen);
ui_object_set_themeable_style_property(ui_PressureCheck, LV_PART_INDICATOR| LV_STATE_DEFAULT, LV_STYLE_BG_OPA, _ui_theme_alpha_BackgroundGreen);
lv_obj_set_style_bg_grad_color(ui_PressureCheck, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureCheck, LV_GRAD_DIR_VER, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_PressureCheck, 40, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_PressureCheck, 60, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_PressureCheck, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_PressureCheck, 60, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_Container6 = lv_obj_create(ui_Screen7);
lv_obj_remove_style_all(ui_Container6);
lv_obj_set_width( ui_Container6, 225);
lv_obj_set_height( ui_Container6, 183);
lv_obj_set_x( ui_Container6, 0 );
lv_obj_set_y( ui_Container6, -48 );
lv_obj_set_align( ui_Container6, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Container6, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_PressureIntensitySlider = lv_slider_create(ui_Container6);
lv_slider_set_range(ui_PressureIntensitySlider, 1,4);
lv_slider_set_value( ui_PressureIntensitySlider, 3, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_PressureIntensitySlider)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_PressureIntensitySlider, 1, LV_ANIM_OFF);
lv_obj_set_width( ui_PressureIntensitySlider, 160);
lv_obj_set_height( ui_PressureIntensitySlider, 30);
lv_obj_set_x( ui_PressureIntensitySlider, -4 );
lv_obj_set_y( ui_PressureIntensitySlider, -36 );
lv_obj_set_align( ui_PressureIntensitySlider, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_PressureIntensitySlider, lv_color_hex(0xACE6C5), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureIntensitySlider, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_PressureIntensitySlider, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureIntensitySlider, LV_GRAD_DIR_HOR, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_PressureIntensitySlider, lv_color_hex(0xACE6C5), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureIntensitySlider, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_PressureIntensitySlider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureIntensitySlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_grad_color(ui_PressureIntensitySlider, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureIntensitySlider, LV_GRAD_DIR_HOR, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_PressureSensitivitySlider = lv_slider_create(ui_Container6);
lv_slider_set_range(ui_PressureSensitivitySlider, 1,4);
lv_slider_set_value( ui_PressureSensitivitySlider, 2, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_PressureSensitivitySlider)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_PressureSensitivitySlider, 1, LV_ANIM_OFF);
lv_obj_set_width( ui_PressureSensitivitySlider, 160);
lv_obj_set_height( ui_PressureSensitivitySlider, 30);
lv_obj_set_x( ui_PressureSensitivitySlider, -7 );
lv_obj_set_y( ui_PressureSensitivitySlider, 35 );
lv_obj_set_align( ui_PressureSensitivitySlider, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_PressureSensitivitySlider, lv_color_hex(0xACE6C5), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureSensitivitySlider, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_PressureSensitivitySlider, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureSensitivitySlider, LV_GRAD_DIR_HOR, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_PressureSensitivitySlider, lv_color_hex(0xACE6C5), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PressureSensitivitySlider, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_PressureSensitivitySlider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureSensitivitySlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_grad_color(ui_PressureSensitivitySlider, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_PressureSensitivitySlider, LV_GRAD_DIR_HOR, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_PressureIntensitySliderLabel = lv_label_create(ui_Container6);
lv_obj_set_width( ui_PressureIntensitySliderLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PressureIntensitySliderLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PressureIntensitySliderLabel, -10 );
lv_obj_set_y( ui_PressureIntensitySliderLabel, 0 );
lv_obj_set_align( ui_PressureIntensitySliderLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_PressureIntensitySliderLabel,"Feedback Intensity");
lv_obj_set_style_text_color(ui_PressureIntensitySliderLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PressureIntensitySliderLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PressureSensitivitySliderLabel = lv_label_create(ui_Container6);
lv_obj_set_width( ui_PressureSensitivitySliderLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PressureSensitivitySliderLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PressureSensitivitySliderLabel, -15 );
lv_obj_set_y( ui_PressureSensitivitySliderLabel, 68 );
lv_obj_set_align( ui_PressureSensitivitySliderLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_PressureSensitivitySliderLabel,"Pressure Sensitivity");
lv_obj_set_style_text_color(ui_PressureSensitivitySliderLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PressureSensitivitySliderLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Panel7, ui_event_Panel7, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button13, ui_event_Button13, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Button14, ui_event_Button14, LV_EVENT_ALL, NULL);

}
