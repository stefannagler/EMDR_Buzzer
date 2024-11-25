// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"
#include "ui_theme_manager.h"
#include "ui_themes.h"

void ToggleOnOff_Animation( lv_obj_t *TargetObject, int delay);
void twist_Animation( lv_obj_t *TargetObject, int delay);
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1( lv_event_t * e);
extern lv_obj_t *ui_Screen1;
extern lv_obj_t *ui_Image1;
extern lv_obj_t *ui_Label1;
extern lv_obj_t *ui_Button1;
extern lv_obj_t *ui_Container2;
extern lv_obj_t *ui_BatBarLeft;
extern lv_obj_t *ui_BatBarCenter;
extern lv_obj_t *ui_BatBarRight;
extern lv_obj_t *ui_ConnectBuzzersLabel;
// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
extern lv_obj_t *ui_Screen2;
void ui_event_Panel2( lv_event_t * e);
extern lv_obj_t *ui_Panel2;
void ui_event_SpeedSlider( lv_event_t * e);
extern lv_obj_t *ui_SpeedSlider;
void ui_event_Button2( lv_event_t * e);
extern lv_obj_t *ui_Button2;
void ui_event_Button3( lv_event_t * e);
extern lv_obj_t *ui_Button3;
extern lv_obj_t *ui_Label2;
void ui_event_TapperOnOffBtn( lv_event_t * e);
extern lv_obj_t *ui_TapperOnOffBtn;
extern lv_obj_t *ui_TapperOnOffLabel;
extern lv_obj_t *ui_Container1;
extern lv_obj_t *ui_BatLabel;
extern lv_obj_t *ui_BatLeftLabel;
extern lv_obj_t *ui_BatRightLabel;
extern lv_obj_t *ui_Image2;
extern lv_obj_t *ui_SpeedLabel;
extern lv_obj_t *ui_SliderValueLabel;
extern lv_obj_t *ui_ConnectBuzzersLabel1;
extern lv_obj_t *ui_isCheckBoxLabel;
// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
extern lv_obj_t *ui_Screen3;
void ui_event_Panel3( lv_event_t * e);
extern lv_obj_t *ui_Panel3;
void ui_event_Button5( lv_event_t * e);
extern lv_obj_t *ui_Button5;
void ui_event_Button4( lv_event_t * e);
extern lv_obj_t *ui_Button4;
extern lv_obj_t *ui_Label3;
extern lv_obj_t *ui_BuzzCheck;
extern lv_obj_t *ui_BuzzOnOffLabel;
extern lv_obj_t *ui_LightCheck;
extern lv_obj_t *ui_SoundCheck;
extern lv_obj_t *ui_Image3;
// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
extern lv_obj_t *ui_Screen4;
void ui_event_Panel4( lv_event_t * e);
extern lv_obj_t *ui_Panel4;
void ui_event_Button6( lv_event_t * e);
extern lv_obj_t *ui_Button6;
void ui_event_Button9( lv_event_t * e);
extern lv_obj_t *ui_Button9;
extern lv_obj_t *ui_Label4;
extern lv_obj_t *ui_Image4;
extern lv_obj_t *ui_Container4;
extern lv_obj_t *ui_BuzzDropdownContainer;
extern lv_obj_t *ui_BuzzPower;
extern lv_obj_t *ui_BuzzLenght;
extern lv_obj_t *ui_LiightDropdownContainer;
extern lv_obj_t *ui_LightIntensity;
extern lv_obj_t *ui_LightLenght;
extern lv_obj_t *ui_Label11;
extern lv_obj_t *ui_Label15;
// SCREEN: ui_Screen5
void ui_Screen5_screen_init(void);
extern lv_obj_t *ui_Screen5;
void ui_event_Panel5( lv_event_t * e);
extern lv_obj_t *ui_Panel5;
void ui_event_Button7( lv_event_t * e);
extern lv_obj_t *ui_Button7;
void ui_event_Button10( lv_event_t * e);
extern lv_obj_t *ui_Button10;
extern lv_obj_t *ui_Label6;
extern lv_obj_t *ui_Label9;
extern lv_obj_t *ui_Image5;
extern lv_obj_t *ui_SoundDropdownContainer;
extern lv_obj_t *ui_SoundVolume;
extern lv_obj_t *ui_SoundLenght;
extern lv_obj_t *ui_Label12;
// SCREEN: ui_Screen6
void ui_Screen6_screen_init(void);
extern lv_obj_t *ui_Screen6;
void ui_event_Panel6( lv_event_t * e);
extern lv_obj_t *ui_Panel6;
void ui_event_Button11( lv_event_t * e);
extern lv_obj_t *ui_Button11;
void ui_event_Button12( lv_event_t * e);
extern lv_obj_t *ui_Button12;
extern lv_obj_t *ui_Label7;
extern lv_obj_t *ui_Image6;
extern lv_obj_t *ui_SliderBrightness;
extern lv_obj_t *ui_Label10;
// SCREEN: ui_Screen7
void ui_Screen7_screen_init(void);
extern lv_obj_t *ui_Screen7;
void ui_event_Panel7( lv_event_t * e);
extern lv_obj_t *ui_Panel7;
void ui_event_Button13( lv_event_t * e);
extern lv_obj_t *ui_Button13;
void ui_event_Button14( lv_event_t * e);
extern lv_obj_t *ui_Button14;
extern lv_obj_t *ui_Image7;
extern lv_obj_t *ui_Label16;
extern lv_obj_t *ui_PressureDropdownContainer;
extern lv_obj_t *ui_PressureDropdown_2;
extern lv_obj_t *ui_PressureDropdown_1;
extern lv_obj_t *ui_PressureCheck;
extern lv_obj_t *ui_Container6;
extern lv_obj_t *ui_PressureIntensitySlider;
extern lv_obj_t *ui_PressureSensitivitySlider;
extern lv_obj_t *ui_PressureIntensitySliderLabel;
extern lv_obj_t *ui_PressureSensitivitySliderLabel;
// SCREEN: ui_Screen8
void ui_Screen8_screen_init(void);
extern lv_obj_t *ui_Screen8;
void ui_event_Button8( lv_event_t * e);
extern lv_obj_t *ui_Button8;
void ui_event_Button15( lv_event_t * e);
extern lv_obj_t *ui_Button15;
extern lv_obj_t *ui_Label5;
void ui_event_Panel1( lv_event_t * e);
extern lv_obj_t *ui_Panel1;
extern lv_obj_t *ui_Image8;
extern lv_obj_t *ui_Container5;
extern lv_obj_t *ui_BuzzIntensitySlider;
extern lv_obj_t *ui_BuzzIntensitySliderLabel;
extern lv_obj_t *ui_BuzzLenghtSlider;
extern lv_obj_t *ui_BuzzLenghtSliderLabel;
extern lv_obj_t *ui_LightIntensitySlider;
extern lv_obj_t *ui_LightIntensitySliderLabel;
extern lv_obj_t *ui_LightColourSlider;
extern lv_obj_t *ui_LightColourSliderLabel;
extern lv_obj_t *ui_SoundIntensitySlider;
extern lv_obj_t *ui_SoundLenghtSliderLabel;
extern lv_obj_t *ui_SoundLenghtSlider;
extern lv_obj_t *ui_SoundIntensitySliderLabel;
// SCREEN: ui_Screen9
void ui_Screen9_screen_init(void);
extern lv_obj_t *ui_Screen9;
extern lv_obj_t *ui_Colorwheel2;
// SCREEN: ui_Screen10
void ui_Screen10_screen_init(void);
extern lv_obj_t *ui_Screen10;
extern lv_obj_t *ui_Colorwheel1;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_slide1_png);   // assets/Slide1.png
LV_IMG_DECLARE( ui_img_1567796331);   // assets/pngtree-slide-left-and-right-png-image_5281204_resized.png
LV_IMG_DECLARE( ui_img_1751633538);   // assets/pngtree-slide-left-and-right-png-image_5281204.png




void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
