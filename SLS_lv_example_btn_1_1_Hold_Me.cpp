/*#include <lvgl.h>
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#ifndef MOD_LV_H
#define MOD_LV_H

//#include "SLS_BLE_Setup.h"

#endif //MOD_LV.h



//#include "Modified_BLE_Setup.h"
// Tapper-related variables
extern int Tapper_OnOff_Flag;
extern int Start_Stop_Flag;

// BLE variables
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint32_t value1;
extern uint32_t testValue1;
extern unsigned long previousDevConnectedMillis;
extern const unsigned long notificationInterval;
extern unsigned long previousUpdateBuzzerMillis;
extern const unsigned long notificationUpdateBuzzerInterval;

extern int Tapper_B_Intensity;
extern int Tapper_L_Intensity;

extern LilyGo_Class amoled;
extern int on_off;  // Global variable for on/off state used in the on_off button

extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint32_t value1;
extern uint32_t testValue1;
//extern int set_speed;

#if LV_USE_BTN && LV_USE_LABEL

extern int main_counter;  // Reference to the global variable in the main file

static int counter_direction = 1;         // 1 for incrementing, -1 for decrementing
static lv_obj_t *main_counter_label;      // Label to display the global main_counter value
static int last_main_counter_value = -1;  // Track the last known value of main_counter
static lv_timer_t *hold_timer;            // Timer to handle continuous increment/decrement
static bool is_timer_active = false;      // Flag to check if the timer is active
static lv_obj_t *on_off_btn;              // Button for on/off state
static lv_obj_t *on_off_label;            // Label for on/off button
*/

#include <lvgl.h>
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>

#ifndef MOD_LV_H
#define MOD_LV_H

// Tapper-related variables
extern int Tapper_OnOff_Flag;
extern int Start_Stop_Flag;
extern int Tapper_Buzz_Flag;
extern int Tapper_B_Intensity;
extern int Tapper_B_Other;
extern int Tapper_Light_Flag;
extern int Tapper_L_Intensity;
extern int Tapper_L_Other;
extern int Tapper_Sound_Flag;
extern int Tapper_S_Intensity;
extern int Tapper_S_Other;
extern int Tapper_Pressure_Flag;
extern int Tapper_P_Intensity;
extern int Tapper_P_Other;

// BLE variables
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint32_t value1;
extern uint32_t testValue1;
extern unsigned long previousDevConnectedMillis;
extern const unsigned long notificationInterval;
extern unsigned long previousUpdateBuzzerMillis;
extern const unsigned long notificationUpdateBuzzerInterval;

// Other global variables
extern LilyGo_Class amoled;
extern int on_off;        // Global variable for on/off state used in the on_off button
extern int main_counter;  // Reference to the global variable in the main file
extern int set_speed;     // Speed variable from the UI slider
extern int brightness;
// Variables for LVGL objects (UI components)
extern lv_obj_t *ui_SliderBrightness;
extern lv_obj_t *ui_SpeedSlider;
extern lv_obj_t *ui_SliderValueLabel;
extern lv_obj_t *ui_BuzzCheck;
extern lv_obj_t *ui_BuzzPower;
extern lv_obj_t *ui_BuzzLenght;
extern lv_obj_t *ui_LightCheck;
extern lv_obj_t *ui_LightIntensity;
extern lv_obj_t *ui_LightLenght;
extern lv_obj_t *ui_SoundCheck;
extern lv_obj_t *ui_SoundVolume;
extern lv_obj_t *ui_SoundLenght;
extern lv_obj_t *ui_PressureCheck;
extern lv_obj_t *ui_PressureDropdown_1;
extern lv_obj_t *ui_PressureDropdown_2;

extern lv_obj_t *ui_BuzzOnOffLabel;

// Static variables and LVGL objects
static int counter_direction = 1;         // 1 for incrementing, -1 for decrementing
static lv_obj_t *main_counter_label;      // Label to display the global main_counter value
static int last_main_counter_value = -1;  // Track the last known value of main_counter
static lv_timer_t *hold_timer;            // Timer to handle continuous increment/decrement
static bool is_timer_active = false;      // Flag to check if the timer is active
static lv_obj_t *on_off_btn;              // Button for on/off state
static lv_obj_t *on_off_label;            // Label for on/off button

// Function prototypes
void get_speed_SpeedSlider(void);
void get_SliderBrightness(void);
void get_buzz_check_value(void);
void get_buzz_intensity_value(void);
void get_buzz_length_value(void);
void get_light_check_value(void);
void get_light_intensity_value(void);
void get_light_length_value(void);
void get_sound_check_value(void);
void get_sound_intensity_value(void);
void get_sound_length_value(void);
void get_pressure_check_value(void);
void get_pressure_intensity_value(void);
void get_pressure_length_value(void);

void set_speed_SpeedSlider(void);
void get_SliderBrightness(void);
void set_buzz_check_value(void);
void set_buzz_intensity_value(void);
void set_buzz_length_value(void);
void set_light_check_value(void);
void set_light_intensity_value(void);
void set_light_length_value(void);
void set_sound_check_value(void);
void set_sound_intensity_value(void);
void set_sound_length_value(void);
void set_pressure_check_value(void);
void set_pressure_intensity_value(void);
void set_pressure_length_value(void);
void storeVariablesToFlash(void);

void lv_example_btn_1_1_Hold_Me(void);
void update_main_counter_label(void);
void update_on_off_button(void);
static void hold_timer_callback(lv_timer_t *timer);
static void on_off_event_handler(lv_event_t *e);
static void event_handler(lv_event_t *e);

//#endif //MOD_LV.hƒ

// Update the main_counter label if the value changes
void update_main_counter_label(void) {
  if (main_counter != last_main_counter_value) {
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "Global: %d", main_counter);
    lv_label_set_text(main_counter_label, buf);
    last_main_counter_value = main_counter;  // Update the last known value
  }
}

// Update the on/off button label and color
void update_on_off_button() {
  if (on_off == 1) {
    lv_label_set_text(on_off_label, "OFF");                            //shows on main screen
    lv_obj_set_style_bg_color(on_off_btn, lv_color_hex(0xFF0000), 0);  // Red color
  } else {
    on_off = 1; //otherwise somehow Gpio 0 psress sends it back to here , it turns on and off since on_off is in RTC mem
    lv_label_set_text(on_off_label, "ON");
    lv_obj_set_style_bg_color(on_off_btn, lv_color_hex(0x00FF00), 0);  // Green color
                                                                       // Enter deep sleep after storing settings
    // Call the function to store variables to flash
    Serial.println("Storing settings before entering deep sleep...");
    storeVariablesToFlash();  // checks for "if changed" before storing settings for the buzzer
    Serial.println("Settings stored successfully.");

    // Notify that the system is about to enter deep sleep
    Serial.println("Entering deep sleep mode...");
    amoled.setBrightness(0);  // Manually turn off display
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake up when GPIO 0 is low

    esp_deep_sleep_start();
  }
}

// On/Off button event handler
static void on_off_event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    // Toggle on_off state
    on_off = !on_off;
    update_on_off_button();  // Update the button appearance
    Serial.print("On/Off state: ");
    Serial.println(on_off);
    //initBuzzers();
    //Tapper_OnOff_Flag = !Tapper_OnOff_Flag;  //gets sent elsewhere

    // pChar_Tapper_OnOff_Flag->setValue(Tapper_OnOff_Flag);
    // pChar_Tapper_OnOff_Flag->notify();
    // delay(delayInBetweenBLE);
  }
}

// Timer callback function to increment or decrement the main_counter
static void hold_timer_callback(lv_timer_t *timer) {
  main_counter += counter_direction;
  Serial.print("Setting Brightness: ");
  Serial.println(main_counter);

  //amoled.setBrightness(main_counter);
  //Tapper_L_Intensity = main_counter;
  if (main_counter >= 255) {
    main_counter = 255;
    counter_direction = -1;  // Start decrementing
  } else if (main_counter <= 100) {
    main_counter = 100;
    counter_direction = 1;  // Start incrementing
  }

  update_main_counter_label();
}

// Event handler for the button under EMDR but it is set to invisable
static void event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    LV_LOG_USER("Clicked");
  } else if (code == LV_EVENT_PRESSING) {
    if (!is_timer_active) {
      is_timer_active = true;
      lv_timer_set_period(hold_timer, 100);  // Set the period of the timer to 100 ms
      lv_timer_resume(hold_timer);           // Start incrementing/decrementing while held
    }
  } else if (code == LV_EVENT_RELEASED) {
    if (is_timer_active) {
      is_timer_active = false;     // Mark the timer as inactive
      lv_timer_pause(hold_timer);  // Pause the timer
    }
  }
}

void lv_example_btn_1_1_Hold_Me(void) {
  // Create On/Off button at the bottom
  on_off_btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(on_off_btn, 100, 100);
  lv_obj_add_event_cb(on_off_btn, on_off_event_handler, LV_EVENT_CLICKED, NULL);
  lv_obj_align(on_off_btn, LV_ALIGN_BOTTOM_MID, 0, -20);

  // Label for On/Off button
  on_off_label = lv_label_create(on_off_btn);
  lv_obj_center(on_off_label);
  update_on_off_button();  // Set initial appearance of the button
  return;                  //cancel execution as it was just a first test
  lv_obj_t *label;

  // Create btn1
  lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn1, 100, 100);
  lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

  // Label for btn1
  label = lv_label_create(btn1);
  lv_label_set_text(label, "Hold Me");
  lv_obj_center(label);

  // Create the label at the top of the screen to display the global main_counter
  main_counter_label = lv_label_create(lv_scr_act());
  char buf[16];
  lv_snprintf(buf, sizeof(buf), "Global: %d", main_counter);
  lv_label_set_text(main_counter_label, buf);
  lv_obj_align(main_counter_label, LV_ALIGN_TOP_MID, 0, 10);

  // Create the hold timer
  hold_timer = lv_timer_create(hold_timer_callback, 100, NULL);
  lv_timer_pause(hold_timer);
}
/*############################################
Functions communicating with Squareline Studio
they are here so they don't get deleted by SLS
#############################################*/

//_ui_label_set_property (ui_Label1, _UI_LABEL_PROPERTY_TEXT, String(main_counter).c_str());

//Get Values in UI==================================================
extern lv_obj_t *ui_SpeedSlider;  // Slider object
extern int set_speed;             // External integer to store the slider value

void get_speed_SpeedSlider(void) {
  // Get the current value of the slider
  set_speed = lv_slider_get_value(ui_SpeedSlider);
}

extern lv_obj_t *ui_SliderBrightness;
extern int brightness;
void get_SliderBrightness(void) {
  // Get the current value of the slider
  brightness = lv_slider_get_value(ui_SliderBrightness);
}
//Get Values in UI==================================================
//GPT:
// For Buzz checkbox
extern lv_obj_t *ui_BuzzCheck;
extern int Tapper_Buzz_Flag;

void get_buzz_check_value(void) {
  Tapper_Buzz_Flag = lv_obj_has_state(ui_BuzzCheck, LV_STATE_CHECKED) ? 1 : 0;
}

// For Buzz intensity dropdown
extern lv_obj_t *ui_BuzzPower;
extern int Tapper_B_Intensity;
extern lv_obj_t *ui_BuzzIntensitySlider;

void get_buzz_intensity_value(void) {
  //int selected_index = lv_dropdown_get_selected(ui_BuzzPower);
  //Tapper_B_Intensity = 50 + selected_index * 50;  // Assuming values 50, 100, 150, 200
  Tapper_B_Intensity = lv_slider_get_value(ui_BuzzIntensitySlider);  //
}

// For Buzz length dropdown (Other)
extern lv_obj_t *ui_BuzzLenght;
extern int Tapper_B_Other;
extern lv_obj_t *ui_BuzzLenghtSlider;
void get_buzz_length_value(void) {
  // int selected_index = lv_dropdown_get_selected(ui_BuzzLenght);
  // Tapper_B_Other = 1 + selected_index;  // Assuming values 1, 2, 3, 4
  Tapper_B_Other = lv_slider_get_value(ui_BuzzLenghtSlider);
}

// For Light checkbox
extern lv_obj_t *ui_LightCheck;
extern int Tapper_Light_Flag;

void get_light_check_value(void) {
  Tapper_Light_Flag = lv_obj_has_state(ui_LightCheck, LV_STATE_CHECKED) ? 1 : 0;
}

// For Light intensity dropdown
extern lv_obj_t *ui_LightIntensity;
extern int Tapper_L_Intensity;
extern lv_obj_t *ui_LightIntensitySlider;

void get_light_intensity_value(void) {
  //int selected_index = lv_dropdown_get_selected(ui_LightIntensity);
  //Tapper_L_Intensity = 100 + selected_index * 50;  // Assuming values 100, 150, 200, etc.
  Tapper_L_Intensity = lv_slider_get_value(ui_LightIntensitySlider);
}

// For Light color dropdown
extern lv_obj_t *ui_LightLenght;
extern int Tapper_L_Other;
extern lv_obj_t *ui_LightColourSlider;
void get_light_length_value(void) {
  // int selected_index = lv_dropdown_get_selected(ui_LightLenght);
  // Tapper_L_Other = 1 + selected_index;  // Assuming values 1, 2, 3, 4
Tapper_L_Other = lv_slider_get_value(ui_LightColourSlider);
}

// For Sound checkbox
extern lv_obj_t *ui_SoundCheck;
extern int Tapper_Sound_Flag;

void get_sound_check_value(void) {
  Tapper_Sound_Flag = lv_obj_has_state(ui_SoundCheck, LV_STATE_CHECKED) ? 1 : 0;
}

// For Sound intensity dropdown
extern lv_obj_t *ui_SoundVolume;
extern int Tapper_S_Intensity;
extern lv_obj_t *ui_SoundIntensitySlider;

void get_sound_intensity_value(void) {
  //int selected_index = lv_dropdown_get_selected(ui_SoundVolume);
  //Tapper_S_Intensity = 100 + selected_index * 50;  // Assuming values 100, 150, 200, etc.
Tapper_S_Intensity = lv_slider_get_value(ui_SoundIntensitySlider);
}

// For Sound length dropdown
extern lv_obj_t *ui_SoundLenght;
extern int Tapper_S_Other;
extern lv_obj_t *ui_SoundLenghtSlider;
void get_sound_length_value(void) {
  //int selected_index = lv_dropdown_get_selected(ui_SoundLenght);
  //Tapper_S_Other = 1 + selected_index;  // Assuming values 1, 2, 3, 4
  Tapper_S_Other = lv_slider_get_value(ui_SoundLenghtSlider);
}

// For Pressure checkbox
extern lv_obj_t *ui_PressureCheck;
extern int Tapper_Pressure_Flag;

void get_pressure_check_value(void) {
  Tapper_Pressure_Flag = lv_obj_has_state(ui_PressureCheck, LV_STATE_CHECKED) ? 1 : 0;
}

// For Pressure intensity dropdown
extern lv_obj_t *ui_PressureDropdown_1;
extern lv_obj_t *ui_PressureIntensitySlider;
extern int Tapper_P_Intensity;

void get_pressure_intensity_value(void) {
//  int selected_index = lv_dropdown_get_selected(ui_PressureDropdown_1);
//  Tapper_P_Intensity = 0 + selected_index * 50;  // Adjust intensity values as needed, getting 1,2,3,4
  Tapper_P_Intensity = lv_slider_get_value(ui_PressureIntensitySlider); //1-4 //5 to 250
}

// For Pressure length dropdown
extern lv_obj_t *ui_PressureDropdown_2;
extern lv_obj_t *ui_PressureSensitivitySlider;
extern int Tapper_P_Other;

void get_pressure_length_value(void) {
  //int selected_index = lv_dropdown_get_selected(ui_PressureDropdown_2);
  //Tapper_P_Other = 1 + selected_index;  // Assuming values 1, 2, 3, 4
  Tapper_P_Other = lv_slider_get_value(ui_PressureSensitivitySlider); //1,2,3,4
}

//Set Values in UI==================================================
// For Speed Slider
/*extern lv_obj_t *ui_SpeedSlider;
extern lv_obj_t *ui_SliderValueLabel;
extern int set_speed;

void set_speed_SpeedSlider(void) {
    lv_slider_set_value(ui_SpeedSlider, set_speed, LV_ANIM_OFF);
}
*/
extern lv_obj_t *ui_SpeedSlider;
extern lv_obj_t *ui_SliderValueLabel;
extern int set_speed;

void set_speed_SpeedSlider(void) {
  // Set the slider to the value of set_speed
  lv_slider_set_value(ui_SpeedSlider, set_speed, LV_ANIM_OFF);

  // Update the label to display the current set_speed
  char buf[8];                                     // Buffer to hold the text
  lv_snprintf(buf, sizeof(buf), "%d", set_speed);  // Convert the integer to string
  lv_label_set_text(ui_SliderValueLabel, buf);     // Set the text of the label
}



//Set Values in UI==================================================

// For Buzz checkbox
extern lv_obj_t *ui_BuzzCheck;
extern int Tapper_Buzz_Flag;

void set_buzz_check_value(void) {
    Serial.println("set_buzz_check_value");
    if (Tapper_Buzz_Flag == 1) {
        lv_obj_add_state(ui_BuzzCheck, LV_STATE_CHECKED);
        lv_label_set_text(ui_BuzzOnOffLabel, "");                // This will make the label text invisible
        lv_obj_add_flag(ui_BuzzOnOffLabel, LV_OBJ_FLAG_HIDDEN);  // This will hide the label
    } else {
        lv_obj_clear_state(ui_BuzzCheck, LV_STATE_CHECKED);
    }
}

// For Buzz intensity dropdown and slider
extern lv_obj_t *ui_BuzzPower;               // Dropdown for buzz intensity
extern lv_obj_t *ui_BuzzIntensitySlider;     // Slider for buzz intensity
extern int Tapper_B_Intensity;

void set_buzz_intensity_value(void) {
    int selected_index = (Tapper_B_Intensity - 50) / 50;  // Reverse calculation for dropdown
    lv_dropdown_set_selected(ui_BuzzPower, selected_index);  // Update dropdown
    lv_slider_set_value(ui_BuzzIntensitySlider, Tapper_B_Intensity, LV_ANIM_OFF);  // Update slider
}

// For Buzz length dropdown and slider
extern lv_obj_t *ui_BuzzLenght;              // Dropdown for buzz length
extern lv_obj_t *ui_BuzzLenghtSlider;        // Slider for buzz length
extern int Tapper_B_Other;

void set_buzz_length_value(void) {
    lv_dropdown_set_selected(ui_BuzzLenght, Tapper_B_Other - 1);  // Update dropdown
    lv_slider_set_value(ui_BuzzLenghtSlider, Tapper_B_Other, LV_ANIM_OFF);  // Update slider
}

// For Light checkbox
extern lv_obj_t *ui_LightCheck;
extern int Tapper_Light_Flag;

void set_light_check_value(void) {
    if (Tapper_Light_Flag == 1) {
        lv_obj_add_state(ui_LightCheck, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_LightCheck, LV_STATE_CHECKED);
    }
}

// For Light intensity dropdown and slider
extern lv_obj_t *ui_LightIntensity;          // Dropdown for light intensity
extern lv_obj_t *ui_LightIntensitySlider;    // Slider for light intensity
extern int Tapper_L_Intensity;

void set_light_intensity_value(void) {
    int selected_index = (Tapper_L_Intensity - 100) / 50;  // Reverse calculation for dropdown
    lv_dropdown_set_selected(ui_LightIntensity, selected_index);  // Update dropdown
    lv_slider_set_value(ui_LightIntensitySlider, Tapper_L_Intensity, LV_ANIM_OFF);  // Update slider
}

// For Light length dropdown and slider
extern lv_obj_t *ui_LightLenght;             // Dropdown for light length
extern lv_obj_t *ui_LightColourSlider;       // Slider for light color/length
extern int Tapper_L_Other;

void set_light_length_value(void) {
    lv_dropdown_set_selected(ui_LightLenght, Tapper_L_Other - 1);  // Update dropdown
    lv_slider_set_value(ui_LightColourSlider, Tapper_L_Other, LV_ANIM_OFF);  // Update slider
}

// For Sound checkbox
extern lv_obj_t *ui_SoundCheck;
extern int Tapper_Sound_Flag;

void set_sound_check_value(void) {
    if (Tapper_Sound_Flag == 1) {
        lv_obj_add_state(ui_SoundCheck, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_SoundCheck, LV_STATE_CHECKED);
    }
}

// For Sound intensity dropdown and slider
extern lv_obj_t *ui_SoundVolume;             // Dropdown for sound intensity
extern lv_obj_t *ui_SoundIntensitySlider;    // Slider for sound intensity
extern int Tapper_S_Intensity;

void set_sound_intensity_value(void) {
    int selected_index = (Tapper_S_Intensity - 100) / 50;  // Reverse calculation for dropdown
    lv_dropdown_set_selected(ui_SoundVolume, selected_index);  // Update dropdown
    lv_slider_set_value(ui_SoundIntensitySlider, Tapper_S_Intensity, LV_ANIM_OFF);  // Update slider
}

// For Sound length dropdown and slider
extern lv_obj_t *ui_SoundLenght;             // Dropdown for sound length
extern lv_obj_t *ui_SoundLenghtSlider;       // Slider for sound length
extern int Tapper_S_Other;

void set_sound_length_value(void) {
    lv_dropdown_set_selected(ui_SoundLenght, Tapper_S_Other - 1);  // Update dropdown
    lv_slider_set_value(ui_SoundLenghtSlider, Tapper_S_Other, LV_ANIM_OFF);  // Update slider
}

// For Pressure checkbox
extern lv_obj_t *ui_PressureCheck;
extern int Tapper_Pressure_Flag;

void set_pressure_check_value(void) {
  if (Tapper_Pressure_Flag == 1) {
    lv_obj_add_state(ui_PressureCheck, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_PressureCheck, LV_STATE_CHECKED);
  }
}

// For Pressure intensity dropdown
extern lv_obj_t *ui_PressureDropdown_1;
extern lv_obj_t *ui_PressureIntensitySlider;
extern int Tapper_P_Intensity;

void set_pressure_intensity_value(void) {
  int selected_index = Tapper_P_Intensity;//1-4 / 50;  // Reverse calculation
  lv_dropdown_set_selected(ui_PressureDropdown_1, selected_index);
  lv_slider_set_value(ui_PressureIntensitySlider, Tapper_P_Intensity, LV_ANIM_OFF);  // Update slider

}

// For Pressure length dropdown
extern lv_obj_t *ui_PressureDropdown_2;
extern lv_obj_t *ui_PressureSensitivitySlider;
extern int Tapper_P_Other;

void set_pressure_length_value(void) {
  lv_dropdown_set_selected(ui_PressureDropdown_2, Tapper_P_Other - 1);  // Reverse calculation
  lv_slider_set_value(ui_PressureSensitivitySlider, Tapper_P_Other - 1, LV_ANIM_OFF);  // Update slider

}
#endif  //MOD_LV.h
//#endif
