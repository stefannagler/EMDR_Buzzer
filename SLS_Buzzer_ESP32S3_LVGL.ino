//start again with working vers 36 to implement GPIO 0 sleep mode
// SLS_Buzzer_ESP32S3_LVGL Refactore setup and loop and GPIO but saving broke,  this has all the functionality to use GPOI 0 for sleep and wakeup
// but lets tidy code here and then start again

///Users/stefannagler/SquareLine/BLE_ESP32S3_AMOLED/AMOLED_BLE_Handmade_v1.0.1/BLEimplementationS3/Button_Label_touchButton/AddSLS/SLS_Buzzer_ESP32S3_LVGL
//SLS_Buzzer_ESP32S3_LVGL.ino

//esp32s3 dev Liliy go AMOLED , Flash 16, Partition 16,3,9.5, PSRAM OPI
//SLS_Buzzer_ESP32S3_LVGL.ino.  lvgl library 8.3.11

#include <XPowersLibInterface.hpp>
//works down to battery 2.55V, 3.2V gives 10-15min runtime
//batery percentages are displayed
#include <NimBLEDevice.h>  // Use NimBLE library
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#include "SLS_BLE_Setup.h"
#include "esp_sleep.h"
#include "ui.h"
#include "BatteryMonitor_scaled_percentage.h"
#include <AceButton.h>
#include <Preferences.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FIRMWARE "vers 18"
//18 
//17  simplified and globalized UI update including sending to BLE, in pressure mode the lights are not changable which is done in central in actOn_SelectLeftRight_40()
//15 pressure implementation
//14 some more refactoring(sleep stuff),
//13 implement pressure to come
//12  it is working (no pressure yet), BLE reconnects now since connect variables are not in rtc so they get reset in sleepmode,, update_on_off_button
//10 prints mem allocation
//8 still implement the Hold to sleep label
//7 seems to store propperly

#define debug 1   //turns serial port on
#define debugP 0  //turns serial port on for power monitoring
// search for :   ^\s*(Serial\.[^\n]*); replace with:  if (debug) { $1; }
#define BLECharacteristicCallbacks NimBLECharacteristicCallbacks
#define BLECharacteristic NimBLECharacteristic

Preferences preferences;

using namespace ace_button;

/*RTC_DATA_ATTR */ const int BUTTON_PIN = 0;  // GPIO 0 button pin
/*RTC_DATA_ATTR */ AceButton button(BUTTON_PIN);
/*RTC_DATA_ATTR */ ButtonConfig *buttonConfig;      // ButtonConfig object
/*RTC_DATA_ATTR*/ bool amoled_initialized = false;  // Flag to track if AMOLED screen is initialized
RTC_DATA_ATTR bool readyToSleep = false;            // Flag to track if we are waiting for button release to sleep

RTC_DATA_ATTR int sleepCycleCount = 0;              // Tracks sleep cycles across deep sleep
RTC_DATA_ATTR const char *flashKey = "sleepCycle";  // Flash storage key
// Forward declaration
void handleEvent(AceButton *, uint8_t, uint8_t);

RTC_DATA_ATTR unsigned long lastStoreTime = 0;  // Tracks last store time to flash

LilyGo_Class amoled;
lv_obj_t *home_button_label;     // For touchpad button control
lv_obj_t *touch_position_label;  // For touchpad button control
lv_obj_t *GPIO_btn_test_label;
lv_obj_t *current_screen;
extern "C++" {  // C++ section for AMOLED operations
  void lv_example_btn_1_1_Hold_Me(void);
  void update_main_counter_label(void);
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
  void daily_storeVariablesToFlash(void);
  void getvbat_per(void);
};

void buzzerControl();
void settings_to_display(void);
void centralWatchdog();
void checkGoToSleepViaInactivity();
void goToSleepViaButtonRelease();
void updateUIValues();    //get variables to RTC by reading the display widgets
void pressureSelected();  //called in buzzerControl() if pressure tick is set , save the ticks but then set them to 0 and send to UI via settingsToDisplay() and send the BLE commands to set them 0 and the pressure one to 1 including the other pressure settings
void ifSettingsChangedDoBLE();//check against variables prev_Tapper_....


RTC_DATA_ATTR uint8_t vbat_per;
RTC_DATA_ATTR byte old_vbat_per;
#define SOC_STEPS_ARRAY 21
RTC_DATA_ATTR float LIPOChart_Used_SOC_to_Percent[SOC_STEPS_ARRAY] = { 4.04, 4.00, 3.97, 3.93, 3.90, 3.86, 3.81, 3.76, 3.71, 3.67, 3.64, 3.62, 3.61, 3.58, 3.56, 3.51, 3.46, 3.40, 3.27, 3.1, 3.0 };

RTC_DATA_ATTR unsigned long long lastDayStoreTime = 0;
RTC_DATA_ATTR const unsigned long long oneDayInMicroseconds = 86400000000ULL;

/*RTC_DATA_ATTR */ bool deviceConnected = false;
/*RTC_DATA_ATTR*/ bool oldDeviceConnected = false;
RTC_DATA_ATTR uint32_t value1 = false;
RTC_DATA_ATTR uint32_t testValue1 = false;
RTC_DATA_ATTR unsigned long previousDevConnectedMillis = 0;
RTC_DATA_ATTR const unsigned long notificationInterval = 1000;
RTC_DATA_ATTR unsigned long previousUpdateBuzzerMillis = 0;
RTC_DATA_ATTR const unsigned long notificationUpdateBuzzerInterval = 10000;
RTC_DATA_ATTR unsigned long currentDevConnectedMillis;
RTC_DATA_ATTR unsigned long currentMillis;

RTC_DATA_ATTR int Tapper_OnOff_Flag = 0;  // changes in TapperStartStopButtonFunction() when stat Tapper button gets pressed
RTC_DATA_ATTR int Start_Stop_Flag = 1;    // can stop everything
RTC_DATA_ATTR int Tapper_Buzz_Flag = 0;
RTC_DATA_ATTR int Tapper_B_Intensity = 50;
RTC_DATA_ATTR int Tapper_B_Other = 3;
RTC_DATA_ATTR int Tapper_Light_Flag = 0;
RTC_DATA_ATTR int Tapper_L_Intensity = 200;
RTC_DATA_ATTR int Tapper_L_Other = 2;
RTC_DATA_ATTR int Tapper_Sound_Flag = 0;
RTC_DATA_ATTR int Tapper_S_Intensity = 100;
RTC_DATA_ATTR int Tapper_S_Other = 2;
RTC_DATA_ATTR int Tapper_Pressure_Flag = 0;
RTC_DATA_ATTR int Tapper_P_Intensity = 1;
RTC_DATA_ATTR int Tapper_P_Other = 1;
RTC_DATA_ATTR int set_speed = 1;

RTC_DATA_ATTR int on_off = 1;  //in RTC so it needs software reset when the on_off button is pressed and flash mem is stored
RTC_DATA_ATTR int main_counter = 150;
RTC_DATA_ATTR int brightness = 200;

RTC_DATA_ATTR int last_Tapper_OnOff_Flag = Tapper_OnOff_Flag;
RTC_DATA_ATTR int last_Start_Stop_Flag = Start_Stop_Flag;
RTC_DATA_ATTR int last_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
RTC_DATA_ATTR int last_Tapper_B_Intensity = Tapper_B_Intensity;
RTC_DATA_ATTR int last_Tapper_B_Other = Tapper_B_Other;
RTC_DATA_ATTR int last_Tapper_Light_Flag = Tapper_Light_Flag;
RTC_DATA_ATTR int last_Tapper_L_Intensity = Tapper_L_Intensity;
RTC_DATA_ATTR int last_Tapper_L_Other = Tapper_L_Other;
RTC_DATA_ATTR int last_Tapper_Sound_Flag = Tapper_Sound_Flag;
RTC_DATA_ATTR int last_Tapper_S_Intensity = Tapper_S_Intensity;
RTC_DATA_ATTR int last_Tapper_S_Other = Tapper_S_Other;
RTC_DATA_ATTR int last_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
RTC_DATA_ATTR int last_Tapper_P_Intensity = Tapper_P_Intensity;
RTC_DATA_ATTR int last_Tapper_P_Other = Tapper_P_Other;
RTC_DATA_ATTR int last_set_speed = set_speed;
RTC_DATA_ATTR int last_main_counter = main_counter;

RTC_DATA_ATTR int prev_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
RTC_DATA_ATTR int prev_Tapper_B_Intensity = Tapper_B_Intensity;
RTC_DATA_ATTR int prev_Tapper_B_Other = Tapper_B_Other;
RTC_DATA_ATTR int prev_Tapper_Light_Flag = Tapper_Light_Flag;
RTC_DATA_ATTR int prev_Tapper_L_Intensity = Tapper_L_Intensity;
RTC_DATA_ATTR int prev_Tapper_L_Other = Tapper_L_Other;
RTC_DATA_ATTR int prev_Tapper_Sound_Flag = Tapper_Sound_Flag;
RTC_DATA_ATTR int prev_Tapper_S_Intensity = Tapper_S_Intensity;
RTC_DATA_ATTR int prev_Tapper_S_Other = Tapper_S_Other;
RTC_DATA_ATTR int prev_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
RTC_DATA_ATTR int prev_Tapper_P_Intensity = Tapper_P_Intensity;
RTC_DATA_ATTR int prev_Tapper_P_Other = Tapper_P_Other;

RTC_DATA_ATTR unsigned long lastUpdateCL = 0;
RTC_DATA_ATTR unsigned long lastUpdateCR = 0;
#define HEARTBEAT_RECEIVE 12000
RTC_DATA_ATTR byte centralWatchdogWasTriggered = 0;
RTC_DATA_ATTR byte previouscentralWatchdogWasTriggered = 0;
RTC_DATA_ATTR unsigned long lastHeartbeat = 0;

RTC_DATA_ATTR unsigned long previousMillis = 0;
RTC_DATA_ATTR unsigned long previousIntervallMillis = 0;

RTC_DATA_ATTR long left_right_interval = 500;
RTC_DATA_ATTR const long global_counter_interval = 1000;
RTC_DATA_ATTR const int ledPin = LED_BUILTIN;
RTC_DATA_ATTR const int buttonPin = 4;

RTC_DATA_ATTR int left_right_Flag = 0;

RTC_DATA_ATTR unsigned long homeButtonPressTime = 0;
RTC_DATA_ATTR bool isLabelVisible = false;
RTC_DATA_ATTR int batVLeft = 1;
RTC_DATA_ATTR int batVRight = 1;
RTC_DATA_ATTR int Spare1 = 0;
RTC_DATA_ATTR int Spare2 = 0;
RTC_DATA_ATTR int Spare3 = 0;
RTC_DATA_ATTR int Spare4 = 0;
RTC_DATA_ATTR int Spare5 = 0;
RTC_DATA_ATTR int vbat1_perCharge = 0;
RTC_DATA_ATTR int vbat2_perCharge = 0;
RTC_DATA_ATTR int Spare1_was_written_Flag = 0;
RTC_DATA_ATTR int vBat1_was_written_Flag = 0;
RTC_DATA_ATTR int vBat1_Heartbeat_written_Flag = 0;
RTC_DATA_ATTR int vBat2_was_written_Flag = 0;
RTC_DATA_ATTR int vBat2_Heartbeat_written_Flag = 0;

/*RTC_DATA_ATTR*/ unsigned long lastTouchTime = 0;  //needs a reset as it seems to keep counting in sleep mode
#define INACTIVITY_TIMEOUT 600000

RTC_DATA_ATTR int setCounter = 0;

RTC_DATA_ATTR int rtcMarker = 0;
RTC_DATA_ATTR const int expectedMarkerValue = 12345;

//########################################################################
//########################################################################
void setup() {
  if (debug) { Serial.begin(115200); }  //may save power similar to disabeling USB CDC in Tools
  //while (!Serial);
  //delay(1000);
  Serial.println(FIRMWARE);
  //Schedule memory monitoring every 10 seconds (for example)
  // xTaskCreate([](void *) {
  //   while (true) {
  //     printMemoryStats();
  //     vTaskDelay(pdMS_TO_TICKS(10000));  // Delay 10 seconds
  //   }
  // },
  //             "MemoryMonitor", 2048, NULL, 1, NULL);


  //=================================================================
  //=================================================================
  //Initialize the board depending on boot up mode

  // Check for the reset cause to see if sleep cycle count is 0 which means reset
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0 && sleepCycleCount == 0) {
    // If not waking up from deep sleep and sleepCycleCount is 0, retrieve from flash
    retrieveVariablesFromFlash();

    //sleepCycleCount = preferences.getInt(flashKey, 0);  // Default to 0 if no value is stored
    Serial.printf("[DEBUG] Retrieved variables from Flash and sleepCycleCount from RTC memory: %d\n", sleepCycleCount);
  }

  // Increment the sleep cycle counter each time the device wakes up
  sleepCycleCount++;
  Serial.printf("[DEBUG] Sleep cycle count: %d\n", sleepCycleCount);

  // Check if the device woke up from deep sleep due to GPIO 0
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("[DEBUG] Woke up from deep sleep due to GPIO 0 (Home Button)");
    Tapper_OnOff_Flag = 0;  //start
    // Reinitialize the display and touch after waking up from deep sleep
    if (!amoled_initialized) {
      Serial.println("[DEBUG] Initializing AMOLED and LVGL after wake-up...");

      if (!amoled.begin()) {
        Serial.println("[ERROR] Failed to initialize the AMOLED display!");
        while (1)
          ;
      }
      //beginLvglHelper(amoled);
      amoled_initialized = true;  // Mark as initialized
    }
  }

  // Only initialize the screen once during the first setup
  if (!amoled_initialized) {
    bool rslt = amoled.begin();
    if (!rslt) {
      while (1) {
        Serial.println("The board model cannot be detected, please raise the Core Debug Level to an error");
        delay(1000);
      }
    }

    // Register LVGL helper
    //beginLvglHelper(amoled);
    amoled_initialized = true;  // Mark as initialized
  }

  amoled.setRotation(1);
  amoled.setBrightness(200);
  beginLvglHelper(amoled);


  // Initialize the button pin and AceButton event handler
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Get button configuration object and set features
  buttonConfig = button.getButtonConfig();
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);  // Enable long press detection

  // Set the long press duration (default is 1000 ms, but we set to 1.5 seconds here)
  buttonConfig->setLongPressDelay(1500);

  button.setEventHandler(handleEvent);
  Serial.println("[DEBUG] AceButton event handler configured");

  //end of the GPIO 0 setup ===============================================

  ui_init();

  lv_example_btn_1_1_Hold_Me();

  // Initialize BLE
  initBLE();

  // Create GPIO_btn_test_label for touch coordinates
  GPIO_btn_test_label = lv_label_create(lv_scr_act());
  lv_label_set_text(GPIO_btn_test_label, FIRMWARE);
  lv_obj_set_style_text_font(GPIO_btn_test_label, &lv_font_montserrat_20, 0);
  lv_obj_center(GPIO_btn_test_label);

  // Create the label for the touchposition
  touch_position_label = lv_label_create(lv_scr_act());
  lv_label_set_text(touch_position_label, "Touch");
  lv_obj_set_style_text_font(touch_position_label, &lv_font_montserrat_24, 0);
  lv_obj_center(touch_position_label);  // Center the label
  // Make the label invisible
  lv_obj_add_flag(touch_position_label, LV_OBJ_FLAG_HIDDEN);

  // Create the label for the touchpad button
  home_button_label = lv_label_create(lv_scr_act());
  lv_label_set_text(home_button_label, "Touch test");
  lv_obj_set_style_text_font(home_button_label, &lv_font_montserrat_20, 0);
  lv_obj_align(home_button_label, LV_ALIGN_CENTER, 0, +40);
  // Make the label invisible
  lv_obj_add_flag(home_button_label, LV_OBJ_FLAG_HIDDEN);

  amoled.setHomeButtonCallback([](void *ptr) {
    if (debug) { Serial.println("Home key pressed!"); }
    // _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
    // _ui_screen_delete(&ui_Screen2);
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen2_screen_init);
    _ui_screen_delete(&ui_Screen1);
    _ui_screen_delete(&ui_Screen3);
    _ui_screen_delete(&ui_Screen4);
    if (!isLabelVisible) {
      // Show the label and update text
      lv_label_set_text(home_button_label, "Home Pressed 1s");
      lv_obj_align(home_button_label, LV_ALIGN_CENTER, 0, +40);
      lv_obj_clear_flag(home_button_label, LV_OBJ_FLAG_HIDDEN);  // Make the label visible

      // Set the press time and mark the label as visible
      homeButtonPressTime = millis();
      isLabelVisible = true;
    }
  },
                               NULL);

  // Print the initial RTC marker value for debugging
  Serial.print("Initial RTC Marker value: ");
  Serial.println(rtcMarker);

  //retrieveVariablesFromFlash(); //only done on reset type reboot
  settings_to_display();
  initBuzzers();
  //store the values so that before shutting down only changed values get put to flash
  last_Tapper_OnOff_Flag = Tapper_OnOff_Flag;
  last_Start_Stop_Flag = Start_Stop_Flag;
  last_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
  last_Tapper_B_Intensity = Tapper_B_Intensity;
  last_Tapper_B_Other = Tapper_B_Other;
  last_Tapper_Light_Flag = Tapper_Light_Flag;
  last_Tapper_L_Intensity = Tapper_L_Intensity;
  last_Tapper_L_Other = Tapper_L_Other;
  last_Tapper_Sound_Flag = Tapper_Sound_Flag;
  last_Tapper_S_Intensity = Tapper_S_Intensity;
  last_Tapper_S_Other = Tapper_S_Other;
  last_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
  last_Tapper_P_Intensity = Tapper_P_Intensity;
  last_Tapper_P_Other = Tapper_P_Other;
  last_set_speed = set_speed;
  last_main_counter = main_counter;

  // Get the current time in microseconds (since boot)
  lastDayStoreTime = esp_timer_get_time();  // Initialize with the boot time or last known time

  lv_obj_set_style_text_color(ui_BuzzCheck, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_CHECKED);   // Black tick color
  lv_obj_set_style_text_color(ui_LightCheck, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_CHECKED);  //   tick color
  lv_obj_set_style_text_color(ui_SoundCheck, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_CHECKED);  //   tick color
}  //setup

//########################################################################
//########################################################################
void loop() {

  static int16_t x, y;
  // bool touched = amoled.getPoint(&x, &y);
  // if (touched) {
  //   Serial.printf("X:%d Y:%d\n", x, y);
  //   lv_label_set_text_fmt(touch_position_label, "X:%d Y:%d", x, y);
  //   lv_obj_center(touch_position_label);
  // }

  // Check if 1 second has passed since the label became visible
  if (isLabelVisible && millis() - homeButtonPressTime >= 1000) {
    // Hide the label after 1 second
    lv_obj_add_flag(home_button_label, LV_OBJ_FLAG_HIDDEN);  // Hide the label
    isLabelVisible = false;                                  // Mark the label as not visible
  }

  lv_task_handler();
  delay(5);

  //buzzerControl();

  // Test of a global counter to display
  currentMillis = millis();
  if (currentMillis - previousMillis >= global_counter_interval) {
    previousMillis = currentMillis;
    // Serial.print("Batvoltage/percent.  ");
    // Serial.println(amoled.getBattVoltage());
    // Serial.println(amoled.getBatteryPercent());
    updateBatteryLabels();
    if (debugP) { Serial.println("Power management"); }
    //Serial.println(amoled.getBatfetDieOverTempLevel1() );
    amoled.setLowBatWarnThreshold(11);  //%
    amoled.setLowBatShutdownThreshold(10);
    if (debugP) { Serial.println(amoled.getLowBatWarnThreshold()); }
    if (debugP) { Serial.println(amoled.getLowBatShutdownThreshold()); }
    if (debugP) { Serial.println(amoled.getLinearChargerVsysDpm()); }
    if (debugP) { Serial.println(".......... "); }
    main_counter++;
    if (main_counter > 255) {
      main_counter = 100;
    }
    if (main_counter < 1) {
      main_counter = 50;
    }

    get_SliderBrightness();
    amoled.setBrightness((uint8_t)brightness);
  }

  currentDevConnectedMillis = millis();

  if (deviceConnected) {
    if (currentDevConnectedMillis - previousDevConnectedMillis >= notificationInterval) {
      previousDevConnectedMillis = currentDevConnectedMillis;
      // Notify characteristics
      delay(delayInBetweenBLE);
      ++value1;
    }

    if (currentDevConnectedMillis - previousUpdateBuzzerMillis >= notificationUpdateBuzzerInterval) {
      previousUpdateBuzzerMillis = currentDevConnectedMillis;
      if (Start_Stop_Flag && Tapper_OnOff_Flag) {
        initBuzzers();  // Keep buzzers updated in case they briefly disconnected
        if (debug) { Serial.println("Buzzer periodic update"); }
      }
    }
  }

  // Reconnect logic
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                        // Small delay before restarting advertising
    NimBLEDevice::startAdvertising();  // Restart advertising

    if (debug) {
      Serial.println("[DEBUG] Device disconnected. Restarting advertising...");
      Serial.println("[DEBUG] Advertising started. Awaiting new connection...");
    }

    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    // Reconnection event
    oldDeviceConnected = deviceConnected;
    NimBLEDevice::startAdvertising();

    if (debug) {
      Serial.println("[DEBUG] Device reconnected successfully.");
      Serial.println("[DEBUG] Re-initiating advertising for client notifications...");
    }
    // retrieveVariablesFromFlash();
    // settings_to_display();
  }

  //but store once a day anyway
  daily_storeVariablesToFlash();
  buzzerControl();

  // Check for button events
  button.check();

  // If we are ready to sleep, wait until the button is released
  if (readyToSleep) {
    goToSleepViaButtonRelease();
  }
  //inactivity??
  if (amoled.getPoint(&x, &y)) {
    lastTouchTime = millis();  // Update the last touch time
    //Serial.printf("X:%d Y:%d\n", x, y);
    //lv_label_set_text_fmt(touch_position_label, "X:%d Y:%d", x, y);
    //lv_obj_center(touch_position_label);
  }
  checkGoToSleepViaInactivity();


  centralWatchdog();              //12 second check but watchdog in centrals is 5 seconds to prevent wtd trigger here
                                  //Potential Bug: if one buzzer is off already, when the second goes off suddenly then Spare1_was_written_Flag does not get set because Spare1 is not being sent by the buzzer
  if (Spare1_was_written_Flag) {  // it was set on receiving Spare1 from Central , so it gets passed from Central to Peripheral and then Peripheral to APP and central, that way each central can change a bit
    pChar_Spare1->setValue(Spare1);
    pChar_Spare1->notify();
    delay(delayInBetweenBLE);
    Spare1_was_written_Flag = 0;
    if (debug) { Serial.println("Spare1 was received and then notified by peripheral in Task1code: " + String(Spare1)); }
  }

  // Check the value of Spare1 and update the label in screen 1 accordingly
  switch (Spare1) {
    case 0:
      // Spare1 is 0 -> Display "No Buzzers"
      lv_label_set_text(ui_ConnectBuzzersLabel, "No Buzzers");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel, LV_OBJ_FLAG_HIDDEN);  // Make sure label is visible
      lv_label_set_text(ui_ConnectBuzzersLabel1, "No Buzzers");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel1, LV_OBJ_FLAG_HIDDEN);  // Make sure label1 is visible
      break;
    case 1:
      // Spare1 is 1 -> Display "No Left Buzzer"
      lv_label_set_text(ui_ConnectBuzzersLabel, "No Left Buzzer");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel, LV_OBJ_FLAG_HIDDEN);  // Make sure label is visible
      lv_label_set_text(ui_ConnectBuzzersLabel1, "No Left Buzzer");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel1, LV_OBJ_FLAG_HIDDEN);  // Make sure label1 is visible
      break;
    case 2:
      // Spare1 is 2 -> Display "No Right Buzzer"
      lv_label_set_text(ui_ConnectBuzzersLabel, "No Right Buzzer");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel, LV_OBJ_FLAG_HIDDEN);  // Make sure label is visible
      lv_label_set_text(ui_ConnectBuzzersLabel1, "No Right Buzzer");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel1, LV_OBJ_FLAG_HIDDEN);  // Make sure label1 is visible
      break;
    case 3:
      // Spare1 is 3 -> Hide both labels
      lv_obj_add_flag(ui_ConnectBuzzersLabel, LV_OBJ_FLAG_HIDDEN);   // Hide the label
      lv_obj_add_flag(ui_ConnectBuzzersLabel1, LV_OBJ_FLAG_HIDDEN);  // Hide label1
      break;
    default:
      // Handle any unexpected values, show a default message
      lv_label_set_text(ui_ConnectBuzzersLabel, "Unknown Buzzer Status");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel, LV_OBJ_FLAG_HIDDEN);  // Make sure label is visible
      lv_label_set_text(ui_ConnectBuzzersLabel1, "Unknown Buzzer Status");
      lv_obj_clear_flag(ui_ConnectBuzzersLabel1, LV_OBJ_FLAG_HIDDEN);  // Make sure label1 is visible
      break;
  }


}  //loop

//########################################################################
//########################################################################

void goToSleepViaButtonRelease() {
  // Get the current screen and set its background color to white
  current_screen = lv_scr_act();
  lv_obj_set_style_bg_color(current_screen, lv_color_hex(0xFFFFFF), 0);  // Set background to white
  lv_obj_set_style_bg_opa(current_screen, LV_OPA_COVER, 0);              // Full opacity

  lv_label_set_text(GPIO_btn_test_label, "");  //erase
  GPIO_btn_test_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(GPIO_btn_test_label, &lv_font_montserrat_30, 0);
  lv_obj_set_style_text_color(GPIO_btn_test_label, lv_color_hex(0xFF0000), 0);
  lv_label_set_long_mode(GPIO_btn_test_label, LV_LABEL_LONG_WRAP);  // Enable text wrapping
  lv_obj_set_width(GPIO_btn_test_label, 200);                       // Set width to control where text wraps
  lv_obj_align(GPIO_btn_test_label, LV_ALIGN_BOTTOM_MID, 0, -150);
  lv_label_set_text(GPIO_btn_test_label, "Release to sleep");
  //lv_obj_center(GPIO_btn_test_label);

  if (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("[DEBUG] Button released! Entering deep sleep...");
    readyToSleep = false;

    // Turn off all peripherals and screen
    amoled.setBrightness(0);  // Manually turn off display

    // Ensure WiFi, Bluetooth, and other peripherals are off (if applicable)
    //esp_wifi_stop();
    //btStop();

    // Prepare for deep sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake up when GPIO 0 is low

    // Enter deep sleep
    esp_deep_sleep_start();
  }
}

void checkGoToSleepViaInactivity() {
  if (millis() - lastTouchTime >= INACTIVITY_TIMEOUT) {
    if (debug) { Serial.println("Entering deep sleep due to inactivity not storing to Flash..."); }
    //storeVariablesToFlash();

    // Prepare for deep sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake up when GPIO 0 is low

    amoled.setBrightness(0);  // Optionally turn off the display before sleep
    esp_deep_sleep_start();   // Enter deep sleep
    // Enable sleep mode with touchpad wake-up enabled
    //   amoled.sleep(true);  // 'true' to enable waking up by touchpad (e.g., home button)
  }
}
//get UI display values
void updateUIValues() {
  static unsigned long lastUpdateMillis = 0;  // Store the last update time

  // Check if 1 second (1000 milliseconds) has passed since the last update
  if (millis() - lastUpdateMillis >= 1000) {
    lastUpdateMillis = millis();  // Update the last update time

    // Get current UI values for each component
    get_speed_SpeedSlider();
    get_buzz_check_value();
    get_buzz_intensity_value();
    get_buzz_length_value();
    get_light_check_value();
    get_light_intensity_value();
    get_light_length_value();
    get_sound_check_value();
    get_sound_intensity_value();
    get_sound_length_value();
    get_pressure_check_value();
    get_pressure_intensity_value();
    get_pressure_length_value();
  }
ifSettingsChangedDoBLE();// compare against prev_Tapper_....and send to BLE
}

void buzzerControl() {

  updateUIValues();  // get values from the display so they update the RTC values and send to BLE
  //ifSettingsChangedDoBLE();

  // Check if Tapper_Pressure_Flag has transitioned from 0 to 1
  if (prev_Tapper_Pressure_Flag == 0 && Tapper_Pressure_Flag == 1) {
    Serial.print("Pressure Function active:  ");
    Serial.print(Tapper_P_Intensity);
    Serial.print("  ");
    Serial.println(Tapper_P_Other);
    pressureSelected();  // Call pressureSelected() only on 0 to 1 transition
  }
  // Check for transition from Tapper_Pressure_Flag 1 to 0 and restore normal operation
  if (prev_Tapper_Pressure_Flag == 1 && Tapper_Pressure_Flag == 0) {
    // Add the code you want to execute when Tapper_Pressure_Flag goes from 1 to 0
    Serial.println("Tapper_Pressure_Flag transitioned from 1 to 0");
    initBuzzers();

    pChar_Tapper_Pressure_Flag->setValue(0);  //Tapper_Pressure_Flag);//Tapper_Pressure_Flag is 0 because the UI set it to 0
    pChar_Tapper_Pressure_Flag->notify();
    delay(delayInBetweenBLE);
  }

  // Update prev_Tapper_Pressure_Flag to the current state for next comparison
  prev_Tapper_Pressure_Flag = Tapper_Pressure_Flag;

  if (!Tapper_Pressure_Flag) {  //not in pressure mode, normal operation
    if (Start_Stop_Flag && Tapper_OnOff_Flag) {
      left_right_interval = 1200 - set_speed * 1000 / 50;  // fasted speed is 200=(1200-1000)
      if (millis() - previousIntervallMillis >= left_right_interval) {
        if (Tapper_Buzz_Flag || Tapper_Light_Flag || Tapper_Sound_Flag) {
          // Make the label invisible if any flag is true
          lv_obj_add_flag(ui_isCheckBoxLabel, LV_OBJ_FLAG_HIDDEN);
        } else {
          // Set the label text and make it visible if none of the flags are true
          lv_label_set_text(ui_isCheckBoxLabel, "Select Mode on next screen");
          lv_obj_clear_flag(ui_isCheckBoxLabel, LV_OBJ_FLAG_HIDDEN);

          // Set the width of the label to allow text wrapping
          lv_obj_set_width(ui_isCheckBoxLabel, 150);  // Adjust width as needed

          // Enable text wrapping
          lv_label_set_long_mode(ui_isCheckBoxLabel, LV_LABEL_LONG_WRAP);

          // Center the label on the x-axis
          //lv_obj_set_align(ui_isCheckBoxLabel, LV_ALIGN_TOP_MID);
        }
        // Toggle theleft_right_Flag value (Left/Right)
        left_right_Flag = !left_right_Flag;
        if (left_right_Flag) {  //blink the background of button
          if (Tapper_Buzz_Flag || Tapper_Light_Flag || Tapper_Sound_Flag) { lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0x00A5A5), LV_PART_MAIN | LV_STATE_DEFAULT); }
          // no function selected
          else {
            lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
          }
          // Change button to
        } else {
          //flash red if not both buzzers sending Spare1 which means !=3
          if (Spare1 == 3) { lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0x00B0B0), LV_PART_MAIN | LV_STATE_DEFAULT); }  // Change button to
          else {
            lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
          }  // colour red

          // Increment the setCounter and update the label
          setCounter++;

          // Convert the integer to a string and update the label
          char buffer[10];                                     // Buffer to store the string representation of setCounter
          snprintf(buffer, sizeof(buffer), "%d", setCounter);  // Convert the integer to a string
          lv_label_set_text(ui_TapperOnOffLabel, buffer);      // Set the label text to the updated setCounter value
        }
        // Send Left/Right Flag to BLE if changed
        pChar_Data_Send_To_Phone->setValue(left_right_Flag);
        pChar_Data_Send_To_Phone->notify();
        delay(delayInBetweenBLE);  // Add a small delay between notifications
        previousIntervallMillis = millis();
      }
    }
  } 
}

void ifSettingsChangedDoBLE(){
          // Check if values have changed before sending to BLE
        if (Tapper_Buzz_Flag != prev_Tapper_Buzz_Flag) {
          pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
          pChar_Tapper_Buzz_Flag->notify();
          prev_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
          delay(delayInBetweenBLE);
          //if (Tapper_Buzz_Flag) lv_label_set_text(ui_BuzzOnOffLabel, "");  //"" or arrow on the Buzz click box
          //else lv_label_set_text(ui_BuzzOnOffLabel, "Off");                //"Off" or arrow
        }

        if (Tapper_B_Intensity != prev_Tapper_B_Intensity) {
          pChar_Tapper_B_Intensity->setValue(Tapper_B_Intensity);
          pChar_Tapper_B_Intensity->notify();
          prev_Tapper_B_Intensity = Tapper_B_Intensity;
          delay(delayInBetweenBLE);
        }

        if (Tapper_B_Other != prev_Tapper_B_Other) {
          pChar_Tapper_B_Other->setValue(Tapper_B_Other);
          pChar_Tapper_B_Other->notify();
          prev_Tapper_B_Other = Tapper_B_Other;
          delay(delayInBetweenBLE);
        }

        if (Tapper_Light_Flag != prev_Tapper_Light_Flag) {
          pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
          pChar_Tapper_Light_Flag->notify();
          prev_Tapper_Light_Flag = Tapper_Light_Flag;
          delay(delayInBetweenBLE);
        }

        if (Tapper_L_Intensity != prev_Tapper_L_Intensity) {
          pChar_Tapper_L_Intensity->setValue(Tapper_L_Intensity);
          pChar_Tapper_L_Intensity->notify();
          prev_Tapper_L_Intensity = Tapper_L_Intensity;
          delay(delayInBetweenBLE);
        }

        if (Tapper_L_Other != prev_Tapper_L_Other) {
          pChar_Tapper_L_Other->setValue(Tapper_L_Other);
          pChar_Tapper_L_Other->notify();
          prev_Tapper_L_Other = Tapper_L_Other;
          delay(delayInBetweenBLE);
        }

        if (Tapper_Sound_Flag != prev_Tapper_Sound_Flag) {
          pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
          pChar_Tapper_Sound_Flag->notify();
          prev_Tapper_Sound_Flag = Tapper_Sound_Flag;
          delay(delayInBetweenBLE);
        }

        if (Tapper_S_Intensity != prev_Tapper_S_Intensity) {
          pChar_Tapper_S_Intensity->setValue(Tapper_S_Intensity);
          pChar_Tapper_S_Intensity->notify();
          prev_Tapper_S_Intensity = Tapper_S_Intensity;
          delay(delayInBetweenBLE);
        }

        if (Tapper_S_Other != prev_Tapper_S_Other) {
          pChar_Tapper_S_Other->setValue(Tapper_S_Other);
          pChar_Tapper_S_Other->notify();
          prev_Tapper_S_Other = Tapper_S_Other;
          delay(delayInBetweenBLE);
        }

        if (Tapper_Pressure_Flag != prev_Tapper_Pressure_Flag) {
          pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
          pChar_Tapper_Pressure_Flag->notify();
          prev_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
          delay(delayInBetweenBLE);
        }

        if (Tapper_P_Intensity != prev_Tapper_P_Intensity) {
          pChar_Tapper_P_Intensity->setValue(Tapper_P_Intensity);
          pChar_Tapper_P_Intensity->notify();
          prev_Tapper_P_Intensity = Tapper_P_Intensity;
          delay(delayInBetweenBLE);
        }

        if (Tapper_P_Other != prev_Tapper_P_Other) {
          pChar_Tapper_P_Other->setValue(Tapper_P_Other);
          pChar_Tapper_P_Other->notify();
          prev_Tapper_P_Other = Tapper_P_Other;
          delay(delayInBetweenBLE);
        }

}
void pressureSelected() {  //just got UI settings and pressure tick was detected as selected

  //  prev_Tapper_Buzz_Flag = 0;//Tapper_Buzz_Flag;
  //  prev_Tapper_Light_Flag = 0;//Tapper_Light_Flag;
  //  prev_Tapper_Sound_Flag = 0;//Tapper_Sound_Flag;


  pChar_Tapper_Buzz_Flag->setValue(0);
  pChar_Tapper_Buzz_Flag->notify();
  delay(delayInBetweenBLE);
  pChar_Tapper_Light_Flag->setValue(0);
  pChar_Tapper_Light_Flag->notify();
  delay(delayInBetweenBLE);
  pChar_Tapper_Sound_Flag->setValue(0);
  pChar_Tapper_Sound_Flag->notify();
  delay(delayInBetweenBLE);

  //  if (Tapper_Pressure_Flag != prev_Tapper_Pressure_Flag) {
  pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);  //is 1 to get here
  pChar_Tapper_Pressure_Flag->notify();
  //    prev_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
  delay(delayInBetweenBLE);
  //  }


  //  if (Tapper_P_Intensity != prev_Tapper_P_Intensity) {
  pChar_Tapper_P_Intensity->setValue(Tapper_P_Intensity);
  pChar_Tapper_P_Intensity->notify();
  //    prev_Tapper_P_Intensity = Tapper_P_Intensity;
  delay(delayInBetweenBLE);
  //  }

  //  if (Tapper_P_Other != prev_Tapper_P_Other) {
  pChar_Tapper_P_Other->setValue(Tapper_P_Other);
  pChar_Tapper_P_Other->notify();
  //    prev_Tapper_P_Other = Tapper_P_Other;
  delay(delayInBetweenBLE);
  //  }
}

// Event handler for button press/release/long press
void handleEvent(AceButton * /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      // Turn on the screen when the button is pressed
      Serial.println("[DEBUG] Button pressed! Turning screen ON");
      amoled.setBrightness(255);  // Set to full brightness
      break;
    case AceButton::kEventReleased:
      // Turn off the screen when the button is released
      Serial.println("[DEBUG] Button released! Turning screen OFF");
      if (!readyToSleep) {
        amoled.setBrightness(100);  // Dim the screen if not going to sleep
      }
      break;
    case AceButton::kEventLongPressed:
      Serial.println("[DEBUG] Button long pressed! Waiting for release to enter deep sleep...");
      readyToSleep = true;  // Set flag to wait for button release before sleep
      break;
    default:
      Serial.println("[DEBUG] Unhandled button event");
      break;
  }
}

void retrieveVariablesFromFlash() {
  // Open the Preferences with a namespace
  preferences.begin("TapperPrefs", true);
  if (debug) { Serial.println("Retrieving variables"); }
  if (debug) { Serial.print("Device connected?  "); }
  if (debug) { Serial.println(deviceConnected); }

  // Retrieve the stored variables
  Tapper_OnOff_Flag = preferences.getInt("T_OnOff_Flag", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_OnOff_Flag: "); }
  if (debug) { Serial.println(Tapper_OnOff_Flag); }

  Start_Stop_Flag = preferences.getInt("T_Start_Flag", 1);  // Key shortened
  if (debug) { Serial.print("Retrieved: Start_Stop_Flag: "); }
  if (debug) { Serial.println(Start_Stop_Flag); }

  Tapper_Buzz_Flag = preferences.getInt("T_Buzz_Flag", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_Buzz_Flag: "); }
  if (debug) { Serial.println(Tapper_Buzz_Flag); }

  Tapper_B_Intensity = preferences.getInt("T_B_Intensity", 50);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_B_Intensity: "); }
  if (debug) { Serial.println(Tapper_B_Intensity); }

  Tapper_B_Other = preferences.getInt("T_B_Other", 3);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_B_Other: "); }
  if (debug) { Serial.println(Tapper_B_Other); }

  Tapper_Light_Flag = preferences.getInt("T_Light_Flag", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_Light_Flag: "); }
  if (debug) { Serial.println(Tapper_Light_Flag); }

  Tapper_L_Intensity = preferences.getInt("T_L_Intensity", 100);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_L_Intensity: "); }
  if (debug) { Serial.println(Tapper_L_Intensity); }

  Tapper_L_Other = preferences.getInt("T_L_Other", 3);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_L_Other: "); }
  if (debug) { Serial.println(Tapper_L_Other); }

  Tapper_Sound_Flag = preferences.getInt("T_Sound_Flag", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_Sound_Flag: "); }
  if (debug) { Serial.println(Tapper_Sound_Flag); }

  Tapper_S_Intensity = preferences.getInt("T_S_Intensity", 100);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_S_Intensity: "); }
  if (debug) { Serial.println(Tapper_S_Intensity); }

  Tapper_S_Other = preferences.getInt("T_S_Other", 2);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_S_Other: "); }
  if (debug) { Serial.println(Tapper_S_Other); }

  Tapper_Pressure_Flag = preferences.getInt("T_Press_Flag", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_Pressure_Flag: "); }
  if (debug) { Serial.println(Tapper_Pressure_Flag); }

  Tapper_P_Intensity = preferences.getInt("T_P_Intensity", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_P_Intensity: "); }
  if (debug) { Serial.println(Tapper_P_Intensity); }

  Tapper_P_Other = preferences.getInt("T_P_Other", 0);  // Key shortened
  if (debug) { Serial.print("Retrieved: Tapper_P_Other: "); }
  if (debug) { Serial.println(Tapper_P_Other); }

  set_speed = preferences.getInt("set_speed", 1);  // Already short
  if (debug) { Serial.print("Retrieved: set_speed: "); }
  if (debug) { Serial.println(set_speed); }

  main_counter = preferences.getInt("main_counter", 150);  // Already short
  if (debug) { Serial.print("Retrieved: main_counter: "); }
  if (debug) { Serial.println(main_counter); }

  // Close the Preferences
  preferences.end();
  // Send BLE notifications for characteristics
  if (deviceConnected) {
    if (debug) { Serial.println("Device connected , retrieved values from memory and sending them to BLE"); }
    if (debug) { Serial.print("Retrieved: set_speed is:  "); }
    if (debug) { Serial.println(set_speed); }

    pChar_Tapper_OnOff_Flag->setValue(Tapper_OnOff_Flag);
    pChar_Tapper_OnOff_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
    pChar_Tapper_Buzz_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_B_Intensity->setValue(Tapper_B_Intensity);
    pChar_Tapper_B_Intensity->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_B_Other->setValue(Tapper_B_Other);
    pChar_Tapper_B_Other->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
    pChar_Tapper_Light_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_L_Intensity->setValue(Tapper_L_Intensity);
    pChar_Tapper_L_Intensity->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_L_Other->setValue(Tapper_L_Other);
    pChar_Tapper_L_Other->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
    pChar_Tapper_Sound_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_S_Intensity->setValue(Tapper_S_Intensity);
    pChar_Tapper_S_Intensity->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_S_Other->setValue(Tapper_S_Other);
    pChar_Tapper_S_Other->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
    pChar_Tapper_Pressure_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_P_Intensity->setValue(Tapper_P_Intensity);
    pChar_Tapper_P_Intensity->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_P_Other->setValue(Tapper_P_Other);
    pChar_Tapper_P_Other->notify();
    delay(delayInBetweenBLE);

    pChar_Pixel_Speed->setValue(set_speed);
    pChar_Pixel_Speed->notify();
    delay(delayInBetweenBLE);

    // pChar_main_counter->setValue(main_counter);
    // pChar_main_counter->notify();
    // delay(delayInBetweenBLE);
  }
}

void storeVariablesToFlash() {  //
  // Check if 10 seconds have passed since the last check
  // if (millis() - lastStoreTime < 10000) {
  //   return;  // Do nothing if 10 seconds haven't passed yet
  // }
  // lastStoreTime = millis();  // Update the last store time

  // Open the Preferences
  preferences.begin("TapperPrefs", false);

  // Only store variables if their value has changed
  if (Tapper_OnOff_Flag != last_Tapper_OnOff_Flag) {
    preferences.putInt("T_OnOff_Flag", Tapper_OnOff_Flag);  // Key shortened
    last_Tapper_OnOff_Flag = Tapper_OnOff_Flag;
    if (debug) { Serial.print("Stored: Tapper_OnOff_Flag: "); }
    if (debug) { Serial.println(Tapper_OnOff_Flag); }
  }
  if (Start_Stop_Flag != last_Start_Stop_Flag) {
    preferences.putInt("T_Start_Flag", Start_Stop_Flag);  // Key shortened
    last_Start_Stop_Flag = Start_Stop_Flag;
    if (debug) { Serial.print("Stored: Start_Stop_Flag: "); }
    if (debug) { Serial.println(Start_Stop_Flag); }
  }
  if (Tapper_Buzz_Flag != last_Tapper_Buzz_Flag) {
    preferences.putInt("T_Buzz_Flag", Tapper_Buzz_Flag);  // Key shortened
    last_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
    if (debug) { Serial.print("Stored: Tapper_Buzz_Flag: "); }
    if (debug) { Serial.println(Tapper_Buzz_Flag); }
  }
  if (Tapper_B_Intensity != last_Tapper_B_Intensity) {
    preferences.putInt("T_B_Intensity", Tapper_B_Intensity);  // Key shortened
    last_Tapper_B_Intensity = Tapper_B_Intensity;
    if (debug) { Serial.print("Stored: Tapper_B_Intensity: "); }
    if (debug) { Serial.println(Tapper_B_Intensity); }
  }
  if (Tapper_B_Other != last_Tapper_B_Other) {
    preferences.putInt("T_B_Other", Tapper_B_Other);  // Key shortened
    last_Tapper_B_Other = Tapper_B_Other;
    if (debug) { Serial.print("Stored: Tapper_B_Other: "); }
    if (debug) { Serial.println(Tapper_B_Other); }
  }
  if (Tapper_Light_Flag != last_Tapper_Light_Flag) {
    preferences.putInt("T_Light_Flag", Tapper_Light_Flag);  // Key shortened
    last_Tapper_Light_Flag = Tapper_Light_Flag;
    if (debug) { Serial.print("Stored: Tapper_Light_Flag: "); }
    if (debug) { Serial.println(Tapper_Light_Flag); }
  }
  if (Tapper_L_Intensity != last_Tapper_L_Intensity) {
    preferences.putInt("T_L_Intensity", Tapper_L_Intensity);  // Key shortened
    last_Tapper_L_Intensity = Tapper_L_Intensity;
    if (debug) { Serial.print("Stored: Tapper_L_Intensity: "); }
    if (debug) { Serial.println(Tapper_L_Intensity); }
  }
  if (Tapper_L_Other != last_Tapper_L_Other) {
    preferences.putInt("T_L_Other", Tapper_L_Other);  // Key shortened
    last_Tapper_L_Other = Tapper_L_Other;
    if (debug) { Serial.print("Stored: Tapper_L_Other: "); }
    if (debug) { Serial.println(Tapper_L_Other); }
  }
  if (Tapper_Sound_Flag != last_Tapper_Sound_Flag) {
    preferences.putInt("T_Sound_Flag", Tapper_Sound_Flag);  // Key shortened
    last_Tapper_Sound_Flag = Tapper_Sound_Flag;
    if (debug) { Serial.print("Stored: Tapper_Sound_Flag: "); }
    if (debug) { Serial.println(Tapper_Sound_Flag); }
  }
  if (Tapper_S_Intensity != last_Tapper_S_Intensity) {
    preferences.putInt("T_S_Intensity", Tapper_S_Intensity);  // Key shortened
    last_Tapper_S_Intensity = Tapper_S_Intensity;
    if (debug) { Serial.print("Stored: Tapper_S_Intensity: "); }
    if (debug) { Serial.println(Tapper_S_Intensity); }
  }
  if (Tapper_S_Other != last_Tapper_S_Other) {
    preferences.putInt("T_S_Other", Tapper_S_Other);  // Key shortened
    last_Tapper_S_Other = Tapper_S_Other;
    if (debug) { Serial.print("Stored: Tapper_S_Other: "); }
    if (debug) { Serial.println(Tapper_S_Other); }
  }
  if (Tapper_Pressure_Flag != last_Tapper_Pressure_Flag) {
    preferences.putInt("T_Press_Flag", Tapper_Pressure_Flag);  // Key shortened
    last_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
    if (debug) { Serial.print("Stored: Tapper_Pressure_Flag: "); }
    if (debug) { Serial.println(Tapper_Pressure_Flag); }
  }
  if (Tapper_P_Intensity != last_Tapper_P_Intensity) {
    preferences.putInt("T_P_Intensity", Tapper_P_Intensity);  // Key shortened
    last_Tapper_P_Intensity = Tapper_P_Intensity;
    if (debug) { Serial.print("Stored: Tapper_P_Intensity: "); }
    if (debug) { Serial.println(Tapper_P_Intensity); }
  }
  if (Tapper_P_Other != last_Tapper_P_Other) {
    preferences.putInt("T_P_Other", Tapper_P_Other);  // Key shortened
    last_Tapper_P_Other = Tapper_P_Other;
    if (debug) { Serial.print("Stored: Tapper_P_Other: "); }
    if (debug) { Serial.println(Tapper_P_Other); }
  }
  if (set_speed != last_set_speed) {
    preferences.putInt("set_speed", set_speed);
    last_set_speed = set_speed;
    if (debug) { Serial.print("Stored: set_speed: "); }
    if (debug) { Serial.println(set_speed); }
  }
  if (main_counter != last_main_counter) {
    preferences.putInt("main_counter", main_counter);  // This key is already short
    last_main_counter = main_counter;
    if (debug) { Serial.print("Stored: main_counter: "); }
    if (debug) { Serial.println(main_counter); }
  }

  // Close the Preferences
  preferences.end();
}
//store vars once a day
void daily_storeVariablesToFlash() {
  // Get the current time in microseconds (since boot)
  unsigned long long currentTime = esp_timer_get_time();

  // Check if 24 hours have passed since the last store
  if (currentTime - lastStoreTime >= oneDayInMicroseconds) {
    // Call the store function to save variables to flash
    storeVariablesToFlash();

    // Update the last store time to the current time
    lastDayStoreTime = currentTime;
  }
}
void settings_to_display() {
  set_speed_SpeedSlider();
  set_buzz_check_value();
  set_buzz_intensity_value();
  set_buzz_length_value();
  set_light_check_value();
  set_light_intensity_value();
  set_light_length_value();
  set_sound_check_value();
  set_sound_intensity_value();
  set_sound_length_value();
  set_pressure_check_value();
  set_pressure_intensity_value();
  set_pressure_length_value();
}

void TapperStartStopButtonFunction(lv_event_t *e) {
  // Store previous values of flags and intensities
  static int prev_Tapper_Buzz_Flag_TapperStartStop = Tapper_Buzz_Flag;
  static int prev_Tapper_B_Intensity_TapperStartStop = Tapper_B_Intensity;
  static int prev_Tapper_B_Other_TapperStartStop = Tapper_B_Other;
  static int prev_Tapper_Light_Flag_TapperStartStop = Tapper_Light_Flag;
  static int prev_Tapper_L_Intensity_TapperStartStop = Tapper_L_Intensity;
  static int prev_Tapper_L_Other_TapperStartStop = Tapper_L_Other;
  static int prev_Tapper_Sound_Flag_TapperStartStop = Tapper_Sound_Flag;
  static int prev_Tapper_S_Intensity_TapperStartStop = Tapper_S_Intensity;
  static int prev_Tapper_S_Other_TapperStartStop = Tapper_S_Other;
  static int prev_Tapper_Pressure_Flag_TapperStartStop = Tapper_Pressure_Flag;
  static int prev_Tapper_P_Intensity_TapperStartStop = Tapper_P_Intensity;
  static int prev_Tapper_P_Other_TapperStartStop = Tapper_P_Other;

  // Toggle the Tapper On/Off Flag
  Tapper_OnOff_Flag = !Tapper_OnOff_Flag;
  pChar_Tapper_OnOff_Flag->setValue(Tapper_OnOff_Flag);
  pChar_Tapper_OnOff_Flag->notify();
  delay(delayInBetweenBLE);

  // Change the button's color and label text based on Tapper_OnOff_Flag
  if (Tapper_OnOff_Flag) {
    // When Tapper is ON
    if (debug) { Serial.println("Start Tapper"); }

    lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0x00A5A5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_TapperOnOffLabel, "Tapper On");
    lv_obj_set_style_text_color(ui_TapperOnOffLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    // Restore the previous flag settings if Tapper_OnOff_Flag is set to 1
    Tapper_Buzz_Flag = prev_Tapper_Buzz_Flag_TapperStartStop;
    Tapper_Light_Flag = prev_Tapper_Light_Flag_TapperStartStop;
    Tapper_Sound_Flag = prev_Tapper_Sound_Flag_TapperStartStop;
    Tapper_Pressure_Flag = prev_Tapper_Pressure_Flag_TapperStartStop;

    // Send the restored values to BLE
    pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
    pChar_Tapper_Buzz_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
    pChar_Tapper_Light_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
    pChar_Tapper_Sound_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
    pChar_Tapper_Pressure_Flag->notify();
    delay(delayInBetweenBLE);
    if (debug) { Serial.println("Tapper turned on, flags restored to previous settings."); }
    lv_obj_set_style_text_font(ui_TapperOnOffLabel, &lv_font_montserrat_36, LV_PART_MAIN | LV_STATE_DEFAULT);

  } else {
    // When Tapper is OFF
    setCounter = 0;  // increments when ON
    lv_obj_set_style_text_font(ui_TapperOnOffLabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (debug) { Serial.println("Stop Tapper"); }
    lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0xA500A5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_TapperOnOffLabel, "Turn Tapper ON");
    lv_obj_set_style_text_color(ui_TapperOnOffLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    Tapper_Buzz_Flag = 0;
    Tapper_Light_Flag = 0;  //without this the EMDR_BLE_Central_v2_27 doe not turn the lights off if eith Tapper ON/Off ot startStop_Flag is off
    Tapper_Sound_Flag = 0;
    Tapper_Pressure_Flag = 0;

    // Send these new values to BLE
    pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
    pChar_Tapper_Buzz_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
    pChar_Tapper_Light_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
    pChar_Tapper_Sound_Flag->notify();
    delay(delayInBetweenBLE);

    pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
    pChar_Tapper_Pressure_Flag->notify();
    delay(delayInBetweenBLE);
    if (debug) { Serial.println("Tapper turned off, all flags set to 0."); }
  }
}


// //called when Tapper On/Off button is pressed
// void TapperStartStopButtonFunction(lv_event_t *e) {
//   // Store previous values of flags and intensities
//   static int prev_Tapper_Buzz_Flag = Tapper_Buzz_Flag;
//   static int prev_Tapper_B_Intensity = Tapper_B_Intensity;
//   static int prev_Tapper_B_Other = Tapper_B_Other;
//   static int prev_Tapper_Light_Flag = Tapper_Light_Flag;
//   static int prev_Tapper_L_Intensity = Tapper_L_Intensity;
//   static int prev_Tapper_L_Other = Tapper_L_Other;
//   static int prev_Tapper_Sound_Flag = Tapper_Sound_Flag;
//   static int prev_Tapper_S_Intensity = Tapper_S_Intensity;
//   static int prev_Tapper_S_Other = Tapper_S_Other;
//   static int prev_Tapper_Pressure_Flag = Tapper_Pressure_Flag;
//   static int prev_Tapper_P_Intensity = Tapper_P_Intensity;
//   static int prev_Tapper_P_Other = Tapper_P_Other;


//   // Toggle the Tapper On/Off Flag
//   Tapper_OnOff_Flag = !Tapper_OnOff_Flag;
//   pChar_Tapper_OnOff_Flag->setValue(Tapper_OnOff_Flag);
//   pChar_Tapper_OnOff_Flag->notify();
//   delay(delayInBetweenBLE);

//   // Change the button's color and label text based on Tapper_OnOff_Flag
//   if (Tapper_OnOff_Flag) {
//     // When Tapper is ON
//     if (debug) { Serial.println("Start Tapper"); }

//     lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0x00A5A5), LV_PART_MAIN | LV_STATE_DEFAULT);      // Change button to
//     lv_label_set_text(ui_TapperOnOffLabel, "Tapper On");                                                        // Update label to "Tapper On"
//     lv_obj_set_style_text_color(ui_TapperOnOffLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);  // Set text color to black
//     // Restore the previous flag settings if Tapper_OnOff_Flag is set to 1
//     Tapper_Buzz_Flag = prev_Tapper_Buzz_Flag;
//     Tapper_Light_Flag = prev_Tapper_Light_Flag;
//     Tapper_Sound_Flag = prev_Tapper_Sound_Flag;
//     Tapper_Pressure_Flag = prev_Tapper_Pressure_Flag;

//     // Send the restored values to BLE
//     pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
//     pChar_Tapper_Buzz_Flag->notify();
//     delay(delayInBetweenBLE);
//     //lv_label_set_text(ui_BuzzOnOffLabel,"  ");//"ON" or arrow

//     pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
//     pChar_Tapper_Light_Flag->notify();
//     delay(delayInBetweenBLE);

//     pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
//     pChar_Tapper_Sound_Flag->notify();
//     delay(delayInBetweenBLE);

//     pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
//     pChar_Tapper_Pressure_Flag->notify();
//     delay(delayInBetweenBLE);
//     if (debug) { Serial.println("Tapper turned on, flags restored to previous settings."); }
//     // lv_obj_set_style_text_font(ui_TapperOnOffLabel, &montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);  // Apply Montserrat 20
//     lv_obj_set_style_text_font(ui_TapperOnOffLabel, &lv_font_montserrat_36, LV_PART_MAIN | LV_STATE_DEFAULT);



//   } else {
//     // When Tapper is OFF
//     setCounter = 0;  //increments when ON
//     lv_obj_set_style_text_font(ui_TapperOnOffLabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

//     //lv_obj_set_style_text_font(ui_TapperOnOffLabel, LV_THEME_DEFAULT_FONT_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);  // Reset to default font
//     //lv_obj_set_style_text_opa(ui_TapperOnOffLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
//     //      lv_obj_set_style_text_font(ui_TapperOnOffLabel, &montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);  // Apply Montserrat 20
//     if (debug) { Serial.println("Stop Tapper"); }
//     lv_obj_set_style_bg_color(ui_TapperOnOffBtn, lv_color_hex(0xA500A5), LV_PART_MAIN | LV_STATE_DEFAULT);      // Change button to original color
//     lv_label_set_text(ui_TapperOnOffLabel, "Turn Tapper ON");                                                   // Update label to "Tapper Off"
//     lv_obj_set_style_text_color(ui_TapperOnOffLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);  // Set text color to
//     // If Tapper_OnOff_Flag is 0, reset all flags to 0 and send the updated values to BLE

//     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     //actually it should be enough to set the Tapper_OnOff_Flag or the start stop flag to 0, then the previous valuse are not needed for storage
//     //if the centrals look for those flags
//     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     Tapper_Buzz_Flag = 0;
//     Tapper_Light_Flag = 0;
//     Tapper_Sound_Flag = 0;
//     Tapper_Pressure_Flag = 0;

//     // Send these new values to BLE
//     pChar_Tapper_Buzz_Flag->setValue(Tapper_Buzz_Flag);
//     pChar_Tapper_Buzz_Flag->notify();
//     delay(delayInBetweenBLE);
//     //lv_label_set_text(ui_BuzzOnOffLabel,"Off");

//     pChar_Tapper_Light_Flag->setValue(Tapper_Light_Flag);
//     pChar_Tapper_Light_Flag->notify();
//     delay(delayInBetweenBLE);

//     pChar_Tapper_Sound_Flag->setValue(Tapper_Sound_Flag);
//     pChar_Tapper_Sound_Flag->notify();
//     delay(delayInBetweenBLE);

//     pChar_Tapper_Pressure_Flag->setValue(Tapper_Pressure_Flag);
//     pChar_Tapper_Pressure_Flag->notify();
//     delay(delayInBetweenBLE);
//     if (debug) { Serial.println("Tapper turned off, all flags set to 0."); }

//     // Add any necessary cleanup or stop actions here
//   }
// }

void updateBatteryLabels() {
  getvbat_per();
  if (debugP) { Serial.println("Bat values to display"); }

  // Display battery percentage for the center label (ui_BatLabel)
  char battVoltageStr[16];                                             // Buffer to store the voltage as a string
  snprintf(battVoltageStr, sizeof(battVoltageStr), "%d%%", vbat_per);  // amoled.getBattVoltage());
  lv_label_set_text(ui_BatLabel, battVoltageStr);                      // Set the label text to the voltage value
  if (debugP) { Serial.println(amoled.getBattVoltage()); }
  if (debugP) { Serial.println(vbat_per); }
  if (debugP) { Serial.print("core temp: "); }
  if (debugP) { Serial.println(amoled.readCoreTemp()); }

  // Retrieve and display batVLeft (BatLeft)
  char batLeftStr[16];
  snprintf(batLeftStr, sizeof(batLeftStr), "%d%%", vbat1_perCharge);  // Convert batVLeft to string with percentage
  lv_label_set_text(ui_BatLeftLabel, batLeftStr);                     // Display the value in the label for BatLeft

  // Retrieve and display batVRight (BatRight)
  char batRightStr[16];
  snprintf(batRightStr, sizeof(batRightStr), "%d%%", vbat2_perCharge);  // Convert batVRight to string with percentage
  lv_label_set_text(ui_BatRightLabel, batRightStr);                     // Display the value in the label for BatRight

  // Update the battery bars (Left, Center, Right) based on vbat_per values
  lv_bar_set_value(ui_BatBarLeft, vbat1_perCharge, LV_ANIM_OFF);   // Set Left bar value
  lv_bar_set_value(ui_BatBarCenter, vbat_per, LV_ANIM_OFF);        // Set Center bar value
  lv_bar_set_value(ui_BatBarRight, vbat2_perCharge, LV_ANIM_OFF);  // Set Right bar value

  // Debug: Print Spare1 value
  if (debug) { Serial.print("Spare1: "); }
  if (debug) { Serial.println(Spare1); }

  // Handle the right side (bit 0 of Spare1)
  if (Spare1 & 0x01) {
    // Bit 0 is 1 -> Set right label to green and keep the percentage text
    lv_obj_set_style_text_color(ui_BatRightLabel, lv_color_hex(0x00FF00), 0);                                 // Green
    lv_obj_set_style_bg_color(ui_BatBarRight, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);  // Green for the filled part
    lv_obj_set_style_bg_color(ui_BatBarRight, lv_color_hex(0xF84138), LV_PART_MAIN | LV_STATE_DEFAULT);       // Red for the background
  } else {
    // Bit 0 is 0 -> Set right label to red and grey out the entire right bar
    lv_obj_set_style_text_color(ui_BatRightLabel, lv_color_hex(0xFF0000), 0);                                 // Red
    lv_obj_set_style_bg_color(ui_BatBarRight, lv_color_hex(0x808080), LV_PART_INDICATOR | LV_STATE_DEFAULT);  // Indicator grey
    lv_obj_set_style_bg_color(ui_BatBarRight, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);       // Background grey
  }

  // Handle the left side (bit 1 of Spare1)
  if (Spare1 & 0x02) {
    // Bit 1 is 1 -> Set left label to green and keep the percentage text
    lv_obj_set_style_text_color(ui_BatLeftLabel, lv_color_hex(0x00FF00), 0);                                 // Green
    lv_obj_set_style_bg_color(ui_BatBarLeft, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);  // Green for the filled part
    lv_obj_set_style_bg_color(ui_BatBarLeft, lv_color_hex(0xF84138), LV_PART_MAIN | LV_STATE_DEFAULT);       // Red for the background
  } else {
    // Bit 1 is 0 -> Set left label to red and grey out the entire left bar
    lv_obj_set_style_text_color(ui_BatLeftLabel, lv_color_hex(0xFF0000), 0);                                 // Red
    lv_obj_set_style_bg_color(ui_BatBarLeft, lv_color_hex(0x808080), LV_PART_INDICATOR | LV_STATE_DEFAULT);  // Indicator grey
    lv_obj_set_style_bg_color(ui_BatBarLeft, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);       // Background grey
  }

  // Ensure the center bar is updating
  lv_bar_set_value(ui_BatBarCenter, vbat_per, LV_ANIM_OFF);  // Re-set center bar value
}


//-------------------------------------------------------------------------------
void centralWatchdog() {  //Spare1_was_written_Flag checks for change
                          // Check if the characteristic value has changed

  //Serial.println(Spare1);
  //delay(100);
  // Check if it's time to check Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_RECEIVE) {
    // Check for vBat1_Heartbeat_written_Flag (readSELECT_PIN_L_R = HIGH BUZZER)
    if (vBat1_Heartbeat_written_Flag != 1) {
      if (debug) { Serial.println("Alarm: vBat1_Heartbeat_written_Flag is not 1!"); }
      Spare1 &= 1;  //leave bit0 alone but turn off bit1, Spare1 gets updated in central if sending again
      pChar_Spare1->setValue(Spare1);
      pChar_Spare1->notify();

      // Additional actions for vBat1_Heartbeat_written_Flag != 1
    }
    vBat1_Heartbeat_written_Flag = 0;  // Resetting to 0 if alarm condition is met

    // Check for vBat2_Heartbeat_written_Flag. (readSELECT_PIN_L_R = LOW BUZZER)
    if (vBat2_Heartbeat_written_Flag != 1) {
      if (debug) { Serial.println("Alarm: vBat2_Heartbeat_written_Flag is not 1!"); }
      Spare1 &= 2;  //leave bit1 alone but turn off bit0, Spare1 gets updated in central if sending again
      pChar_Spare1->setValue(Spare1);
      pChar_Spare1->notify();
      // Additional actions for vBat2_Heartbeat_written_Flag != 1
    }
    vBat2_Heartbeat_written_Flag = 0;  // Resetting to 0 if alarm condition is met

    // Reset the timer
    lastHeartbeat = millis();
  }
}  //centralWatchdog()

void printMemoryStats() {
  Serial.print("Free heap: ");
  Serial.print(esp_get_free_heap_size());

  Serial.print("   Largest free block: ");
  Serial.print(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  Serial.print("   Task stack high watermark: ");
  Serial.println(uxTaskGetStackHighWaterMark(NULL));  // For current task

  // #ifdef CONFIG_SPIRAM_SUPPORT
  // Serial.print("Free PSRAM: ");
  // Serial.println(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  // #endif

  Serial.print("Free PSRAM: ");
  Serial.println(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  Serial.print("Largest PSRAM block: ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}
/*8 bit to RGB

Here's how you can convert an 8-bit value into an RGB value that covers the color wheel:

Map the 8-bit value to the HSV color wheel:

Hue (H): Map the 8-bit value (0-255) directly to the hue value (0-255) of the HSV color model.
Saturation (S) and Value (V): Set both to 255 to ensure full saturation and brightness.
Convert the HSV value to RGB: You can use a function to convert HSV to RGB. Here's a simple example in C++:

cpp
Copy code
// Function to convert HSV to RGB
uint32_t hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t region, remainder, p, q, t;
    uint32_t r, g, b;

    if (s == 0) {
        r = g = b = v;
        return ((r << 16) | (g << 8) | b);  // Return RGB as 24-bit value
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }

    return ((r << 16) | (g << 8) | b);  // Return RGB as 24-bit value
}

// Example usage
uint8_t colorIndex = 128; // Any 8-bit value from 0-255
uint32_t color = hsvToRgb(colorIndex, 255, 255);
This function takes an 8-bit hue value (0-255) and converts it to an RGB value that covers the full spectrum of the color wheel. You can then use the resulting RGB value wherever you need a color.

COLOR WHEEL to 8BIT:
To convert the color wheel output from a 24-bit RGB value to an 8-bit value that represents the full range of colors, we need to map the 24-bit colors into an 8-bit space. Since an 8-bit value only has 256 possible values, we cannot directly use the full 24-bit RGB values, which have over 16 million combinations. However, we can create a mapping that represents a simplified version of the color spectrum using the 8-bit format.

Here's how we can achieve this:

Segment the Color Wheel:

The color wheel can be divided into 6 main segments: Red, Yellow, Green, Cyan, Blue, and Magenta.
We will assign a specific range of values in the 8-bit space to each of these segments.
Map the Segments to 8-Bit Values:

Each segment represents a part of the HSV color spectrum, primarily using the hue (H).
We convert these segments into 8-bit values and map the RGB values accordingly.
Here’s a C++ function that performs this conversion:

cpp
Copy code
// Function to map RGB to an 8-bit color index
uint8_t rgbTo8Bit(uint8_t r, uint8_t g, uint8_t b) {
    // Normalize the RGB values to find the maximum and minimum values
    uint8_t maxVal = max(max(r, g), b);
    uint8_t minVal = min(min(r, g), b);

    // Calculate the hue based on the RGB values
    uint8_t hue;
    if (maxVal == minVal) {
        hue = 0; // No color (gray scale)
    } else if (maxVal == r) {
        hue = 43 * (g - b) / (maxVal - minVal);
    } else if (maxVal == g) {
        hue = 85 + 43 * (b - r) / (maxVal - minVal);
    } else {
        hue = 171 + 43 * (r - g) / (maxVal - minVal);
    }

    // Adjust hue to fit the 8-bit value range (0-255)
    if (hue < 0) hue += 256;

    // Map the hue value to 8-bit
    return hue;
}

// Example usage
uint32_t color = 0xFF0000; // Red
uint8_t r = (color >> 16) & 0xFF;
uint8_t g = (color >> 8) & 0xFF;
uint8_t b = color & 0xFF;
uint8_t colorIndex = rgbTo8Bit(r, g, b);
Explanation:
Normalization:

We normalize the RGB values by finding the maximum and minimum of the three values. This helps in calculating the hue.
Hue Calculation:

The hue value is calculated based on which of the RGB components is the largest. The formula varies depending on the dominant color channel.
Hue Adjustment:

We adjust the hue value to ensure it fits into the 0-255 range for 8-bit representation.
How It Works:
The hue (0-255) now covers the entire spectrum of the color wheel:
0 represents red, 85 represents green, 170 represents blue, and values in between cover the transitions (e.g., cyan, magenta, yellow).
By using this method, you can convert the 24-bit RGB values from the color wheel into an 8-bit representation that cycles through the full range of colors.

*/

/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Explanation
RGB to 8-bit Hue:

The function RGBto8Bit calculates the hue from the RGB values.
Hue is then scaled to an 8-bit value (0–255).
8-bit Hue to RGB:

The function hueToRGB converts the 8-bit hue value back to RGB using fixed saturation and value (HSV model).
The RGB values are output as 8-bit values.
This approach allows you to compress RGB values into an 8-bit hue representation and reconstruct the RGB values from this compressed form.

// Function to convert RGB to an 8-bit hue value
uint8_t RGBto8Bit(uint8_t red, uint8_t green, uint8_t blue) {
    float r = red / 255.0;
    float g = green / 255.0;
    float b = blue / 255.0;

    float max = fmaxf(r, fmaxf(g, b));
    float min = fminf(r, fminf(g, b));
    float hue;

    if (max == min) {
        hue = 0;
    } else if (max == r) {
        hue = (60 * ((g - b) / (max - min)) + 360);
    } else if (max == g) {
        hue = (60 * ((b - r) / (max - min)) + 120);
    } else {
        hue = (60 * ((r - g) / (max - min)) + 240);
    }

    hue = fmodf(hue, 360);
    return (uint8_t)(hue / 360.0 * 255);
}

// Function to convert an 8-bit hue value back to RGB
void hueToRGB(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b) {
    float h = hue / 255.0 * 360;
    float s = 1.0; // Saturation fixed at 100%
    float v = 1.0; // Value fixed at 100%

    int i = (int)(h / 60) % 6;
    float f = (h / 60) - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i) {
        case 0:
            *r = (uint8_t)(v * 255);
            *g = (uint8_t)(t * 255);
            *b = (uint8_t)(p * 255);
            break;
        case 1:
            *r = (uint8_t)(q * 255);
            *g = (uint8_t)(v * 255);
            *b = (uint8_t)(p * 255);
            break;
        case 2:
            *r = (uint8_t)(p * 255);
            *g = (uint8_t)(v * 255);
            *b = (uint8_t)(t * 255);
            break;
        case 3:
            *r = (uint8_t)(p * 255);
            *g = (uint8_t)(q * 255);
            *b = (uint8_t)(v * 255);
            break;
        case 4:
            *r = (uint8_t)(t * 255);
            *g = (uint8_t)(p * 255);
            *b = (uint8_t)(v * 255);
            break;
        case 5:
            *r = (uint8_t)(v * 255);
            *g = (uint8_t)(p * 255);
            *b = (uint8_t)(q * 255);
            break;
    }
}

// Example usage
void example() {
    uint8_t red = 255;
    uint8_t green = 0;
    uint8_t blue = 0;

    // Convert RGB to 8-bit hue value
    uint8_t hue8bit = RGBto8Bit(red, green, blue);
    Serial.print("8-bit Hue: ");
    Serial.println(hue8bit);

    // Convert back to RGB
    uint8_t r, g, b;
    hueToRGB(hue8bit, &r, &g, &b);

    Serial.print("Converted RGB - R: ");
    Serial.print(r);
    Serial.print(", G: ");
    Serial.print(g);
    Serial.print(", B: ");
    Serial.println(b);
}

*/