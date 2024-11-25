/*
  Battery Monitor with scaled percentage for LIPO Battery

   The A0 pin is used to calculate the battery level.
   the reading interval is the HEARTBEAT
*/
//#define Time_Between_BLE_Bat_Update 10000
//from EMDR_BLE_Central_v2_27.ino
extern float LIPOChart_Used_SOC_to_Percent[];
extern byte old_vbat_per;  // last battery level reading from analog input
extern LilyGo_Class amoled;
#define SOC_STEPS_ARRAY 21


// 3.0V ADC range and 12-bit ADC resolution = 3000mV/4095
//#define VBAT_MV_PER_LSB   (0.7326007F)

//2.4V ADC range and 12-bit ADC resolution = 2400mV/4095.   Could not find the setting for 1.8V = 3* 0.6V
#define VBAT_MV_PER_LSB (0.58594F)
// 1M + 510K divider on VBAT = (0.51M / (0.510M + 1M)) this is internal to the XIAU Seeed BLE if P0_14 is low to enable
#define VBAT_DIVIDER (0.337748F)
// Compensation factor for the VBAT divider
#define VBAT_DIVIDER_COMP (2.960787F)
//#define VBAT_CONVERSION 1.778639//((510e3 + 1000e3) / 510e3) * 2460 /4095)      * vbat_raw //2.46 is a measured compensation to make reading accurate for a particular chip
#define VBAT_CONVERSION 1   //for xiaoBLE: 1.73483456  //((510e3 + 1000e3) / 510e3) * 2400 /4095)      * vbat_raw
int vbat_raw;
extern uint8_t vbat_per;
float vbat_mv;
float vbat_V;
//cyclic buffer
// extern const int bufferSize;
// extern int buffer[];
// extern int bufferIndex;
// extern int totalReadings;
// extern int sum;
// extern float vbat_average;
const int bufferSize = 10;
int buffer[bufferSize]={0,0,0,0,0,0,0,0,0,0};
int bufferIndex = 0;
int totalReadings = 0;
int sum = 0;
float vbat_average = 0;

void getvbat_per();
uint8_t mvToPercent(float);
void addToBuffer(int);
float calculateAverage();


void getvbat_per(void) {//execute every heartbeat
  //digitalWrite(P0_14, LOW);  //VBat voltage divider to ground when LOW to be able to measure, set to HIGH saves 2.5mueA

  vbat_raw = amoled.getBattVoltage(); //analogRead(PIN_VBAT);  //(VBAT_PIN);

  // Add new value to the cyclic buffer
  //This setup ensures that, initially, the average is based on however many samples have been taken (up to 10), and after the buffer is filled, it always averages the last 10 readings.
  addToBuffer(vbat_raw);
  // Calculate average
  vbat_average = calculateAverage();

  vbat_mv = VBAT_CONVERSION * vbat_raw;        //(float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  int vbat_mv_average = VBAT_CONVERSION * vbat_average;        //(float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  vbat_V = VBAT_CONVERSION * vbat_average / 1000;  //(float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;


  // vbat_mv = VBAT_CONVERSION * vbat_raw;        //(float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  // vbat_V = VBAT_CONVERSION * vbat_raw / 1000;  //(float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  vbat_per = mvToPercent(vbat_V);              //((510e3 + 1000e3) / 510e3) * 2.46 * vbat_raw / 4095 );//2.1148);//2.1148 is to make 3000 fullscale with the voltage divider being different and the ref voltage being 2.4V

  Serial.print("ADC = ");
  Serial.print(vbat_raw);// * VBAT_MV_PER_LSB);
  Serial.print(" mV (");
  // Serial.print(vbat_raw);
  // Serial.print(") ");

  // Serial.print("LIPO = ");
  // Serial.print(vbat_mv);
  // Serial.print(" mV ");
  // Serial.print(vbat_mv_average);
  // Serial.print(" mVaverage ");
  
  // Serial.print(vbat_V,3);
  // Serial.print(" V (");
  Serial.print(vbat_per);
  Serial.println("%)");
  //Serial.print(Vbat);
  //Serial.println("V");
  //vbat_perChar.writeValue(vbat_per);  // and update the battery level characteristic
  old_vbat_per = vbat_per;  // save the level for next comparison

  //Serial.println("vbat_per was written to BLE");
  //}

  //delay(4000);
}

void addToBuffer(int value) {
    // Subtract the oldest value from sum
    sum -= buffer[bufferIndex];

    // Add new value to buffer and sum
    buffer[bufferIndex] = value;
    sum += value;

    // Update bufferIndex, totalReadings
    bufferIndex = (bufferIndex + 1) % bufferSize;//cycle through the buffer
    if (totalReadings < bufferSize) {
        totalReadings++;
    }
}

float calculateAverage() {
    if (totalReadings == 0) return 0; // Avoid division by zero
    return (float)sum / totalReadings;
}


/*
It iterates through the LIPOChart_Used_SOC_to_Percent array.
It checks if the voltage is between two array values.
If so, it performs linear interpolation to find the corresponding battery percentage.
The interpolation formula calculates the slope between the two adjacent array points and then uses it to find the percentage value for the given voltage.

*/


uint8_t mvToPercent(float voltage) {
    //float LIPOChart_Used_SOC_to_Percent[SOC_STEPS_ARRAY] = {4.04, 4.00, 3.97, 3.93, 3.90, 3.86, 3.81, 3.76, 3.71, 3.67, 3.64, 3.62, 3.61, 3.58, 3.56, 3.51, 3.46, 3.40, 3.27, 3.1, 3.0};
    uint8_t percentages[SOC_STEPS_ARRAY] = {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};

    if (voltage >= LIPOChart_Used_SOC_to_Percent[0]) {
        return 100;
    }

    for (int i = 0; i < SOC_STEPS_ARRAY - 1; i++) {
        if (voltage >= LIPOChart_Used_SOC_to_Percent[i + 1]) {
            float slope = (percentages[i] - percentages[i + 1]) / (LIPOChart_Used_SOC_to_Percent[i] - LIPOChart_Used_SOC_to_Percent[i + 1]);
            return percentages[i] + slope * (voltage - LIPOChart_Used_SOC_to_Percent[i]);
        }
    }

    return 0; // Return 0 if voltage is below the lowest threshold
}



/*
#include <Arduino.h>
#include <bluefruit.h>
#include <xiaobattery.h>

Xiao battery;
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
// Serial.println(battery.IsChargingBattery());
Serial.println(battery.GetBatteryVoltage());
}
```
*/
/*
https://usermanual.wiki/Pdf/bluefruitnrf52featherlearningguide.291033005/view
https://hutscape.com/tutorials/measure-battery-nrf52
https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Hardware/adc_vbat/adc_vbat.ino
*/
