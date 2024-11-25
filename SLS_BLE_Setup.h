extern bool deviceConnected;
#ifndef BLE_Setup_H
#define BLE_Setup_H

#include <NimBLEDevice.h>
#include "SLS_BLE_Defs.h"
//#include "globals.h"



// BLE variables
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint32_t value1;
extern uint32_t testValue1;
extern unsigned long previousDevConnectedMillis;
extern const unsigned long notificationInterval;
extern unsigned long previousUpdateBuzzerMillis;
extern const unsigned long notificationUpdateBuzzerInterval;

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

// extern int tapperBuzzFlagState;
// extern int start_Stop_Flag;
// extern int tapper_OnOff_Flag;
// extern int tapper_B_Intensity;

// General variables
extern int on_off;
extern int main_counter;
extern unsigned long previousMillis;
extern unsigned long previousIntervallMillis;

extern long on_off_Intervall;
extern const long interval;
extern const int ledPin;
extern const int buttonPin;

extern int Spare1;//used for HEARTBEAT (also when Bat voltage gets sent), was 100 for one and 1 for the other buzzer
extern int Spare2;
extern int Spare3;
extern int Spare4;
extern int Spare5;
extern int vbat1_perCharge;
extern int vbat2_perCharge;
extern int Spare1_was_written_Flag;
extern int vBat1_was_written_Flag;
extern int vBat1_Heartbeat_written_Flag;
extern int vBat2_was_written_Flag;
extern int vBat2_Heartbeat_written_Flag;

// BLE related
NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr;


NimBLECharacteristic* pChar_Pixel_Speed = nullptr;
NimBLECharacteristic* pChar_On_Off = nullptr;
NimBLECharacteristic* pChar_Start_Stop_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_OnOff_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_Buzz_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_B_Intensity = nullptr;
NimBLECharacteristic* pChar_Tapper_B_Other = nullptr;
NimBLECharacteristic* pChar_Tapper_Light_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_L_Intensity = nullptr;
NimBLECharacteristic* pChar_Tapper_L_Other = nullptr;
NimBLECharacteristic* pChar_Tapper_Sound_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_S_Intensity = nullptr;
NimBLECharacteristic* pChar_Tapper_S_Other = nullptr;
NimBLECharacteristic* pChar_Tapper_Pressure_Flag = nullptr;
NimBLECharacteristic* pChar_Tapper_P_Intensity = nullptr;
NimBLECharacteristic* pChar_Tapper_P_Other = nullptr;
NimBLECharacteristic* pChar_StringLightFunction = nullptr;
NimBLECharacteristic* pChar_Data_Send_To_Phone = nullptr;
NimBLECharacteristic* pChar_Spare1 = nullptr;
NimBLECharacteristic* pChar_Spare2 = nullptr;
NimBLECharacteristic* pChar_Spare3 = nullptr;
NimBLECharacteristic* pChar_Spare4 = nullptr;
NimBLECharacteristic* pChar_Spare5 = nullptr;
NimBLECharacteristic* pChar_vbat1_perCharge = nullptr;
NimBLECharacteristic* pChar_vbat2_perCharge = nullptr;

class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
    pServer->startAdvertising();
  }
};
//--------------------------------------------------------------------------------------------- 41
class CharacteristicCallBack_pChar_Spare1 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string charValue = pChar->getValue();
    String charValueString = String(charValue.c_str());
    char charValueChar = charValueString.charAt(0);  // Get the first character
    Spare1 = (int)(unsigned char)charValueChar;      // Convert character to byte value
    Serial.println("41 CharacteristicCallBack_pChar_Spare1 (heartbeat from a buzzer and from App): " + String(Spare1));
    Spare1_was_written_Flag = 1;  //so it can be notified elsewhere or it may be looping in on itself, so it gets passed from Central to Peripheral and then Peripheral to APP and central
  }
};  //CharacteristicCallBack_pChar_Spare1

// BUZZER with readSELECT_PIN_L_R high =vbat1 =Spare1 = 0001
class CharacteristicCallBack_pChar_vbat1_perCharge : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string charValue = pChar->getValue();
    if (!charValue.empty()) {
      char charValueChar = charValue[0];                                              // Get the first character
      vbat1_perCharge = static_cast<int>(static_cast<unsigned char>(charValueChar));  // Convert character to byte value
      Serial.println("46 CharacteristicCallBack_pChar_vbat1_perCharge: " + String(vbat1_perCharge));
      vBat1_was_written_Flag = 1;
      vBat1_Heartbeat_written_Flag = 1;
    } else {
      Serial.println("Received empty value for vbat1_perCharge");
    }
  }
};  //CharacteristicCallBack_pChar_vbat1_perCharge()

//--------------------------------------------------------------------------------------------- 47
//  BUZZER with readSELECT_PIN_L_R low =vbat2 =Spare1 = 0010
class CharacteristicCallBack_pChar_vbat2_perCharge : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string charValue = pChar->getValue();
    if (!charValue.empty()) {
      char charValueChar = charValue[0];                                              // Get the first character
      vbat2_perCharge = static_cast<int>(static_cast<unsigned char>(charValueChar));  // Convert character to byte value
      Serial.println("47 CharacteristicCallBack_pChar_vbat2_perCharge: " + String(vbat2_perCharge));
      vBat2_was_written_Flag = 1;
      vBat2_Heartbeat_written_Flag = 1;
    } else {
      Serial.println("Received empty value for vbat2_perCharge");
    }
  }
};  //CharacteristicCallBack_pChar_vbat2_perCharge()

void initBLE() {
  NimBLEDevice::init("ESP32");
  NimBLEDevice::setMTU(256);  // Set MTU

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  // Create Characteristics
  pChar_Pixel_Speed = pService->createCharacteristic(
    CHAR_PIXEL_SPEED_V00_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_On_Off = pService->createCharacteristic(
    CHAR_ON_OFF_V01_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Start_Stop_Flag = pService->createCharacteristic(
    CHAR_START_STOP_FLAG_V26_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_OnOff_Flag = pService->createCharacteristic(
    CHAR_TAPPER_ON_OFF_FLAG_VT01_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_Buzz_Flag = pService->createCharacteristic(
    CHAR_TAPPER_BUZZ_FLAG_VT02_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_B_Intensity = pService->createCharacteristic(
    CHAR_TAPPER_BUZZ_INTENSITY_VT12_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_B_Other = pService->createCharacteristic(
    CHAR_TAPPER_BUZZ_OTHER_VT22_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_Light_Flag = pService->createCharacteristic(
    CHAR_TAPPER_LIGHT_FLAG_VT03_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_L_Intensity = pService->createCharacteristic(
    CHAR_TAPPER_LIGHT_INTENSITY_V13_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_L_Other = pService->createCharacteristic(
    CHAR_TAPPER_LIGHT_OTHER_VT23_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_Sound_Flag = pService->createCharacteristic(
    CHAR_TAPPER_SOUND_FLAG_VT04_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_S_Intensity = pService->createCharacteristic(
    CHAR_TAPPER_SOUND_INTENSITY_VT14_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_S_Other = pService->createCharacteristic(
    CHAR_TAPPER_SOUND_OTHER_VT24_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_Pressure_Flag = pService->createCharacteristic(
    CHAR_TAPPER_PRESSURE_FLAG_VT05_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_P_Intensity = pService->createCharacteristic(
    CHAR_Tapper_PRESSURE_INTENSITY_VT15_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Tapper_P_Other = pService->createCharacteristic(
    CHAR_TAPPER_PRESSURE_OTHER_VT25_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_StringLightFunction = pService->createCharacteristic(
    CHAR_STING_LIGHT_FUNCTION_VL01_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Data_Send_To_Phone = pService->createCharacteristic(
    CHAR_SEND_TO_PHONE,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Spare1 = pService->createCharacteristic(
    CHAR_SPARE1_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Spare2 = pService->createCharacteristic(
    CHAR_SPARE2_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Spare3 = pService->createCharacteristic(
    CHAR_SPARE3_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Spare4 = pService->createCharacteristic(
    CHAR_SPARE4_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_Spare5 = pService->createCharacteristic(
    CHAR_SPARE5_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );


  pChar_vbat1_perCharge = pService->createCharacteristic(
    Char_vbat1_perCharge_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );

  pChar_vbat2_perCharge = pService->createCharacteristic(
    Char_vbat2_perCharge_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
// Set the callback functions for the characteristics
  pChar_Spare1->setCallbacks(new CharacteristicCallBack_pChar_Spare1());                            //41


  pChar_vbat1_perCharge->setCallbacks(new CharacteristicCallBack_pChar_vbat1_perCharge());  // Assign callback for vbat1_perCharge
  pChar_vbat2_perCharge->setCallbacks(new CharacteristicCallBack_pChar_vbat2_perCharge());  // Assign callback for vbat2_perCharge

  // Start the service
  pService->start();

  // Start advertising
  NimBLEDevice::startAdvertising();
  Serial.println("Bluetooth® device active, waiting for connections...");
}


void initBuzzers() {
  pChar_Start_Stop_Flag->setValue(Start_Stop_Flag);
  pChar_Start_Stop_Flag->notify();
  delay(delayInBetweenBLE);

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

  // Notify other characteristics as needed
}



#endif // BLE_Setup_H
