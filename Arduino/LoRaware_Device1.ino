/**
 *  LoRaware Rev.6
 *      Author: David Rodriguez
 *      September 2019
 *  
 *  For US915MHz, Requires
 *  Beelan LoRaWAN Library: https://github.com/BeelanMX/Beelan-LoRaWAN
 *  
 *  Other Required Libraries
 *  ESP32_AnalogWrite: https://github.com/ERROPiX/ESP32_AnalogWrite
 *  Sparkfun_MAX3010x_Sensor_Library: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
 *  Adafruit_BME280_Library: https://github.com/adafruit/Adafruit_BME280_Library
 *  
 *  LoRaware Device #1 
 */
#include <lorawan.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <analogWrite.h>
Adafruit_BME280 bme;
MAX30105 particleSensor;
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 34; //Must be on PWM pin
byte readLED = 2; //Blinks with each data read
byte battPin = 0; //Battery Voltage Pin
byte button = 33; //Push Button Pin
//ABP Credentials - FOR Device: LoRaware_1
const char *devAddr = "00000000"; 
const char *nwkSKey = "00000000000000000000000000000000";
const char *appSKey = "00000000000000000000000000000000";
#define MAX_BRIGHTNESS 255
#define SS 18
#define MISO 19
#define MOSI 27
#define SCK 5
unsigned long previousMillis = 0;  // will store last time message sent
int countHR = 0; // MAX30102 HR Error Counter
int countO2 = 0; // MAX30102 HR Error Counter
int wearState = 0; // Default Wear State Enabled
int dbgState = 0; // Default Debugging Disabled
char myStr[50];
char myStrCB[26];
char myStrVB[26];
char myStrT[26];
char myStrH[26];
char myStrHR[26];
char myStrO2[26];
char payload[255];
char emergencyPayload[255];
char outStr[255];
byte recvStatus = 0;
bool eState = false;
const sRFM_pins RFM_pins = {
  .CS = 18,
  .RST = 14,
  .DIO0 = 26,
  .DIO1 = 25,
  .DIO2 = 32,
  .DIO5 = 35,
};

void LEDbreathe() {
  // Breathes Green LED with Sinusoidal PWM Function
  for (int i = 0; i < 360; i++) {
    analogWrite(readLED, (sin(i * 0.0174533) + 1) * (127/16));
    delay(6);
} // End of void LEDbreathe()

void LoRaware_norm() {
  // LoRaware Normal Operation
  LEDbreathe();
  particleSensor.wakeUp();
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25 sps
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    if (eState == true) {
      break;
    }
    else {
     redBuffer[i] = particleSensor.getRed();
     irBuffer[i] = particleSensor.getIR();
     particleSensor.nextSample();
    }    
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  float measuredvbat = analogRead(battPin);
  float battV;
  battV = (((measuredvbat)*(2.1) / (4096)))*2;
  dtostrf(battV,6, 3, myStrVB);
  dtostrf((bme.readTemperature()*(1.8) + 32),6, 3, myStrT);
  dtostrf(bme.readHumidity(),6, 3, myStrH);
  dtostrf(heartRate,6, 2, myStrHR);
  dtostrf(spo2,6, 2, myStrO2);  
  String str;
  str = (String(myStrVB) +","+ String(myStrT) +","+ String(myStrH) +","+ String(myStrHR) +","+ String(myStrO2));
  unsigned int str_len = 37;
  str.toCharArray(payload,str_len); 
  Serial.println(payload);
  // LoRaWAN Data Send Uplink to Gateway
  lora.sendUplink(payload, strlen(payload), 0); // 0 = LoRaWAN UNCONFIRMED Message Type
  analogWrite(readLED,0);
  particleSensor.shutDown();
  delay(5000);
  recvStatus = lora.readData(outStr);
  if(recvStatus) {
  }
  // Check Lora RX
  lora.update();
  particleSensor.wakeUp();
} // End of void LoRaware_norm()

void emergencyPush() {
  analogWrite(readLED,250);
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // Disregard Interrupts faster than 200 ms
  if (interrupt_time - last_interrupt_time > 200){
    eState = !eState;
  }
  last_interrupt_time = interrupt_time;
} // End of void emergencyPush()

void sendEmergencyLoRa() {
  analogWrite(readLED,250);
  // LoRaWAN Emergency Message Payload
  particleSensor.shutDown();
  analogWrite(readLED,250);
  delay(300);
  String emergencyStr;
  emergencyStr = (String(0) +","+ String(0) +","+ String(0) +","+ String(0) +","+ String(0));
  Serial.println("Sending Emergency msg");
  unsigned int str_len = 37;
  emergencyStr.toCharArray(emergencyPayload,str_len); 
  Serial.println(emergencyPayload);
  delay(100);    
  // LoRaWAN Data Send Uplink to Gateway
  lora.sendUplink(emergencyPayload, strlen(emergencyPayload), 0); // 0 = LoRaWAN UNCONFIRMED Message Type
  // Check Lora RX
  lora.update();
  eState = !eState;
  analogWrite(readLED,0);    
} // End of void sendEmergencyLoRa()

void setup() {
  Serial.begin(115200);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  pinMode(battPin, INPUT);
  pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), emergencyPush, HIGH); // Button Pin as Interrupt Pin
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  SPI.begin(SCK,MISO,MOSI,SS);
  if(!lora.init()){
    Serial.println("RFM95 not detected");
    delay(5000);
    yield();
    return;
  }
  // LoRaWAN Class
  lora.setDeviceClass(CLASS_A);
  // Data Rate
  lora.setDataRate(SF8BW125);
  // Channel Set to Random
  lora.setChannel(MULTI);
  // ABP Key and DevAddress
  lora.setNwkSKey(nwkSKey);
  lora.setAppSKey(appSKey);
  lora.setDevAddr(devAddr);
  byte ledBrightness = 45; //Options: 0=Off to 255=50mA
  byte sampleAverage = 16; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 118; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384 
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // MAX30102 Sensor Configuration
} // End of void setup

void loop() {
  start:
  if (eState == true) {
    sendEmergencyLoRa();
    goto start;
  }
  else {
    eState = false;
    //LEDbreathe();
    LoRaware_norm();
    goto start;
  }
} // End of void loop