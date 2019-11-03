
// LoRaware Debugging and Testing: Sensor Measurements

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
int dbgState = 0; // TESTING Default Debugging Disabled
char myStr[50];
char myStrCB[26];
char myStrVB[26];
char myStrT[26];
char myStrH[26];
char myStrHR[26];
char myStrO2[26];
char payload[255];
char emergencyPayload[64];
char outStr[255];
byte recvStatus = 0;
boolean buttonActive = false;
boolean longPressActive = false;
long buttonTimer = 0;
long longPressTime = 3000; // ms
int inputVarInt;
const sRFM_pins RFM_pins = {
  .CS = 18,
  .RST = 14,
  .DIO0 = 26,
  .DIO1 = 25,
  .DIO2 = 32,
  .DIO5 = 35,
};

void LEDbreathe() {
  for (int i = 0; i < 360; i++) {
    analogWrite(readLED, (sin(i * 0.0174533) + 1) * (127/2));
    delay(3);
  }
  for (int i = 360; i > 0; i--) {
    analogWrite(readLED, (sin(i * 0.0174533) + 1) * (127/2));
    delay(3);
  }
  analogWrite(readLED,0);
}

void LoRawareDebugMode() {
  LEDbreathe();
  analogWrite(readLED,0);
  dbgState = 1;
  bufferLength = 75; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false)
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      digitalWrite(readLED, LOW);
      particleSensor.nextSample();
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
  
  // WearState Detection (wearState = 0 Indicates device currently being worn)
      if (validHeartRate == 0){
        countHR = countHR+1;
        }
      else {
        countHR = 0; 
      }
      //if (myStrO2 == "-999.00"){
      if (validSPO2 == 0){
        countO2 = countO2+1;
        }
      else {
        countO2 = 0;
      }
  if ((countHR > 1) && (countO2 > 2)){
    wearState = 1; 
    //Serial.println("Patient Not Detected");
    } // Indicates Device is NOT currently being worn
  else {
    wearState = 0;
    }
  String str;
  str = (String(myStrVB) +","+ String(myStrT) +","+ String(myStrH) +","+ String(myStrHR) +","+ String(myStrO2)+","+ String(wearState));
  particleSensor.shutDown();
      if (inputVarInt == 1) {
        Serial.println(String(myStrVB) +"\t"+ String(myStrT) +"\t"+ String(myStrH) +"\t"+ String(myStrHR) +"\t"+ String(myStrO2)+"\t"+ String(wearState));
      }
      else if (inputVarInt ==2) {
        Serial.println(myStrVB);
      }
      else if (inputVarInt == 3) {
        Serial.println(myStrT);
      }
      else if (inputVarInt == 4) {
        Serial.println(myStrH);
      }
      else if (inputVarInt == 5) {
        if (myStrHR != "-999.00") {
          Serial.println(myStrHR); }
        else {
        }
      }
      else if (inputVarInt == 6) {
        if (myStrHR != "-999.00") {
          Serial.println(myStrO2); }
        else {
        }
      }
      if (inputVarInt == 7) {
        if (wearState == 0) {
          Serial.print("countHR: ");
          Serial.print(countHR);
          Serial.print("\t");
          Serial.print("countO2: ");
          Serial.print(countO2);
          Serial.print("\t");
          Serial.println("Device ON Patient"); }
        else {
          Serial.print("countHR: ");
          Serial.print(countHR);
          Serial.print("\t");
          Serial.print("countO2: ");
          Serial.print(countO2);
          Serial.print("\t");
          Serial.println("Device OFF Patient"); }
      }
  delay(1000);
  particleSensor.wakeUp();
} // End of Void LoRawareDebugMode()


void modeChange() {
  if (digitalRead(button) == LOW) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
    }
  }
  else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
        dbgState = !dbgState;
      } 
      else {
        String emergencyStr;
        emergencyStr = (String(0) +","+ String(0) +","+ String(0) +","+ String(0) +","+ String(0)+","+ String(0));
        Serial.println(emergencyStr);
        unsigned int str_len = 14;
        emergencyStr.toCharArray(emergencyPayload,str_len); 
        Serial.println(emergencyPayload);
        // Flash Green Square LED
        digitalWrite(pulseLED,HIGH);
        delay(500);
        digitalWrite(pulseLED,LOW);    
      }
      buttonActive = false;
      }
  }  
} // End of void ModeChange

void setup()
{
  Serial.begin(115200);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  pinMode(battPin, INPUT);
  pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), modeChange, LOW); // Define Button Pin GPIO as Interrupt Pin
  delay(2000);
  SPI.begin(SCK,MISO,MOSI,SS);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1); // Repeats this if Error Detected
  }
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1); // Repeats this if Error Detected
  }

  Serial.println("Select one of the following Data Measurements");
  Serial.println("\t1 --> ALL DATA (BV,T,H,HR,O2,WS)\n\t2 --> Battery Voltage (V)");
  Serial.println("\t3 --> Temperature (F)\n\t4 --> Humidity (%RH)");
  Serial.println("\t5 --> Heart Rate (BPM)\n\t6 --> Blood Oximetry (%)");
  Serial.println("\t7 --> Wear State (ON Patient/OFF Patient)\n");
  delay(200);
  String inputVar;
  while(!Serial.available()){}
  inputVar = (Serial.readString());
  Serial.println();
  Serial.println(inputVar);
  inputVarInt = inputVar.toInt();

  if ((inputVar.toInt() != (1)) && (inputVar.toInt() != (2)) && (inputVar.toInt() != (3)) && (inputVar.toInt() != (4)) && (inputVar.toInt() != (5)) && (inputVar.toInt() != (6)) && (inputVar.toInt() != (7))) {
        Serial.println("Not a Valid Input, try again. . . ");
        return;
      }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200 (Example uses 100)
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384 (Example uses 4096)
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
} // End of void setup()


void loop() {

  LoRawareDebugMode();
}


