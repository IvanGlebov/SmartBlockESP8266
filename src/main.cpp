#include <Arduino.h> // Main library
#include <Wire.h> // For all I2C sensors and I2C bus to master
#include <BH1750.h> // For BH1750 digital light sensor
#include <SPI.h> //
#include <Adafruit_Sensor.h> // For all Adafruit sensors lib
#include <Adafruit_BME280.h> // For BME280 sensor
#include <Adafruit_INA219.h> // For INA219 sensor 
#include <WEMOS_SHT3X.h> // For SHT30 digital temp and humidity sensor
#include <math.h> // For using atof()
// #include <SimpleTimer.h> // For timed data collection

// BOARD ADDRESS IS SET TO 1 //
enum modes{master=1, sensors=2};

#define masterBus 4
#define sensorBus 3

bool collectFlag = false;

// Structure for packet variables saving
struct packetData{
  int id;
  float airTemp;
  float airHum;
  float groundTemp;
  float groundHum;
  float lightLevel;
};

struct inaPack{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
};

/* BH1750 lisht sensor settings */
BH1750 lightSensor;
// Function to get light level in lux from BH1750 sensor with auto enviroment adjustment
float getLight();

// TH-12 SENSOR BLOCK //
Adafruit_INA219 ina219_h(0x40);
Adafruit_INA219 ina219_t(0x41);
float getGroundTemp();
float getGroundHum();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }


// SHT30 shield 
// SHT3X sht30(0x44);
// void getSHT(float *); // Recieve array of 2 elements as input value and write values to these array


// BME280 instead of SHT30
Adafruit_BME280 bme;
void getBME(float *data);

// If true - debug info will be shown in console every 3 seconds
#define debug true 

// Function in loop cycle called only if "debug" is true
void debugLoop(int);


void requestEvent();
// New format - sending data to function in the structure
String structFormPacket(packetData d1);
// Old version with sending data right to function, not in the structure format
// String formPacket(int id, float tempAir, float humAir, float tempGround, float humGround, float lightLevel);

// Function to switch between master and sensors I2C buses
void switchBus(int mode);

packetData sensorsData;

packetData collectData();

void setup() {
  // Serial bus initialization
  Serial.begin(115200);
  // pinMode(masterBus, OUTPUT);
  // pinMode(sensorBus, OUTPUT);

  // switchBus(sensors);
  // digitalWrite(masterBus, LOW);
  // digitalWrite(sensorBus, HIGH);
  // digitalWrite(masterBus, LOW);
  // digitalWrite(sensorBus, HIGH);
  // I2C bus initialization
  Wire.begin(9); 

  // BH1750 initialization
  lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  // TH-12 SENSOR BLOCK //
  if (! ina219_t.begin()) {
    if (debug) Serial.println("Failed to find INA219_1 chip");
  }
  if (! ina219_h.begin()){
    if (debug) Serial.println("Failed to find INA219_2 chip");
  }
  // Low range high accuracy
  ina219_t.setCalibration_16V_400mA();
  ina219_h.setCalibration_16V_400mA();

  // BME280 SENSOR BLOCK
  if (!bme.begin(0x76, &Wire))
    Serial.println("BME280 finfing error");
  

  // Set up timer for data collecting
  // dataCollect.setInterval(collectInterval);

  // digitalWrite(masterBus, HIGH);
  // digitalWrite(sensorBus, LOW);
  // Board will wait for master call, collect data and send it to master
  Wire.onRequest(requestEvent); 
  Serial.println("Setup finished");
  collectData();
}

void loop() {
  // digitalWrite(masterBus, HIGH);
  // digitalWrite(sensorBus, HIGH);
  if (collectFlag == true){
    collectFlag = false;
    sensorsData = collectData();
  }
  debugLoop(1000);

}

packetData collectData(){
  packetData temp;
  Serial.println("Collecting data");
  // Open sensors bus for collecting data
  // and close master bus
  digitalWrite(masterBus, LOW);
  digitalWrite(sensorBus, HIGH);
  // Collect data
  // float shtBuff[2];
  // getSHT(shtBuff);

  float bmeBuff[2];
  getBME(bmeBuff);
  temp.airHum = bmeBuff[0];
  temp.airTemp = bmeBuff[1];
  temp.groundHum = getGroundHum();
  temp.groundTemp = getGroundTemp();
  temp.id = 2;
  temp.lightLevel = getLight();
  // Close sensors bus and open master bus again
  digitalWrite(sensorBus, LOW);
  digitalWrite(masterBus, HIGH);
  // return data
  return temp;
}

float getLight(){
  float lux = lightSensor.readLightLevel(true);
  // AutoLight measurement time adjustment for different lihgt enviroment
  if(lux < 0) Serial.println("Error reading ligt level");
  else {
    if (lux > 40000.0){
      // reduce measurement time - when on direct sun light
      if (lightSensor.setMTreg(32)){
        if (debug) Serial.println(F("Setting MTReg to low value for high light environment"));
      }
      else {
        if(debug) Serial.println(F("Error setting MTReg to low value for high light environment"));
      }
    }
    else {
      if (lux > 10.0){
        // regular light enviroment
        if (lightSensor.setMTreg(69)){
          if (debug) Serial.println(F("Setting MTReg to default value for normal light environment"));
        }
        else {
          if (debug) Serial.println(F("Error setting MTReg to default value for normal light environment"));
        }
      }
      else {
        if (lux <= 10.0){
          // very low light enviroment
          if (lightSensor.setMTreg(138)){
            if (debug) Serial.println(F("Setting MTReg to high value for low light environment"));
          }
          else {
            if (debug) Serial.println(F("Error setting MTReg to high value for low light environment"));
          }
        }
      }
    }
  }
  return lux;
}

float getGroundTemp(){
  // Struct for data saving
  inaPack tempPack;
  tempPack.shuntvoltage = ina219_t.getShuntVoltage_mV();
  tempPack.busvoltage = ina219_t.getBusVoltage_V();
  tempPack.current_mA = ina219_t.getCurrent_mA();
  tempPack.power_mW = ina219_t.getPower_mW();
  tempPack.loadvoltage = tempPack.busvoltage + (tempPack.shuntvoltage / 1000);
   Serial.print("Bus Voltage:   "); Serial.print(tempPack.busvoltage); Serial.println(" V");
  // Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  // Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  // Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  // Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  // Serial.println("------------------------------------");
  // Serial.println("Humidity : " + String(mapFloat(monitor.busVoltage(), 0, 4.36, 0, 100)));
  // Serial.println("Temperature    : " + String(map(busvoltage, 0, 10, 0, 100)));
  // Serial.println("Humidity : " + String(map(busvoltage, 0, 10, 0, 100)));
  return mapFloat(tempPack.busvoltage, 0, 10, -40, 80);
}

float getGroundHum(){
  // Struct for data saving
  inaPack humPack;
  humPack.shuntvoltage = ina219_h.getShuntVoltage_mV();
  humPack.busvoltage = ina219_h.getBusVoltage_V();
  humPack.current_mA = ina219_h.getCurrent_mA();
  humPack.power_mW = ina219_h.getPower_mW();
  humPack.loadvoltage = humPack.busvoltage + (humPack.shuntvoltage / 1000);

  return mapFloat(humPack.busvoltage, 0, 10, 0, 100);
}

void getSHT(float *data){
  // float *pointer = data;
  // if(sht30.get() == 0){
    // *(data) = sht30.cTemp;
    // *(data+1) = sht30.humidity;
  // }
}

void getBME(float *data){
  *(data) = bme.readTemperature();
  *(data+1) = bme.readHumidity();
}

void debugLoop(int delayTime){
  // digitalWrite(masterBus, HIGH);
  // digitalWrite(sensorBus, HIGH);
  // switchBus(sensors);
  Serial.println("--------------------------------------------");
  Serial.println("Light level : " + String(getLight()) + " lux");


  // Serial.println("SHT30 data reading");
  // getSHT(sht);
  // Serial.println("SHT30 temperature : " + String(sht[0]) + " C");
  // Serial.println("STH30 humidity : " + String(sht[1]) + " %");

  float bme_buff[2];
  getBME(bme_buff);
  Serial.println("BME 280 sensor");
  Serial.println("BME280 humidity :" + String(bme_buff[1]) + " %");
  Serial.println("BME280 temperature :" + String(bme_buff[0]) + " C");
  
  Serial.println("Ground humidity : " + String(getGroundHum()) + " %");
  Serial.println("Ground temperature : " + String(getGroundTemp()) + " C");
  
  Serial.println("--------------------------------------------");
  // Serial.println("swithc")
  delay(delayTime);
  
}

String structFormPacket(packetData d1){
  String temp = "";
  double l, r;

  // ID adding to string
  if(d1.id < 10) temp += "0" + String(d1.id);
  else temp += String(d1.id);

  // Air temperature adding to string
  if(d1.airTemp >= 0) temp += "+";
  else temp += "-";
  //  abs(x) ((x)>0?(x):-(x))
  float airTemp = abs(d1.airTemp);
  l = int(airTemp); // Left part of the number
  r = airTemp - l; // right part of the nember
  r *= 100; // Convert r value to full integer
  r = int(r);
  if(l < 10) temp += "0" + String(static_cast<int>(l));
  else if (l < 100) temp += String(static_cast<int>(l));
  if(r < 10) temp += "0" + String(static_cast<int>(r));
  else if (r < 100)  temp += String(static_cast<int>(r));

  // Air humidity adding to string
  l = int(d1.airHum);
  r = int((d1.airHum-l)*100);
  if(l < 10) temp += "00" + String(static_cast<int>(l));
  else if (l < 100) temp += "0" + String(static_cast<int>(l));
  else if (l == 100) temp += "100/";
  if(r < 10) temp += "0" + String(static_cast<int>(r));
  else if (r < 100)  temp += String(static_cast<int>(r));

  //tempGround
  if(d1.groundTemp >= 0) temp += "+";
  else temp += "-";
  //  abs(x) ((x)>0?(x):-(x))
  float groundTemp = abs(d1.groundTemp);
  l = int(groundTemp); // Left part of the number
  r = groundTemp - l; // right part of the nember
  r *= 100; // Convert r value to full integer
  r = int(r);
  // temp += "T";
  if(l < 10) temp += "0" + String(static_cast<int>(l));
  else if (l < 100) temp += String(static_cast<int>(l));
  if(r < 10) temp += "0" + String(static_cast<int>(r));
  else if (r < 100)  temp += String(static_cast<int>(r));

  // Ground humidity adding to string
  l = int(d1.groundHum);
  r = int((d1.groundHum-l)*100);
  if(l < 10) temp += "00" + String(static_cast<int>(l));
  else if (l < 100) temp += "0" + String(static_cast<int>(l));
  else if (l == 100) temp += "100";
  if(r < 10) temp += "0" + String(static_cast<int>(r));
  else if (r < 100)  temp += String(static_cast<int>(r));

  // Light level adding to string
  l = int(d1.lightLevel);
  r = int((d1.lightLevel-l)*100);
  if(l < 10) temp += "000" + String(static_cast<int>(l));
  else if (l < 100) temp += "00" + String(static_cast<int>(l));
  else if (l < 1000) temp += "0" + String(static_cast<int>(l));
  else if (l < 10000) temp += String(static_cast<int>(l));
  else temp += "9999/";
  if(r < 10) temp += "0" + String(static_cast<int>(r));
  else if (r < 100)  temp += String(static_cast<int>(r));

  return temp;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Serial.println("Request from Master to send data");

  String packet1 = structFormPacket(sensorsData);

  int len = 28;
  for(int i=0; i < len; i++){
    Wire.write(packet1[i]);
    if (debug) Serial.print(packet1[i]);
  }
  if (debug) Serial.println();

  collectFlag = true;
}

// Function to switch between master and sensors I2C buses
void switchBus(int mode){
  switch(mode){
    case master:
      digitalWrite(masterBus, HIGH);
      digitalWrite(sensorBus, LOW);
      if (debug) Serial.println("Switching to master bus");
      break;
    case sensors:
      digitalWrite(masterBus, LOW);
      digitalWrite(sensorBus, HIGH);
      if (debug) Serial.println("Switching to sensors bus");
      break;
  }
}