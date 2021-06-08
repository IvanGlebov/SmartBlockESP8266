#include <Arduino.h> // Main library

#define BLYNK_PRINT Serial

// For air data
#define USE_SHT_AIR false
#define USE_BME true

// For gound data
#define USE_INA false
#define USE_SHT_GROUND true

#define SENSOR_NUMBER 1

#define USE_LOCAL_SERVER true

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <Wire.h>            // For all I2C sensors and I2C bus to master
#include <BH1750.h>          // For BH1750 digital light sensor
#include <SPI.h>             //
#include <Adafruit_Sensor.h> // For all Adafruit sensors lib
#include <Adafruit_BME280.h> // For BME280 sensor
#include <Adafruit_INA219.h> // For INA219 sensor
#include <WEMOS_SHT3X.h>     // For SHT30 digital temp and humidity sensor
#include <math.h>            // For using atof()
// #include <SimpleTimer.h> // For timed data collection

char auth[] = "sBCfNk-E3r-LPhH6Ry4nQxU_aBg1rwSR"; // Home sensor 1
// char auth[] = "6MMzG4BhYC5imFx72PvISj8aNABqBTQc"; // Home sensor 2
// char auth[] = "FbS_SjsZ-FSD49xWy0OWnQkSnimge3VF"; // Remote sensor 1
// char auth[] = "oEXGpkzmIHH6m3__7QBR7b_MNQflpfN3"; // Remote sensor 2

char ssid_prod[] = "Farm_router"; // prod

char ssid_local[] = "Keenetic-4926"; // home
// char ssid_local[] = "ZeroTwo1"; // home

char pass_prod[] = "zqecxwrv123"; // prod
char pass_local[] = "Q4WmFQTa";   // home
// char pass_local[] = "zqecxwrv123";   // home

// BOARD ADDRESS IS SET TO 1 //

WidgetBridge bridge1(V1);

BlynkTimer sendData;

BLYNK_CONNECTED()
{
  // Token of master board
  bridge1.setAuthToken("Rz8hI-YjZVfUY7qQb8hJGBFh48SuUn84");
}

// enum modes{master=1, sensors=2};

#define masterBus 4
#define sensorBus 3

bool collectFlag = false;

// Structure for packet variables saving
struct packetData
{
  int id;
  float airTemp;
  float airHum;
  float groundTemp;
  float groundHum;
  float lightLevel;
};

struct inaPack
{
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
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SHT30 shield
// SHT3X sht(0x44);
SHT3X sht(0x44);
void getSHT(float *); // Recieve array of 2 elements as input value and write values to these array

// BME280 instead of SHT30
Adafruit_BME280 bme;
void getBME(float *data);

// If true - debug info will be shown in console every 3 seconds
#define debug true

// Function in loop cycle called only if "debug" is true
void debugLoop(int);

void collectData();

void setup()
{
  // Serial bus initialization
  Serial.begin(115200);
  Wire.begin();

  // BH1750 initialization
  lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  // TH-12 SENSOR BLOCK //
  if (USE_INA)
  {
    if (!ina219_t.begin())
    {
      if (debug)
        Serial.println("Failed to find INA219_1 chip");
    }
    if (!ina219_h.begin())
    {
      if (debug)
        Serial.println("Failed to find INA219_2 chip");
    }
  }
  // Low range high accuracy
  ina219_t.setCalibration_16V_400mA();
  ina219_h.setCalibration_16V_400mA();

  // BME280 SENSOR BLOCK
  if (USE_BME)
  {
    if (!bme.begin(0x76, &Wire))
      Serial.println("BME280 not found");
  }
  if (USE_LOCAL_SERVER)
  {
    // String wifi = WiFi.softAPIP();
    // Blynk.begin(auth, ssid_local, pass_local, "iota02.keenetic.link", 778);
    Blynk.begin(auth, ssid_local, pass_local, IPAddress(192, 168, 1, 106), 8080);
  }
  else
  {
    Blynk.begin(auth, ssid_prod, pass_prod, IPAddress(10, 1, 92, 35), 8080);
  }
  // Blynk.begin(auth, ssid, pass, "iotablynk.iota02.keenetic.link");

  sendData.setInterval(1000L, collectData);
}

void loop()
{

  Blynk.run();
  sendData.run();
  // debugLoop(1000);
}

void collectData()
{
  packetData temp;
  Serial.println("Collecting data");
  if (USE_SHT_AIR)
  {
    float shtBuff[2];
    getSHT(shtBuff);
    temp.airTemp = shtBuff[0];
    temp.airHum = shtBuff[1];
  }
  if (USE_BME)
  {
    float bmeBuff[2];
    getBME(bmeBuff);
    temp.airTemp = bmeBuff[0];
    temp.airHum = bmeBuff[1];
  }
  if (USE_SHT_GROUND)
  {
    float shtBuff[2];
    getSHT(shtBuff);
    temp.groundTemp = shtBuff[0];
    temp.groundHum = shtBuff[1];
  }
  if (USE_INA)
  {
    temp.groundHum = getGroundHum();
    temp.groundTemp = getGroundTemp();
  }
  temp.lightLevel = getLight();
  if (SENSOR_NUMBER == 1)
  {
    bridge1.virtualWrite(V70, temp.airTemp);
    bridge1.virtualWrite(V71, temp.airHum);
    bridge1.virtualWrite(V72, temp.groundTemp);
    bridge1.virtualWrite(V73, temp.groundHum);
    bridge1.virtualWrite(V74, temp.lightLevel);
  }
  if (SENSOR_NUMBER == 2)
  {
    bridge1.virtualWrite(V75, temp.airTemp);
    bridge1.virtualWrite(V76, temp.airHum);
    bridge1.virtualWrite(V77, temp.groundTemp);
    bridge1.virtualWrite(V78, temp.groundHum);
    bridge1.virtualWrite(V79, temp.lightLevel);
  }
}

float getLight()
{
  float lux = lightSensor.readLightLevel();
  // AutoLight measurement time adjustment for different lihgt enviroment
  if (lux < 0)
    Serial.println("Error reading ligt level");
  else
  {
    if (lux > 40000.0)
    {
      // reduce measurement time - when on direct sun light
      if (lightSensor.setMTreg(32))
      {
        if (debug)
          Serial.println(F("Setting MTReg to low value for high light environment"));
      }
      else
      {
        if (debug)
          Serial.println(F("Error setting MTReg to low value for high light environment"));
      }
    }
    else
    {
      if (lux > 10.0)
      {
        // regular light enviroment
        if (lightSensor.setMTreg(69))
        {
          if (debug)
            Serial.println(F("Setting MTReg to default value for normal light environment"));
        }
        else
        {
          if (debug)
            Serial.println(F("Error setting MTReg to default value for normal light environment"));
        }
      }
      else
      {
        if (lux <= 10.0)
        {
          // very low light enviroment
          if (lightSensor.setMTreg(138))
          {
            if (debug)
              Serial.println(F("Setting MTReg to high value for low light environment"));
          }
          else
          {
            if (debug)
              Serial.println(F("Error setting MTReg to high value for low light environment"));
          }
        }
      }
    }
  }
  return lux;
}

float getGroundTemp()
{
  // Struct for data saving
  inaPack tempPack;
  tempPack.shuntvoltage = ina219_t.getShuntVoltage_mV();
  tempPack.busvoltage = ina219_t.getBusVoltage_V();
  tempPack.current_mA = ina219_t.getCurrent_mA();
  tempPack.power_mW = ina219_t.getPower_mW();
  tempPack.loadvoltage = tempPack.busvoltage + (tempPack.shuntvoltage / 1000);
  Serial.print("Bus Voltage:   ");
  Serial.print(tempPack.busvoltage);
  Serial.println(" V");
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

float getGroundHum()
{
  // Struct for data saving
  inaPack humPack;
  humPack.shuntvoltage = ina219_h.getShuntVoltage_mV();
  humPack.busvoltage = ina219_h.getBusVoltage_V();
  humPack.current_mA = ina219_h.getCurrent_mA();
  humPack.power_mW = ina219_h.getPower_mW();
  humPack.loadvoltage = humPack.busvoltage + (humPack.shuntvoltage / 1000);

  return mapFloat(humPack.busvoltage, 0, 10, 0, 100);
}

void getSHT(float *data)
{
  if (sht.get() == 0)
  {
    *(data) = sht.cTemp;
    *(data + 1) = sht.humidity;
  }
}

void getBME(float *data)
{
  *(data) = bme.readTemperature();
  *(data + 1) = bme.readHumidity();
}

void debugLoop(int delayTime)
{
  Serial.println("--------------------------------------------");
  Serial.println("Light level : " + String(getLight()) + " lux");
  if (USE_SHT_AIR || USE_SHT_GROUND)
  {
    float sht[2];
    Serial.println("SHT30 data reading");
    getSHT(sht);
    Serial.println("SHT30 temperature : " + String(sht[0]) + " C");
    Serial.println("STH30 humidity : " + String(sht[1]) + " %");
  }
  float bme_buff[2];
  getBME(bme_buff);
  Serial.println("BME 280 sensor");
  Serial.println("BME280 humidity :" + String(bme_buff[1]) + " %");
  Serial.println("BME280 temperature :" + String(bme_buff[0]) + " C");

  Serial.println("Ground humidity : " + String(getGroundHum()) + " %");
  Serial.println("Ground temperature : " + String(getGroundTemp()) + " C");

  Serial.println("--------------------------------------------");
  delay(delayTime);
}
