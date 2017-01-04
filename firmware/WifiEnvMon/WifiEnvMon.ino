#include <Arduino.h>

#include <Wire.h>
#include <U8g2lib.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <ESP8266WiFi.h>          // ESP8266 Core WiFi Library
#include <DNSServer.h>            // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include "Config.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
DHT_Unified dht(DHTPIN, DHTTYPE);

unsigned long lastUpdate;
uint32_t dhtDelayMS;
sensors_event_t humidityEvent;
sensors_event_t temperatureEvent;

void setup()
{
  #ifdef DEBUGP
    Serial.begin(115200);
  #endif

  DEBUG_PRINTLN(F("In setup"));
  
  u8g2.begin();
  dht.begin();

  initDhtDelay();

  WiFiManager wifiManager;
  wifiManager.autoConnect("WifiEnvMonAP", "WifiEnvMon");
}

void loop()
{
  if ((millis() - lastUpdate) < dhtDelayMS)
  {
    return;
  }

  lastUpdate = millis();

  readSensors();
  updateDisplay();
}

void initDhtDelay()
{
  DEBUG_PRINTLN(F("Reading required delay for DHT sensor."));
  sensor_t sensor;
  dht.humidity().getSensor(&sensor);
  dhtDelayMS = sensor.min_delay / 1000;
}

void readSensors()
{
  readHumidity();
  readTemperature();
}

void readHumidity()
{
  dht.humidity().getEvent(&humidityEvent);
}

void readTemperature()
{
  dht.temperature().getEvent(&temperatureEvent);
}

void updateDisplay()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont12_tf);
  
  u8g2.setCursor(0, 10);
  u8g2.print("T : ");
  if (isnan(temperatureEvent.temperature))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(temperatureEvent.temperature);
    u8g2.print("C / ");
    u8g2.print(temperatureEvent.temperature * 1.8 + 32);
    u8g2.print("F");
  }
  
  u8g2.setCursor(0, 20);
  u8g2.print("H : ");
  if (isnan(humidityEvent.relative_humidity))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(humidityEvent.relative_humidity);
    u8g2.print("%");
  }

  u8g2.setCursor(0, 30);
  u8g2.print("IP: ");
  u8g2.print(WiFi.localIP());
  
  u8g2.sendBuffer();
}

