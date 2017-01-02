#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN    0
#define DHTTYPE   DHT22 // options: DHT11 DHT21 DHT22

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
DHT_Unified dht(DHTPIN, DHTTYPE);

unsigned long lastUpdate;
uint32_t dhtDelayMS;
sensors_event_t humidityEvent;
sensors_event_t temperatureEvent;

void setup()
{
  u8g2.begin();
  dht.begin();

  initDhtDelay();
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
  u8g2.setFont(u8g2_font_ncenB14_tr);
  
  u8g2.setCursor(0, 20);
  u8g2.print("T: ");
  u8g2.print(temperatureEvent.temperature);
  u8g2.print(" *C");

  u8g2.setCursor(0, 40);
  u8g2.print("H: ");
  u8g2.print(humidityEvent.relative_humidity);
  u8g2.print(" %");
  
  u8g2.sendBuffer();
}

