#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ArduinoJson.h>

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

// default values that are overwritten if there are different values in config.json
char mqtt_username[50] = "YOUR_USERNAME";
char mqtt_key[50] = "YOUR_API_KEY";
char sensor_name[50] = "UNIQUE_NAME_FOR_SENSOR";
char mqtt_server[50] = "io.adafruit.com";
char mqtt_port[6] = "1883";

// flag for saving data
bool shouldSaveConfig = false;

unsigned long lastUpdate;
uint32_t dhtDelayMS;
sensors_event_t humidityEvent;
sensors_event_t temperatureEvent;

void saveConfigCallback()
{
  DEBUG_PRINTLN(F("Should save config"));
  shouldSaveConfig = true;
}

void setup()
{
  #ifdef DEBUGP
    Serial.begin(115200);
    DEBUG_PRINTLN(F("In setup"));
  #endif

  u8g2.begin();
  dht.begin();

  readConfig();
  initWiFi();
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

void readConfig()
{
  //DEBUG_PRINTLN(F("formatting FS..."));
  //SPIFFS.format();  // for testing
    
  DEBUG_PRINTLN(F("mounting FS..."));

  if (SPIFFS.begin())
  {
    DEBUG_PRINTLN(F("mounted file system"));
    
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      DEBUG_PRINTLN(F("reading config file"));
      
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        DEBUG_PRINTLN(F("opened config file"));
        
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());

        #ifdef DEBUGP
          json.printTo(Serial);
        #endif
        
        if (json.success())
        {
          DEBUG_PRINTLN(F("\nparsed json"));

          strcpy(sensor_name, json["sensor_name"]);
          strcpy(mqtt_username, json["mqtt_username"]);
          strcpy(mqtt_key, json["mqtt_key"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);

          DEBUG_PRINTLN(F("setings copied from json"));
        }
        else
        {
          DEBUG_PRINTLN(F("failed to load json config"));
        }
      }
    }
  }
  else
  {
    DEBUG_PRINTLN(F("failed to mount FS"));
  }
}

void saveConfig()
{
  DEBUG_PRINTLN(F("saving config..."));
    
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["sensor_name"] = sensor_name;
  json["mqtt_username"] = mqtt_username;
  json["mqtt_key"] = mqtt_key;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;

  #ifdef DEBUGP
    json.printTo(Serial);
  #endif

  File configFile = SPIFFS.open("/config.json", "w");
  if (configFile)
  {
    json.printTo(configFile);
    configFile.close();
    DEBUG_PRINTLN(F("save complete"));
  }
  else
  {
    DEBUG_PRINTLN(F("failed to open config file for writing"));
  }
}

void initWiFi()
{
  DEBUG_PRINTLN(F("initializing WiFi..."));
  
  WiFiManagerParameter custom_sensor_name("name", "sensor name", sensor_name, 50);
  WiFiManagerParameter custom_mqtt_username("username", mqtt_username, mqtt_username, 50);
  WiFiManagerParameter custom_mqtt_key("key", mqtt_key, mqtt_key, 50);
  WiFiManagerParameter custom_mqtt_server("server", mqtt_server, mqtt_server, 50);
  WiFiManagerParameter custom_mqtt_port("port", mqtt_port, mqtt_port, 6);
  
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //wifiManager.setTimeout(120);
  wifiManager.addParameter(&custom_sensor_name);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  //wifiManager.resetSettings();  // for testing

  if (!wifiManager.autoConnect("WifiEnvMonAP", "WifiEnvMon"))
  {
    DEBUG_PRINTLN(F("failed to connect and hit timeout"));
    delay(3000);
    ESP.reset();  // reset and try again
    delay(5000);
  }

  DEBUG_PRINT(F("connected to AP: "));
  DEBUG_PRINTLN(WiFi.SSID());
  DEBUG_PRINT(F("local IP: "));
  DEBUG_PRINTLN(WiFi.localIP());

  // read updated parameters
  strcpy(sensor_name, custom_sensor_name.getValue());
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_key, custom_mqtt_key.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  
  // save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfig();
  }
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

  u8g2.setCursor(0, 54);
  u8g2.print("N : ");
  u8g2.print(sensor_name);
  
  u8g2.setCursor(0, 64);
  u8g2.print("IP: ");
  u8g2.print(WiFi.localIP());
  
  u8g2.sendBuffer();
}

