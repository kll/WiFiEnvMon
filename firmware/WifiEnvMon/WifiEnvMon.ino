#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <U8g2lib.h>

#include <ESP8266WiFi.h>          // ESP8266 Core WiFi Library
#include <DNSServer.h>            // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include "Config.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
Adafruit_BME280 bme;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);

// default values that are overwritten if there are different values in config.json
String mqtt_username = "YOUR_USERNAME";
String mqtt_key = "YOUR_API_KEY";
String sensor_name = "UNIQUE_NAME_FOR_SENSOR";
String mqtt_server = "io.adafruit.com";
uint16_t mqtt_port = 1883;

String celciusFeed;
String fahrenheitFeed;
String humidityFeed;
String pressureFeed;
String airQualityFeed;
String lightFeed;

// flag for saving data
bool shouldSaveConfig = false;

unsigned long lastUpdate;
float temperature;
float pressure;
float humidity;
int airQuality;
sensors_event_t lightEvent;

// for mqtt library
void MQTT_connect();
WiFiClient client;
Adafruit_MQTT_Client* mqtt;
Adafruit_MQTT_Publish* celciusPublish;
Adafruit_MQTT_Publish* fahrenheitPublish;
Adafruit_MQTT_Publish* humidityPublish;
Adafruit_MQTT_Publish* pressurePublish;
Adafruit_MQTT_Publish* airQualityPublish;
Adafruit_MQTT_Publish* lightPublish;

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

  if (!bme.begin(0x76))
  {
    DEBUG_PRINTLN(F("Could not find a valid BME280 sensor!"));
  }

  if (!tsl.begin())
  {
    DEBUG_PRINTLN(F("Could not find a valid TSL2561 sensor!"));
  }

  readConfig();
  initWiFi();
  initMqtt();
  initLightSensor();
}

void loop()
{
  if ((millis() - lastUpdate) < INTERVAL)
  {
    return;
  }

  lastUpdate = millis();

  readSensors();
  updateDisplay();

  MQTT_connect();
  publishData();
}

void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt->connected())
  {
    return;
  }

  DEBUG_PRINTLN(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt->connect()) != 0)
  {
    // connect will return 0 for connected
    DEBUG_PRINTLN(mqtt->connectErrorString(ret));
    DEBUG_PRINTLN(F("Retrying MQTT connection in 5 seconds..."));
    mqtt->disconnect();
    delay(5000);
    retries--;
    if (retries == 0)
    {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  
  DEBUG_PRINTLN(F("MQTT Connected!"));
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

          sensor_name = json["sensor_name"].as<String>();
          mqtt_username = json["mqtt_username"].as<String>();
          mqtt_key = json["mqtt_key"].as<String>();
          mqtt_server = json["mqtt_server"].as<String>();
          mqtt_port = json["mqtt_port"].as<uint16_t>();

          celciusFeed = mqtt_username + "/feeds/" + sensor_name + "_celcius";
          fahrenheitFeed = mqtt_username + "/feeds/" + sensor_name + "_fahrenheit";
          humidityFeed = mqtt_username + "/feeds/" + sensor_name + "_humidity";
          pressureFeed = mqtt_username + "/feeds/" + sensor_name + "_pressure_hpa";
          airQualityFeed = mqtt_username + "/feeds/" + sensor_name + "_airquality";
          lightFeed = mqtt_username + "/feeds/" + sensor_name + "_light_lux";

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
  
  WiFiManagerParameter custom_sensor_name("name", "sensor name", sensor_name.c_str(), 50);
  WiFiManagerParameter custom_mqtt_username("username", "MQTT username", mqtt_username.c_str(), 50);
  WiFiManagerParameter custom_mqtt_key("key", "MQTT API key", mqtt_key.c_str(), 50);
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", mqtt_server.c_str(), 50);
  char portBuffer[8];
  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", itoa(mqtt_port, portBuffer, 10), 6);
  
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
  sensor_name = String(custom_sensor_name.getValue());
  mqtt_username = String(custom_mqtt_username.getValue());
  mqtt_key = String(custom_mqtt_key.getValue());
  mqtt_server = String(custom_mqtt_server.getValue());
  mqtt_port = atoi(custom_mqtt_port.getValue());
  
  // save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfig();
  }
}

void initMqtt()
{
  DEBUG_PRINTLN(F("initializing MQTT client..."));
  mqtt = new Adafruit_MQTT_Client(&client, mqtt_server.c_str(), mqtt_port, mqtt_username.c_str(), mqtt_key.c_str());
  celciusPublish = new Adafruit_MQTT_Publish(mqtt, celciusFeed.c_str());
  fahrenheitPublish = new Adafruit_MQTT_Publish(mqtt, fahrenheitFeed.c_str());
  humidityPublish = new Adafruit_MQTT_Publish(mqtt, humidityFeed.c_str());
  pressurePublish = new Adafruit_MQTT_Publish(mqtt, pressureFeed.c_str());
  airQualityPublish = new Adafruit_MQTT_Publish(mqtt, airQualityFeed.c_str());
  lightPublish = new Adafruit_MQTT_Publish(mqtt, lightFeed.c_str());
}

void initLightSensor()
{
  // You can also manually set the gain or enable auto-gain support
  //tsl.setGain(TSL2561_GAIN_1X);   // No gain ... use in bright light to avoid sensor saturation
  //tsl.setGain(TSL2561_GAIN_16X);  // 16x gain ... use in low light to boost sensitivity
  tsl.enableAutoRange(true);        // Auto-gain ... switches automatically between 1x and 16x
  
  // Changing the integration time gives you better sensor resolution (402ms = 16-bit data)
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);   // fast but low resolution
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium resolution and speed
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);    // 16-bit data but slowest conversions
}

void readSensors()
{
  readHumidity();
  readTemperature();
  readPressure();
  readAirQuality();
  readLight();
}

void readAirQuality()
{
  airQuality = analogRead(MICSPIN);
  DEBUG_PRINT(F("Air Quality: "));
  DEBUG_PRINTLN(airQuality);
}

void readHumidity()
{
  humidity = bme.readHumidity();
}

void readTemperature()
{
  temperature = bme.readTemperature();

  // apply configured offset
  if (!isnan(temperature))
  {
    temperature += BMEOFFSET;
  }
}

void readPressure()
{
  pressure = bme.readPressure() / 100.0F;
}

void readLight()
{
  tsl.getEvent(&lightEvent);

  DEBUG_PRINT(F("Light lux: "));
  if (lightEvent.light)
  {
    DEBUG_PRINTLN(lightEvent.light);
  }
  else
  {
    DEBUG_PRINTLN(F("ERROR"));
  }
}

void updateDisplay()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont12_tf);
  
  u8g2.setCursor(0, 10);
  u8g2.print("T : ");
  if (isnan(temperature))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(temperature, 2);
    u8g2.print("C / ");
    u8g2.print(convertToFahrenheight(temperature), 2);
    u8g2.print("F");
  }
  
  u8g2.setCursor(0, 20);
  u8g2.print("H : ");
  if (isnan(humidity))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(humidity);
    u8g2.print("%");
  }

  u8g2.setCursor(0, 30);
  u8g2.print("P : ");
  if (isnan(pressure))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(pressure);
    u8g2.print("hPa");
  }

  u8g2.setCursor(0, 40);
  u8g2.print("AQ: ");
  if (isnan(airQuality))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(airQuality);
  }

  u8g2.setCursor(0, 50);
  u8g2.print("LT: ");
  if (isnan(lightEvent.light))
  {
    u8g2.print("ERROR");
  }
  else
  {
    u8g2.print(lightEvent.light);
    u8g2.print(" lux");
  }

  u8g2.setCursor(0, 64);
  u8g2.print(sensor_name);
  u8g2.print(" ");
  u8g2.print(WiFi.localIP());
  
  u8g2.sendBuffer();
}

void publishData()
{
  if (!isnan(temperature))
  {
    if (!celciusPublish->publish(temperature))
    {
      DEBUG_PRINTLN(F("Failed to publish celcius"));
    }

    if (!fahrenheitPublish->publish(convertToFahrenheight(temperature)))
    {
      DEBUG_PRINTLN(F("Failed to publish fahrenheit"));
    }
  }

  if (!isnan(humidity))
  {
    if (!humidityPublish->publish(humidity))
    {
      DEBUG_PRINTLN(F("Failed to publish humidity"));
    }
  }

  if (!isnan(pressure))
  {
    if (!pressurePublish->publish(pressure))
    {
      DEBUG_PRINTLN(F("Failed to publish pressure"));
    }
  }

  if (!isnan(airQuality))
  {
    if (!airQualityPublish->publish(airQuality))
    {
      DEBUG_PRINTLN(F("Failed to publish air quality"));
    }
  }

  if (!isnan(lightEvent.light))
  {
    if (!lightPublish->publish(lightEvent.light))
    {
      DEBUG_PRINTLN(F("Failed to publish light"));
    }
  }
}

float convertToFahrenheight(float celcius)
{
  float fahrenheit = celcius * 1.8 + 32;
  return round(fahrenheit * 10) / 10.0;
}

