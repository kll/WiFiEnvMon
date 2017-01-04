#ifndef CONFIG_H
#define CONFIG_H

#define DHTPIN    0
#define DHTTYPE   DHT22 // options: DHT11 DHT21 DHT22

#define INTERVAL  10000

#define DEBUGP //enable/disable serial debug output

#ifdef DEBUGP
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

#endif

