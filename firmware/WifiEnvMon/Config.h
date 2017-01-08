#ifndef CONFIG_H
#define CONFIG_H

#define BMEOFFSET -1.5

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

