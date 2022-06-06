#ifndef CUPOLA_H
#define CUPOLA_H



// Global variables
enum states {INIT, SLEEP, CONNECTION, STANDBY};
extern enum states state;
extern boolean debug_mode;


void setup();
void loop();


// defined in ble.h but implemented in cupola.cpp:
// void rfCmdHandler(BLEDevice central, BLECharacteristic characteristic);           // called if manual rf command changed

#endif
