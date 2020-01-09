#ifndef CUPOLA_H
#define CUPOLA_H

#include <ArduinoBLE.h>

// Global variables
enum modes {INIT, SLEEP, ADVERTISE, STANDBY, ON};
extern enum modes mode;
extern float mag_raw[3];
extern float mag_smooth[3];
extern float mag_filt[3];

void setup();
void loop();
void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic);
void magReadHandler(BLEDevice central, BLECharacteristic characteristic);

#endif
