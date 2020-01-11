#ifndef BLE_H
#define BLE_H

#include <ArduinoBLE.h>



void initBLEPeripherial();
void writeMagRaw(float mag_raw[]);
void writeMagFilt(float mag_filt[]);
void updateSwitches();
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);

void initBLECentral();

#endif
