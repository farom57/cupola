#ifndef BLE_H
#define BLE_H

#include <ArduinoBLE.h>

#ifdef CUPOLA
extern BLEDevice central;
#else
extern BLEDevice peripheral;
#endif



void initBLEPeripherial();
bool connectBLE();
void disconnectBLE();
void writeMagRaw(float mag_raw[]);
void writeMagFilt(float mag_filt[]);
void updateSwitches();
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void connectionAliveHandler(BLEDevice central, BLECharacteristic characteristic);

void initBLECentral();
BLECharacteristic findCharacteristic(const char * uuid);
bool isAlive();
void remoteModeON(bool on);
void readRemoteMag(float res[]);

#endif
