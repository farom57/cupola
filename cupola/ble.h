#ifndef BLE_H
#define BLE_H

#include <ArduinoBLE.h>
#include "cupola.h"
#include "rf.h"

extern BLEDevice remote;

#define UUID_PREFIX "c3fe2f77-e1c8-4b1c-a0f3-ef88d05031"

//   ---------------------------------------
//   ---   Peripheral public functions   ---
//   ---------------------------------------
void initBLEPeripheral();            // prepare BLE for incomming connections
bool connectBLEPeripheral();         // Check if a central device is trying to connect and establish the connection. Return true if succesful
void disconnectBLE();                 // Disconnect, working both for peripheral and central
void writeMagRaw(float mag_raw[]);    // update Mag Raw characteristic
void writeMagFilt(float mag_smooth[]);  // update Mag Filt characteristic
void writeState(enum states val);     // update State characteristic
void writeRfCmd(enum rf_commands cmd);// update rf commande characteristic
enum states readState();              // read State characteristic
enum rf_commands readRfCmd();         // read Rf command characteristic
void updateSwitches();                // update switches characteristics
bool connectedPeripheral();           // return true if the connection is alive, ignore timeout if switch1 is ON
void checkStWritten();                // check if a setting char has been written and save the setting

//   -------------------------------
//   ---   Peripheral handlers   ---
//   -------------------------------
void blePeripheralConnectHandler(BLEDevice central);                              // called when a central device tries to connect
void blePeripheralDisconnectHandler(BLEDevice central);                           // called when a central close the connection
void connectionAliveHandler(BLEDevice central, BLECharacteristic characteristic); // called when the keepalive characteristic is changed
void rfCmdHandler(BLEDevice central, BLECharacteristic characteristic);           // implemented in cupola.cpp, called if manual rf command changed


#endif
