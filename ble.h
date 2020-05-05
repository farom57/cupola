#ifndef BLE_H
#define BLE_H

#include <ArduinoBLE.h>
#include "cupola.h"

extern BLEDevice remote;

#define UUID_PREFIX "c3fe2f77-e1c8-4b1c-a0f3-ef88d05031"

//   ---------------------------------------
//   ---   Peripheral public functions   ---
//   ---------------------------------------
void initBLEPeripheral();            // prepare BLE for incomming connections
bool connectBLEPeripheral();         // Check if a central device is trying to connect and establish the connection. Return true if succesful
void disconnectBLE();                 // Disconnect, working both for peripheral and central
void writeMagRaw(float mag_raw[]);    // update Mag Raw characteristic
void writeMagFilt(float mag_filt[]);  // update Mag Filt characteristic
void writeAcc(float acc[]);           // update Acc characteristic
void writeState(enum states val);     // update State characteristic
enum states readState();              // read State characteristic
void updateSwitches();                // update switches characteristics
bool connectedPeripheral();          // return true if the connection is alive, ignore timeout if switch1 is ON


//   -------------------------------
//   ---   Peripheral handlers   ---
//   -------------------------------
void magReadHandler(BLEDevice central, BLECharacteristic characteristic);         // implemented in cupola.cpp, called if a mag measurment is requested
void blePeripheralConnectHandler(BLEDevice central);                              // called when a central device tries to connect
void blePeripheralDisconnectHandler(BLEDevice central);                           // called when a central close the connection
void connectionAliveHandler(BLEDevice central, BLECharacteristic characteristic); // called when the keepalive characteristic is changed

//   ------------------------------------
//   ---   Central public functions   ---
//   ------------------------------------
void initBLECentral();                    // Prepare BLE central
bool connectBLECentral();                 // Scan for BLE peripheral and connect, return true if succesful
//void disconnectBLE();                   // Disconnect, working both for peripheral and central
void readRemoteMagRaw(float mag_raw[]);   // Read Mag Raw on the remote device
void readRemoteMagFilt(float mag_filt[]); // Read Mag Filt on the remote device
void readRemoteAcc(float acc[]);          // Read Acc on the remote device
void setRemoteState(enum states state);   // set the state on the remote device
bool connectedCentral();                  // return true if the connection is alive, ignore timeout if debug is true
bool remoteBtn();                         // return state of the remote btn
//   ----------------------------
//   ---   Central handlers   ---
//   ----------------------------
//void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic);  // implemented in cupola.cpp, called if a switch is changed on the peripheral

//   -------------------------------------
//   ---   Private functions   ---
//   -------------------------------------
BLECharacteristic* findCharacteristic(const char * uuid); // return the characteristic that correspond to the uuid





#endif
