#include "ble.h"
#include "io.h"
#include "imu.h"
#include "cupola.h"
#include "utility.h"
#include <ArduinoBLE.h>
#include <Arduino.h>

#define CONNECTION_KEEPALIVE_TIMEOUT 10000L
#define CONNECTION_KEEPALIVE_TIMEOUT2 20000L


//   ---   BLE global variables   ---

long connectionLastAlive = -2147483647L;
bool connected_central = false;
bool connected_peripheral = false;


BLEDevice remote;

// BLE Services
BLEService* batteryService;
BLEService* switchService;
BLEService* stateService;
BLEService* magService;
BLEService* aliveService;
BLEService* commandService;
// TODO: ajouter temperature

// BLE Characteristic
BLECharacteristic* batteryLevelChar;
BLEDescriptor* batteryLevelDescr;

// Button(0) & switch(1 to 4) Characteristics
BLEBoolCharacteristic* switchChar[5];
BLEDescriptor* switchDescr[5];

// STATE
BLEByteCharacteristic* stateChar;
BLEDescriptor* stateDescr;

// Mag & Acc
BLECharacteristic *magRawStringChar, *magFiltStringChar;
BLEDescriptor *magRawStringDescr, *magFiltStringDescr;
BLEFloatCharacteristic *magRawXChar, *magRawYChar, *magRawZChar, *magFiltXChar, *magFiltYChar, *magFiltZChar;
BLEDescriptor *magRawXDescr, *magRawYDescr, *magRawZDescr, *magFiltXDescr, *magFiltYDescr, *magFiltZDescr;
BLEFloatCharacteristic *magRawChar[3], *magFiltChar[3];
BLEDescriptor *magRawDescr[3], *magFiltDescr[3];

// Keep alive
BLELongCharacteristic* aliveChar;
BLEDescriptor* aliveDescr;


// Commands
BLEByteCharacteristic *rfCmdChar;
BLEDescriptor *rfCmdDescr;


//   ---------------------------------------
//   ---   Peripheral public functions   ---
//   ---------------------------------------



// prepare BLE for incomming connections
void initBLEPeripheral() {
  log_i("Starting BLE peripheral");

  if (!BLE.begin()) {
    log_e("starting BLE failed!");
    digitalWrite(LEDB, LOW);
    while (1);
  }

  // BLE Characteristic
  batteryService = new BLEService(UUID_PREFIX "00");
  batteryLevelChar = new BLECharacteristic(UUID_PREFIX "01", BLERead | BLENotify, "Not implemented");
  batteryLevelDescr = new BLEDescriptor ("2901", "Battery voltage");
  batteryService->addCharacteristic(*batteryLevelChar);
  batteryLevelChar->addDescriptor(*batteryLevelDescr);
  float bat = 0;
  for(int i=0;i<100; i++){
    bat+=analogRead(A5)/512./100.;
  }
  bat=0.657/(1-1./bat);
  char buf[32];
  snprintf(buf, 32, "%6.3f", bat);
  batteryLevelChar->writeValue(buf);

  switchService = new BLEService(UUID_PREFIX "10");
  switchChar[0] = new BLEBoolCharacteristic(UUID_PREFIX "11", BLERead | BLENotify);
  switchChar[1] = new BLEBoolCharacteristic(UUID_PREFIX "12", BLERead | BLENotify);
  switchChar[2] = new BLEBoolCharacteristic(UUID_PREFIX "13", BLERead | BLENotify);
  switchChar[3] = new BLEBoolCharacteristic(UUID_PREFIX "14", BLERead | BLENotify);
  switchChar[4] = new BLEBoolCharacteristic(UUID_PREFIX "15", BLERead | BLENotify);
  switchDescr[0] = new BLEDescriptor ("2901", "Button");
  switchDescr[1] = new BLEDescriptor ("2901", "Switch 1");
  switchDescr[2] = new BLEDescriptor ("2901", "Switch 2");
  switchDescr[3] = new BLEDescriptor ("2901", "Switch 3");
  switchDescr[4] = new BLEDescriptor ("2901", "Switch 4");
  for (int i = 0; i <= 4; i++) {
    switchService->addCharacteristic(*switchChar[i]);
    switchChar[i]->addDescriptor(*switchDescr[i]);
    switchChar[i]->writeValue(switch_i(i));
  }

  stateService = new BLEService(UUID_PREFIX "20");
  stateChar = new BLEByteCharacteristic (UUID_PREFIX "21", BLERead | BLEWrite);
  stateDescr = new BLEDescriptor ("2901", "State");
  stateService->addCharacteristic(*stateChar);
  stateChar->addDescriptor(*stateDescr);
  stateChar->writeValue(state);

  magService = new BLEService(UUID_PREFIX "30");
  magRawStringChar = new BLECharacteristic(UUID_PREFIX "31", BLERead | BLENotify, " xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  magRawStringDescr = new BLEDescriptor ("2901", "Mag raw X,Y,Z");
  magRawXChar = new BLEFloatCharacteristic(UUID_PREFIX "32", BLERead | BLENotify);
  magRawXDescr = new BLEDescriptor ("2901", "Mag raw X");
  magRawYChar = new BLEFloatCharacteristic(UUID_PREFIX "33", BLERead | BLENotify);
  magRawYDescr = new BLEDescriptor ("2901", "Mag raw Y");
  magRawZChar = new BLEFloatCharacteristic(UUID_PREFIX "34", BLERead | BLENotify);
  magRawZDescr = new BLEDescriptor ("2901", "Mag raw Z");
  magRawChar[0] = magRawXChar;
  magRawChar[1] = magRawYChar;
  magRawChar[2] = magRawZChar;
  magRawDescr[0] = magRawXDescr;
  magRawDescr[1] = magRawYDescr;
  magRawDescr[2] = magRawZDescr;

  magFiltStringChar = new BLECharacteristic(UUID_PREFIX "35", BLERead | BLENotify, " xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  magFiltStringDescr = new BLEDescriptor ("2901", "Mag filt X,Y,Z");
  magFiltXChar = new BLEFloatCharacteristic(UUID_PREFIX "36", BLERead | BLENotify);
  magFiltXDescr = new BLEDescriptor ("2901", "Mag filt X");
  magFiltYChar = new BLEFloatCharacteristic(UUID_PREFIX "37", BLERead | BLENotify);
  magFiltYDescr = new BLEDescriptor ("2901", "Mag filt Y");
  magFiltZChar = new BLEFloatCharacteristic(UUID_PREFIX "38", BLERead | BLENotify);
  magFiltZDescr = new BLEDescriptor ("2901", "Mag filt Z");
  magFiltChar[0] = magFiltXChar;
  magFiltChar[1] = magFiltYChar;
  magFiltChar[2] = magFiltZChar;
  magFiltDescr[0] = magFiltXDescr;
  magFiltDescr[1] = magFiltYDescr;
  magFiltDescr[2] = magFiltZDescr;

  magService->addCharacteristic(*magRawStringChar);
  magRawStringChar->addDescriptor(*magRawStringDescr);
  magRawStringChar->writeValue(" xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  magService->addCharacteristic(*magFiltStringChar);
  magFiltStringChar->addDescriptor(*magFiltStringDescr);
  magFiltStringChar->writeValue(" xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");


  for (int i = 0; i <= 2; i++) {
    magService->addCharacteristic(*magRawChar[i]);
    magRawChar[i]->addDescriptor(*magRawDescr[i]);
    magRawChar[i]->writeValue(0.);
    magService->addCharacteristic(*magFiltChar[i]);
    magFiltChar[i]->addDescriptor(*magFiltDescr[i]);
    magFiltChar[i]->writeValue(0.);
  }

  aliveService = new BLEService(UUID_PREFIX "50");
  aliveChar = new BLELongCharacteristic(UUID_PREFIX "51", BLERead | BLEWrite);
  aliveDescr = new BLEDescriptor ("2901", "Alive timeout in ms");
  aliveService->addCharacteristic(*aliveChar);
  aliveChar->addDescriptor(*aliveDescr);



  commandService = new BLEService(UUID_PREFIX "70");
  rfCmdChar = new BLEByteCharacteristic (UUID_PREFIX "71", BLERead | BLEWrite | BLENotify);
  rfCmdDescr = new BLEDescriptor ("2901", "RF command 0:NA 1:UP 2:DOWN 3:LEFT 4:RIGHT 5:SQUARE 6:HORN");
  commandService->addCharacteristic(*rfCmdChar);
  rfCmdChar->addDescriptor(*rfCmdDescr);
  rfCmdChar->writeValue(0);

  BLE.setLocalName("Cupola");
  //BLE.setAdvertisedService(*batteryService);
  BLE.addService(*batteryService);
  BLE.addService(*switchService);
  BLE.addService(*stateService);
  BLE.addService(*magService);
  BLE.addService(*aliveService);
  BLE.addService(*commandService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  aliveChar->setEventHandler(BLERead, connectionAliveHandler);
  rfCmdChar->setEventHandler(BLEWrite, rfCmdHandler);

  BLE.advertise();

  log_i("Bluetooth device active, waiting for connections...");
}


// Check if a central device is trying to connect and establish the connection. Return true if succesful
bool connectBLEPeripheral() {
  remote = BLE.central();
  if (remote && remote.connected()) {
    log_i("Connected:");
    log_i("  address: %s", remote.address().c_str());
    aliveChar->writeValue(CONNECTION_KEEPALIVE_TIMEOUT2);
    connectionLastAlive = millis();
    connected_peripheral = true;
    return true;
  }
  return false;
}


// Disconnect, working both for peripheral and central
void disconnectBLE() {
  log_w("Disconnected");
  remote.disconnect();
  log_w("Rebooting");
  delay(0.1);
  // system_reset();
  // NVIC_SystemReset();
  shutdown();
}


// update Mag Raw characteristic
void writeMagRaw(float mag_raw[]) {
  char buf[32];
  snprintf(buf, 32, "%6.3f,%6.3f,%6.3f", mag_raw[0], mag_raw[1], mag_raw[2]);
  magRawStringChar->writeValue(buf);
  for (int i = 0; i < 3; i++) {
    magRawChar[i]->writeValue(mag_raw[i]);
  }
}


// update Mag Filt characteristic
void writeMagFilt(float mag_smooth[]) {
  char buf[32];
  snprintf(buf, 32, "%6.3f,%6.3f,%6.3f", mag_smooth[0], mag_smooth[1], mag_smooth[2]);
  magFiltStringChar->writeValue(buf);
  for (int i = 0; i < 3; i++) {
    magFiltChar[i]->writeValue(mag_smooth[i]);
  }
}


// update State characteristic
void writeState(enum states val) {
  stateChar->writeValue((byte)val);
}

// read State characteristic
enum states readState() {
  return (enum states)stateChar->value();
}

// update rf commande characteristic
void writeRfCmd(enum rf_commands cmd) {
  rfCmdChar->writeValue((byte)cmd);
}

// read State characteristic
enum rf_commands readRfCmd() {
  return (enum rf_commands)rfCmdChar->value();
}

// update switches characteristics
void updateSwitches() {
  if (switch_1_chg()) {
    switchChar[1]->writeValue(switch_1());
  }
  if (switch_2_chg()) {
    switchChar[2]->writeValue(switch_2());
  }
  if (switch_3_chg()) {
    switchChar[3]->writeValue(switch_3());
  }
  if (switch_4_chg()) {
    switchChar[4]->writeValue(switch_4());
  }
  if (btn_chg()) {
    switchChar[0]->writeValue(btn());
  }
}

// return true if the connection is alive, ignore timeout if switch1 is ON
bool connectedPeripheral() {
  if (!remote.connected()) {
    //log_d("connection dead: not connected");
    return false;
  }

  if (debug_mode == true) {
    return true;
  }

  if (long(millis()) - connectionLastAlive < aliveChar->value()) {
    return true;
  } else {
    log_e("connection dead: timeout");
    return false;
  }
}


//   -------------------------------
//   ---   Peripheral handlers   ---
//   -------------------------------



// called when a central device tries to connect
void blePeripheralConnectHandler(BLEDevice central) {
  connectionLastAlive = millis();
  connected_peripheral = true;
}


// called when a central close the connection
void blePeripheralDisconnectHandler(BLEDevice central) {
  log_w("BLE disconnected event");
  connected_peripheral = false;
}


// called when the keepalive characteristic is changed
void connectionAliveHandler(BLEDevice central, BLECharacteristic characteristic) {
  connectionLastAlive = millis();
  log_d("alive %l",connectionLastAlive);
}
