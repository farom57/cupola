#include "settings.h"
#include "ble.h"
#include "io.h"
#include "imu.h"
#include "cupola.h"
#include "utility.h"
#include <ArduinoBLE.h>
#include "nrf_temp.h"


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
BLEService* accService;
BLEService* aliveService;
BLEService* stService;
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
BLECharacteristic *magRawStringChar, *magFiltStringChar, *accStringChar, *headStrChar;
BLEDescriptor *magRawStringDescr, *magFiltStringDescr, *accStringDescr;
BLEFloatCharacteristic *magRawXChar, *magRawYChar, *magRawZChar, *magFiltXChar, *magFiltYChar, *magFiltZChar, *accXChar, *accYChar, *accZChar, *headRawChar, *headFiltChar;
BLEDescriptor *magRawXDescr, *magRawYDescr, *magRawZDescr, *magFiltXDescr, *magFiltYDescr, *magFiltZDescr, *accXDescr, *accYDescr, *accZDescr, *headRawDescr, *headFiltDescr, *headStrDescr;
BLEFloatCharacteristic *magRawChar[3], *magFiltChar[3], *accChar[3];
BLEDescriptor *magRawDescr[3], *magFiltDescr[3], *accDescr[3];

// Keep alive
BLELongCharacteristic* aliveChar;
BLEDescriptor* aliveDescr;

// Settings
BLECharacteristic *stChar[ST_NB];
BLEDescriptor *stDescr[ST_NB];

// Commands
BLEByteCharacteristic *rfCmdChar;
BLEDescriptor *rfCmdDescr;
//BLEBoolCharacteristic *trackChar;
//BLEDescriptor *trackDescr;
BLEFloatCharacteristic *targetHeadingChar;
BLEDescriptor *targetHeadingDescr;

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
  // TODO: implement battery service

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
  magRawStringChar = new BLECharacteristic(UUID_PREFIX "31", BLERead, " xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
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

  headRawChar = new BLEFloatCharacteristic(UUID_PREFIX "39", BLERead | BLENotify);
  headRawDescr = new BLEDescriptor ("2901", "Heading raw");
  headFiltChar = new BLEFloatCharacteristic(UUID_PREFIX "3A", BLERead | BLENotify);
  headFiltDescr = new BLEDescriptor ("2901", "Heading filt");
  headStrChar = new BLECharacteristic(UUID_PREFIX "3B", BLERead | BLENotify, "xxx.xxxxx");
  headStrDescr = new BLEDescriptor ("2901", "Heading filt str");


  magService->addCharacteristic(*magRawStringChar);
  magRawStringChar->addDescriptor(*magRawStringDescr);
  magRawStringChar->writeValue(" xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  magService->addCharacteristic(*magFiltStringChar);
  magFiltStringChar->addDescriptor(*magFiltStringDescr);
  magFiltStringChar->writeValue(" xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  magService->addCharacteristic(*headRawChar);
  headRawChar->addDescriptor(*headRawDescr);
  headRawChar->writeValue(0.);
  magService->addCharacteristic(*headFiltChar);
  headFiltChar->addDescriptor(*headFiltDescr);
  headFiltChar->writeValue(0.);
  magService->addCharacteristic(*headStrChar);
  headStrChar->addDescriptor(*headStrDescr);
  headStrChar->writeValue("xxx.xxxxx");

  for (int i = 0; i <= 2; i++) {
    magService->addCharacteristic(*magRawChar[i]);
    magRawChar[i]->addDescriptor(*magRawDescr[i]);
    magRawChar[i]->writeValue(0.);
    magService->addCharacteristic(*magFiltChar[i]);
    magFiltChar[i]->addDescriptor(*magFiltDescr[i]);
    magFiltChar[i]->writeValue(0.);
  }

  accService = new BLEService(UUID_PREFIX "40");
  accStringChar = new BLECharacteristic(UUID_PREFIX "41", BLERead | BLENotify, " xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  accStringDescr = new BLEDescriptor ("2901", "Acc X,Y,Z");
  accXChar = new BLEFloatCharacteristic(UUID_PREFIX "42", BLERead | BLENotify);
  accXDescr = new BLEDescriptor ("2901", "Acc X");
  accYChar = new BLEFloatCharacteristic(UUID_PREFIX "43", BLERead | BLENotify);
  accYDescr = new BLEDescriptor ("2901", "Acc Y");
  accZChar = new BLEFloatCharacteristic(UUID_PREFIX "44", BLERead | BLENotify);
  accZDescr = new BLEDescriptor ("2901", "Acc Z");
  accChar[0] = accXChar;
  accChar[1] = accYChar;
  accChar[2] = accZChar;
  accDescr[0] = accXDescr;
  accDescr[1] = accYDescr;
  accDescr[2] = accZDescr;
  accService->addCharacteristic(*accStringChar);
  accStringChar->addDescriptor(*accStringDescr);
  accStringChar->writeValue(" xxx.xxxxx, yyy.yyyyy, zzz.zzzzz");
  for (int i = 0; i <= 2; i++) {
    accService->addCharacteristic(*accChar[i]);
    accChar[i]->addDescriptor(*accDescr[i]);
    accChar[i]->writeValue(0.);
  }


  aliveService = new BLEService(UUID_PREFIX "50");
  aliveChar = new BLELongCharacteristic(UUID_PREFIX "51", BLERead | BLEWrite);
  aliveDescr = new BLEDescriptor ("2901", "Alive timeout in ms");
  aliveService->addCharacteristic(*aliveChar);
  aliveChar->addDescriptor(*aliveDescr);

  stService = new BLEService(UUID_PREFIX "60");
  for (int i = 0; i < ST_NB; i++) {
    char uuid[40];
    char val[ST_VAL_LEN] = "not initalized";
    int sufix = 0x60 + i + 1;
    sprintf(uuid, UUID_PREFIX "%02x", sufix);
    readSt(settings[i].key, val);
    stChar[i] = new BLECharacteristic(uuid, BLERead | BLEWrite, ST_VAL_LEN);
    stChar[i]->writeValue(val, strlen(val) + 1);
    stDescr[i] = new BLEDescriptor ("2901", settings[i].key);
    stService->addCharacteristic(*stChar[i]);
    stChar[i]->addDescriptor(*stDescr[i]);
  }

  commandService = new BLEService(UUID_PREFIX "70");
  rfCmdChar = new BLEByteCharacteristic (UUID_PREFIX "71", BLERead | BLEWrite | BLENotify);
  rfCmdDescr = new BLEDescriptor ("2901", "RF command 0:NA 1:UP 2:DOWN 3:LEFT 4:RIGHT 5:SQUARE 6:HORN");
  commandService->addCharacteristic(*rfCmdChar);
  rfCmdChar->addDescriptor(*rfCmdDescr);
  rfCmdChar->writeValue(0);
  //trackChar = new BLEBoolCharacteristic (UUID_PREFIX "72", BLERead | BLEWrite | BLENotify);
  //trackDescr = new BLEDescriptor ("2901", "Enable tracking");
  //commandService->addCharacteristic(*trackChar);
  //trackChar->addDescriptor(*trackDescr);
  //trackChar->writeValue(0);
  targetHeadingChar = new BLEFloatCharacteristic (UUID_PREFIX "73", BLERead | BLEWrite | BLENotify);
  targetHeadingDescr = new BLEDescriptor ("2901", "Heading target in deg");
  commandService->addCharacteristic(*targetHeadingChar);
  targetHeadingChar->addDescriptor(*targetHeadingDescr);
  targetHeadingChar->writeValue(0.);


  BLE.setLocalName("Cupola");
  BLE.setAdvertisedService(*batteryService);

  BLE.addService(*batteryService);
  BLE.addService(*switchService);
  BLE.addService(*stateService);
  BLE.addService(*magService);
  BLE.addService(*accService);
  BLE.addService(*aliveService);
  BLE.addService(*stService);
  BLE.addService(*commandService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  aliveChar->setEventHandler(BLERead, connectionAliveHandler);
  magRawStringChar->setEventHandler(BLERead, magReadHandler);
  magRawXChar->setEventHandler(BLERead, magReadHandler);
  magRawYChar->setEventHandler(BLERead, magReadHandler);
  magRawZChar->setEventHandler(BLERead, magReadHandler);
  headRawChar->setEventHandler(BLERead, magReadHandler);
  //rfCmdChar->setEventHandler(BLEWrite, rfCmdHandler);

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
  system_reset();
}


// update Mag Raw characteristic
void writeMagRaw(float mag_raw[]) {
  char buf[32];
  snprintf(buf, 32, "%6.3f,%6.3f,%6.3f", mag_raw[0], mag_raw[1], mag_raw[2]);
  magRawStringChar->writeValue(buf);
  for (int i = 0; i < 3; i++) {
    magRawChar[i]->writeValue(mag_raw[i]);
  }
  headRawChar->writeValue(heading(mag_raw)*360/2./PI);

}


// update Mag Filt characteristic
void writeMagFilt(float mag_filt[]) {
  char buf[32];
  snprintf(buf, 32, "%6.3f,%6.3f,%6.3f", mag_filt[0], mag_filt[1], mag_filt[2]);
  magFiltStringChar->writeValue(buf);
  for (int i = 0; i < 3; i++) {
    magFiltChar[i]->writeValue(mag_filt[i]);
  }

  headFiltChar->writeValue(heading_smooth);
  snprintf(buf, 32, "%8.5f", heading_smooth);
  headStrChar->writeValue(buf);
}


// update Acc characteristic
void writeAcc(float acc[]) {
  char buf[32];
  snprintf(buf, 32, "%6.3f,%6.3f,%6.3f", acc[0], acc[1], acc[2]);
  accStringChar->writeValue(buf);
  for (int i = 0; i < 3; i++) {
    accChar[i]->writeValue(acc[i]);
  }
  //log_d("Acc: %s", buf);
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
void writeRfCmd(enum rf_commands cmd){
  rfCmdChar->writeValue((byte)cmd);
}

// read State characteristic
enum rf_commands readRfCmd() {
  return (enum rf_commands)rfCmdChar->value();
}

float readTarget(){
  log_d("%f",targetHeadingChar->value());
  return targetHeadingChar->value();
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

  if (operating_mode == DEBUG) {
    return true;
  }

  if (millis() - connectionLastAlive < aliveChar->value()) {
    return true;
  } else {
    log_e("connection dead: timeout");
    return false;
  }
}

// check if a setting char has been written and save the setting
void checkStWritten() {
  for (int i = 0; i < ST_NB; i++) {
    if (stChar[i]->written()) {
      saveSt(settings[i].key, (const char*)stChar[i]->value(), stChar[i]->valueLength());
    }
  }
}



//   -------------------------------
//   ---   Peripheral handlers   ---
//   -------------------------------



// implemented in cupola.cpp, called if a mag measurment is requested
//void magReadHandler(BLEDevice central, BLECharacteristic characteristic);


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
}






//   ------------------------------------
//   ---   Central public functions   ---
//   ------------------------------------


// Prepare BLE central
void initBLECentral() {
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.scanForName("Cupola");
  log_i("Scanning ...");
}


// Scan for BLE peripheral and connect, return true if succesful
bool connectBLECentral() {
  remote = BLE.available();
  if (remote) {
    log_i("Found");
    BLE.stopScan();

    // connect to the peripheral
    if (remote.connect()) {
      log_i("Connected");
    } else {
      log_e("Failed to connect!");
      return false;
    }

    //log_d("Discovering attributes ...");
    if (remote.discoverAttributes()) {
      //log_d("Attributes discovered");
    } else {
      log_e("Attribute discovery failed!");
      disconnectBLE();
      return false;
    }

    // BLE Characteristic
    batteryLevelChar = findCharacteristic(UUID_PREFIX "01");
    switchChar[0] = (BLEBoolCharacteristic*)findCharacteristic(UUID_PREFIX "11");
    switchChar[1] = (BLEBoolCharacteristic*)findCharacteristic(UUID_PREFIX "12");
    switchChar[2] = (BLEBoolCharacteristic*)findCharacteristic(UUID_PREFIX "13");
    switchChar[3] = (BLEBoolCharacteristic*)findCharacteristic(UUID_PREFIX "14");
    switchChar[4] = (BLEBoolCharacteristic*)findCharacteristic(UUID_PREFIX "15");
    stateChar = (BLEByteCharacteristic*)findCharacteristic(UUID_PREFIX "21");
    magRawStringChar = findCharacteristic(UUID_PREFIX "31");
    magRawXChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "32");
    magRawYChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "33");
    magRawZChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "34");
    magRawChar[0] = magRawXChar;
    magRawChar[1] = magRawYChar;
    magRawChar[2] = magRawZChar;
    magFiltStringChar = findCharacteristic(UUID_PREFIX "35");
    magFiltXChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "36");
    magFiltYChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "37");
    magFiltZChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "38");
    magFiltChar[0] = magFiltXChar;
    magFiltChar[1] = magFiltYChar;
    magFiltChar[2] = magFiltZChar;
    accStringChar = findCharacteristic(UUID_PREFIX "41");
    accXChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "42");
    accYChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "43");
    accZChar = (BLEFloatCharacteristic*)findCharacteristic(UUID_PREFIX "44");
    accChar[0] = accXChar;
    accChar[1] = accYChar;
    accChar[2] = accZChar;
    aliveChar = (BLELongCharacteristic*)findCharacteristic(UUID_PREFIX "51");

    if (
      batteryLevelChar &&
      switchChar[0] && switchChar[1] && switchChar[2] && switchChar[3] && switchChar[4] &&
      stateChar &&
      magRawStringChar && magRawXChar && magRawYChar && magRawZChar && magFiltStringChar && magFiltXChar && magFiltYChar && magFiltZChar &&
      accStringChar && accXChar && accYChar && accZChar &&
      aliveChar ) {
      //log_d("Characteristics OK");
    } else {
      log_e("Problem with haracteristics");
      disconnectBLE();
      return false;
    }

    if (!switchChar[0]->subscribe()) {
      log_e("Cannot subscribe to switchChar");
      disconnectBLE();
      return false;
    }
    //    switchChar[0]->setEventHandler(BLEWritten, btnChangedHandler);
    aliveChar->read();
    aliveChar->writeValue(CONNECTION_KEEPALIVE_TIMEOUT2);
    connectionLastAlive = millis();
    log_i("Connected");
    connected_central = true;
    return true;
  }
  //log_w("not found");
  return false;
}


// Disconnect, working both for peripheral and central, implemented above
// void disconnectBLE();


// Read Mag Raw on the remote device
void readRemoteMagRaw(float res[]) {
  for (int i = 0; i < 3; i++) {
    float tmp;
    magRawChar[i]->readValue(&tmp, 4);
    //printg("magFiltChar[%d]=%f=%0h",i,tmp,tmp);
    res[i] = tmp;
  }
}


// Read Mag Filt on the remote device
void readRemoteMagFilt(float res[]) {
  for (int i = 0; i < 3; i++) {
    float tmp;
    magFiltChar[i]->readValue(&tmp, 4);
    res[i] = tmp;
  }
}


// Read Acc on the remote device
void readRemoteAcc(float res[])  {
  for (int i = 0; i < 3; i++) {
    float tmp;
    accChar[i]->readValue(&tmp, 4);
    res[i] = tmp;
  }
}



// set the state on the remote device
void setRemoteState(enum states state) {
  stateChar->writeValue((uint8_t)state);
}

// return true if the connection is alive, ignore timeout if debug is true
bool connectedCentral() {
  if (!BLE.connected()) {
    //log_e("connection dead: not connected");
    connected_central = false;
    return false;
  }
  //printg("isAlive(): millis=%ld connectionLastAlive=%ld\n\r", millis(), connectionLastAlive);
  if (millis() - connectionLastAlive > CONNECTION_KEEPALIVE_TIMEOUT) {

    if (aliveChar->read()) {
      //printg("  update OK\n\r");
      connectionLastAlive = millis();
      return true;
    } else {
      log_e("connection dead: update faillure");
      connected_central = false;
      return false;
    }
  } else {
    //printg("  OK\n\r");
    return true;
  }
}

// return state of the remote btn
bool remoteBtn() {
  if (connected_central) {
    return switchChar[0]->value();
  }
  return false;
}



//   ----------------------------
//   ---   Central handlers   ---
//   ----------------------------


// implemented in cupola.cpp, called if a switch is changed on the peripheral
//void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic);






//   -------------------------------------
//   ---   Private functions   ---
//   -------------------------------------


// return the characteristic that correspond to the uuid
BLECharacteristic* findCharacteristic(const char * uuid) {
  BLECharacteristic ret;
  ret = remote.characteristic(uuid);
  if (!ret) {
    log_e("unable to find characteristic %s", uuid);
  }
  return new BLECharacteristic(ret);

}
