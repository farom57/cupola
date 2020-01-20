#include "settings.h"
#ifdef MOUNT
#include "cupola.h"
#include "utility.h"
#include "ble.h"

long connectionLastAlive = -2147483647L;

BLEDevice peripheral;

BLECharacteristic batteryLevelChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031c0", BLERead | BLENotify, "Not implemented");

BLECharacteristic switch1Char; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d1", BLERead | BLENotify);
BLECharacteristic switch2Char; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d2", BLERead | BLENotify);
BLECharacteristic switch3Char; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d3", BLERead | BLENotify);
BLECharacteristic switch4Char; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d4", BLERead | BLENotify);
BLECharacteristic btnChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d5", BLERead | BLENotify);

BLECharacteristic modeONChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031e1", BLERead | BLEWrite);

BLECharacteristic magRawStringChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f1", BLERead, " xxx.xxxxx, xxx.xxxxx, xxx.xxxxx");
BLECharacteristic magRawXChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f2", BLERead | BLENotify);
BLECharacteristic magRawYChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f3", BLERead | BLENotify);
BLECharacteristic magRawZChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f4", BLERead | BLENotify);
BLECharacteristic magFiltStringChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f5", BLERead | BLENotify, " xxx.xxxxx, xxx.xxxxx, xxx.xxxxx");
BLECharacteristic magFiltXChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f6", BLERead | BLENotify);
BLECharacteristic magFiltYChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f7", BLERead | BLENotify);
BLECharacteristic magFiltZChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f8", BLERead | BLENotify);
BLECharacteristic aliveChar; // ("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031b1", BLERead | BLEWrite);
BLECharacteristic *magRawChar[3] = {&magRawXChar, &magRawYChar, &magRawZChar};
BLECharacteristic *magFiltChar[3] = {&magFiltXChar, &magFiltYChar, &magFiltZChar};

void initBLECentral() {
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  BLE.scanForName("Cupola");
  printg("Scanning ...\n\r");
}

bool connectBLE() {
  peripheral = BLE.available();
  if (peripheral) {
    printg("Found\n\r");
    BLE.stopScan();

    // connect to the peripheral
    if (peripheral.connect()) {
      printg("Connected\n\r");
    } else {
      printg("Failed to connect!\n\r");
      return false;
    }

    Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      printg("Attributes discovered\n\r");
    } else {
      printg("Attribute discovery failed!\n\r");
      disconnectBLE();
      return false;
    }


    batteryLevelChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031c1");
    switch1Char = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d1");
    switch2Char = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d2");
    switch3Char = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d3");
    switch4Char = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d4");
    btnChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d5");
    modeONChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031e1");
    magRawStringChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f1");
    magRawXChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f2");
    magRawYChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f3");
    magRawZChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f4");
    magFiltStringChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f5");
    magFiltXChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f6");
    magFiltYChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f7");
    magFiltZChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f8");
    aliveChar = findCharacteristic("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031b1");
    magRawChar[0] = &magRawXChar;
    magRawChar[1] = &magRawYChar;
    magRawChar[2] = &magRawZChar;
    magFiltChar[0] = &magFiltXChar;
    magFiltChar[1] = &magFiltYChar;
    magFiltChar[2] = &magFiltZChar;

    if (
      batteryLevelChar &&
      switch1Char && switch2Char && switch3Char && switch4Char && btnChar &&
      modeONChar &&
      magRawStringChar && magRawXChar && magRawYChar && magRawZChar &&
      magFiltStringChar && magFiltXChar && magFiltYChar && magFiltZChar &&
      aliveChar) {
      printg("Characteristics OK\n\r");
    } else {
      printg("Problem with haracteristics\n\r");
      disconnectBLE();
      return false;
    }

    if (!btnChar.subscribe()) {
      printg("Cannot subscribe to btnChar\n\r");
      disconnectBLE();
      return false;
    }
    btnChar.setEventHandler(BLEWritten, btnChangedHandler);
    aliveChar.read();
    aliveChar.writeValue(CONNECTION_KEEPALIVE_TIMEOUT2);
    connectionLastAlive = millis();
    printg("Connected\n\r");
    return true;
  }
  printg("not found\n\r");
  return false;
}

void disconnectBLE() {
  printg("Disconnected\n\r");
  peripheral.disconnect();
  printg("Rebooting\n\r");
  wait(0.1);
  system_reset();

}


BLECharacteristic findCharacteristic(const char * uuid) {
  BLECharacteristic ret;
  ret = peripheral.characteristic(uuid);
  if (!ret) {
    printg("unable to find characteristic %s\n\r", uuid);
  }
  return ret;

}

bool isAlive() {
  if (!BLE.connected()) {
    printg("isAlive(): not connected\n\r");
    return false;
  }
  //printg("isAlive(): millis=%ld connectionLastAlive=%ld\n\r", millis(), connectionLastAlive);
  if (millis() - connectionLastAlive > CONNECTION_KEEPALIVE_TIMEOUT) {

    if (aliveChar.read()) {
      //printg("  update OK\n\r");
      connectionLastAlive = millis();
      return true;
    } else {
      printg("  update FAILLURE\n\r");
      return false;
    }
  } else {
    //printg("  OK\n\r");
    return true;
  }
}

void remoteModeON(bool on) {
  modeONChar.writeValue((uint8_t)(on ? ON : STANDBY));
}

void readRemoteMag(float res[]) {
  for (int i = 0; i < 3; i++) {
    float tmp;
    magFiltChar[i]->readValue(&tmp, 4);
    //printg("magFiltChar[%d]=%f=%0h\n\r",i,tmp,tmp);
    res[i]=tmp;
  }
}

#endif
