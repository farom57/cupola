// define CUPOLA or MOUNT depending on the module
#define CUPOLA
//#define MOUNT



// Includes

#include <ArduinoBLE.h>

#include "mbed.h"
#include "imu.h"
#include "utility.h"
#include "io.h"
#include "settings.h"

//#include "ble.h"

enum modes {INIT, SLEEP, ADVERTISE, STANDBY, ON} mode;


// Variables

float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];

// BLE Services
BLEService batteryService("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031c0");
BLEService switchService("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d0");
BLEService modeService("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031e0");
BLEService magService("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f0");

// BLE Characteristic
BLECharacteristic batteryLevelChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031c0", BLERead | BLENotify, "Not implemented");
BLEDescriptor batteryLevelDescr("2901", "Battery voltage");

BLEBoolCharacteristic switch1Char("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d1", BLERead | BLENotify);
BLEBoolCharacteristic switch2Char("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d2", BLERead | BLENotify);
BLEBoolCharacteristic switch3Char("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d3", BLERead | BLENotify);
BLEBoolCharacteristic switch4Char("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d4", BLERead | BLENotify);
BLEBoolCharacteristic btnChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031d5", BLERead | BLENotify);
BLEDescriptor switch1Descr("2901", "Switch 1");
BLEDescriptor switch2Descr("2901", "Switch 2");
BLEDescriptor switch3Descr("2901", "Switch 3");
BLEDescriptor switch4Descr("2901", "Switch 4");
BLEDescriptor btnDescr("2901", "Button");

BLEBoolCharacteristic modeONChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031e1", BLERead | BLEWrite);
BLEDescriptor modeONDescr("2901", "Mode ON");

BLECharacteristic magRawStringChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f1", BLERead, " x.xxxxx, x.xxxxx, x.xxxxx");
BLEDescriptor magRawStringDescr("2901", "Mag raw X,Y,Z");
BLEFloatCharacteristic magRawXChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f2", BLERead | BLENotify);
BLEDescriptor magRawXDescr("2901", "Mag raw X");
BLEFloatCharacteristic magRawYChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f3", BLERead | BLENotify);
BLEDescriptor magRawYDescr("2901", "Mag raw Y");
BLEFloatCharacteristic magRawZChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f4", BLERead | BLENotify);
BLEDescriptor magRawZDescr("2901", "Mag raw Z");
BLECharacteristic magFiltStringChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f5", BLERead | BLENotify, "x.xxxxx,x.xxxxx,x.xxxxx");
BLEDescriptor magFiltStringDescr("2901", "Mag filt X,Y,Z");
BLEFloatCharacteristic magFiltXChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f6", BLERead | BLENotify);
BLEDescriptor magFiltXDescr("2901", "Mag filt X");
BLEFloatCharacteristic magFiltYChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f7", BLERead | BLENotify);
BLEDescriptor magFiltYDescr("2901", "Mag filt Y");
BLEFloatCharacteristic magFiltZChar("c3fe2f77-e1c8-4b1c-a0f3-ef88d05031f8", BLERead | BLENotify);
BLEDescriptor magFiltZDescr("2901", "Mag filt Z");
BLEFloatCharacteristic *magRawChar[3] = {&magRawXChar, &magRawYChar, &magRawZChar};
BLEFloatCharacteristic *magFiltChar[3] = {&magFiltXChar, &magFiltYChar, &magFiltZChar};




void setup() {

  initIO();

  led_red(true);

  Serial.begin(9600);

  // in debug mode wait btn to initialize
  if(switch_1()){
    while(!btn()){}
  }
  
  //IMU self test
  initIMUMag();
  if (!testIMUMag()) {
    Serial.println("Failed to initialize IMU!");
    digitalWrite(LEDG, LOW);
    while (1);
  }
  stopIMU();

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    digitalWrite(LEDB, LOW);
    while (1);
  }

  BLE.setLocalName("Cupola");
  BLE.setAdvertisedService(batteryService);
  BLE.setAdvertisedService(switchService);
  BLE.setAdvertisedService(modeService);
  BLE.setAdvertisedService(magService);

  batteryService.addCharacteristic(batteryLevelChar);
  batteryLevelChar.addDescriptor(batteryLevelDescr);

  switchService.addCharacteristic(switch1Char);
  switchService.addCharacteristic(switch2Char);
  switchService.addCharacteristic(switch3Char);
  switchService.addCharacteristic(switch4Char);
  switchService.addCharacteristic(btnChar);
  switch1Char.addDescriptor(switch1Descr);
  switch2Char.addDescriptor(switch2Descr);
  switch3Char.addDescriptor(switch3Descr);
  switch4Char.addDescriptor(switch4Descr);
  btnChar.addDescriptor(btnDescr);
  switch1Char.writeValue(switch_1());
  switch2Char.writeValue(switch_2());
  switch3Char.writeValue(switch_3());
  switch4Char.writeValue(switch_4());
  btnChar.writeValue(btn());

  modeService.addCharacteristic(modeONChar);
  modeONChar.addDescriptor(modeONDescr);
  modeONChar.writeValue(false);

  magService.addCharacteristic(magRawStringChar);
  magService.addCharacteristic(magRawXChar);
  magService.addCharacteristic(magRawYChar);
  magService.addCharacteristic(magRawZChar);
  magService.addCharacteristic(magFiltStringChar);
  magService.addCharacteristic(magFiltXChar);
  magService.addCharacteristic(magFiltYChar);
  magService.addCharacteristic(magFiltZChar);
  magRawStringChar.addDescriptor(magRawStringDescr);
  magRawXChar.addDescriptor(magRawXDescr);
  magRawYChar.addDescriptor(magRawYDescr);
  magRawZChar.addDescriptor(magRawZDescr);
  magFiltStringChar.addDescriptor(magFiltStringDescr);
  magFiltXChar.addDescriptor(magFiltXDescr);
  magFiltYChar.addDescriptor(magFiltYDescr);
  magFiltZChar.addDescriptor(magFiltZDescr);
  magRawXChar.writeValue(0);
  magRawYChar.writeValue(0);
  magRawZChar.writeValue(0);
  magFiltXChar.writeValue(0);
  magFiltYChar.writeValue(0);
  magFiltZChar.writeValue(0);

  BLE.addService(batteryService);
  BLE.addService(switchService);
  BLE.addService(modeService);
  BLE.addService(magService);

  //BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  //BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  modeONChar.setEventHandler(BLEWritten, modeChangedHandler);
  magRawStringChar.setEventHandler(BLERead, magReadHandler);
  magRawXChar.setEventHandler(BLERead, magReadHandler);
  magRawYChar.setEventHandler(BLERead, magReadHandler);
  magRawZChar.setEventHandler(BLERead, magReadHandler);

  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  /*if (!loadSettings()) {
    defaultSettings();
    }

    initBLESettings();*/

  mode = ADVERTISE;
  led_red(true);
  led_green(true);

}


void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    led_red(false);
    while (central.connected()) {
      if (switch_1_chg()) {
        switch1Char.writeValue(switch_1());
      }
      if (switch_2_chg()) {
        switch2Char.writeValue(switch_2());
      }
      if (switch_3_chg()) {
        switch3Char.writeValue(switch_3());
      }
      if (switch_4_chg()) {
        switch4Char.writeValue(switch_4());
      }
      if (btn_chg()) {
        btnChar.writeValue(btn());
      }

    }

  }
  led_red(true);
}

void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
  if (modeONChar.value()) {
    mode = ON;
    digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
    led_red(true);
    led_green(false);
    led_yellow(false);
    digitalWrite(LEDG, LOW);
    initIMUMag();
    digitalWrite(LEDG, HIGH);
    while (!magAvailable()) {}
    readMagConv(mag_raw);
    for (int i = 0; i < 3; i++) {
      constrain(mag_raw[i], -99., 99.);
      mag_smooth[i] = mag_raw[i];
      mag_filt[i] = mag_raw[i];
      magRawChar[i]->writeValue(mag_raw[i]);
      magFiltChar[i]->writeValue(mag_raw[i]);
    }
    char buf[27];
    snprintf(buf, 27, "%8.4f,%8.4f,%8.4f", mag_raw[0], mag_raw[1], mag_raw[2]);
    magRawStringChar.writeValue(buf);
    magFiltStringChar.writeValue(buf);
    led_red(false);
    led_green(true);
    led_yellow(true);
  }
}

void magReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  while (!magAvailable()) {}
  readMagConv(mag_raw);
  for (int i = 0; i < 3; i++) {
    constrain(mag_raw[i], -99., 99.);
    mag_smooth[i] = mag_raw[i];
    mag_filt[i] = mag_raw[i];
    magRawChar[i]->writeValue(mag_raw[i]);
    magFiltChar[i]->writeValue(mag_raw[i]);
  }
  char buf[27];
  snprintf(buf, 27, "%8.4f,%8.4f,%8.4f", mag_raw[0], mag_raw[1], mag_raw[2]);
  magRawStringChar.writeValue(buf);
  magFiltStringChar.writeValue(buf);
  Serial.println(buf);
}
