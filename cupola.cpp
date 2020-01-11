#include "imu.h"
#include "utility.h"
#include "io.h"
#include "settings.h"
#include "vector.h"
#include "cupola.h"

#include "ble.h"






// Global variables

enum modes mode = INIT;
float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];
float mag_filt_remote[3];
float acc_raw[3];
float acc_smooth[3];
float acc_filt[3];
#ifdef CUPOLA
BLEDevice central;
#else
BLEDevice peripheral;
#endif

void setup() {

  initIO();

  ledR(true);

  Serial.begin(57600);

  // in debug mode wait btn to initialize
  if (switch_1()) {
    while (!btn()) {}
  }

  //IMU self test
#ifdef CUPOLA
  initIMUMag();
  if (!testIMUMag()) {
    Serial.println("Failed to initialize IMU!");
    digitalWrite(LEDG, LOW);
    while (1);
  }
  stopIMU();
  initBLEPeripherial();
  
#else
  initIMUMagAcc();
  if (!testIMUMagAcc()) {
    Serial.println("Failed to initialize IMU!");
    digitalWrite(LEDG, LOW);
    while (1);
  }
  stopIMU();
  initBLECentral();
  
#endif

  mode = CONNECTION;
  ledR(true);
  ledG(true);

}

void loop() {
  static long connectionLastAlive = 2147483647L;
  // --- mode transitions ---
  // transition from INIT to CONNECTION is managed in setup()
  if (mode == CONNECTION && central && central.connected()) {
    mode = STANDBY;
  }
  // TODO: transition to SLEEP
  if (mode == STANDBY && (!central.connected() || millis() > connectionLastAlive + CONNECTION_TIMEOUT)) {
    mode = CONNECTION;
    //TODO: force disconnect
  }
  // transitions between STANDBY and ON are managed in modeChangedHandler()

  // --- mode operation ---
  if (mode == CONNECTION) {
    ledG(true);
    ledY(false);
    ledR(true);

    central = BLE.central();
    if (central && central.connected()) {
      mode = STANDBY;
      return;
    }

    wait(1.);
  }

  if (mode == STANDBY) {
    ledG(true);
    ledY(false);
    ledR(false);

    if (!central.connected()) {
      mode = CONNECTION;
      return;
    }
    if (millis() > connectionLastAlive + CONNECTION_TIMEOUT) {
      mode = CONNECTION;
      BLE.disconnect();
      return;
    }

    updateSwitches();

    wait(0.1);
  }

  if (mode == ON) {
    ledG(true);
    ledY(true);
    ledR(false);

    if (!central.connected()) {
      mode = CONNECTION;
      return;
    }
    if (millis() > connectionLastAlive + CONNECTION_TIMEOUT) {
      mode = CONNECTION;
      BLE.disconnect();
      return;
    }

    updateMag();
    updateSwitches();


    wait(0.05);
  }

}

void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t requestedMode;
  characteristic.readValue(requestedMode);
  if (requestedMode >= ON) {
    // RED LED will stay ON in case of error
    ledR(true);
    ledG(false);
    ledY(false);

    initIMUMag();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);

    mode = ON;

  } else if (requestedMode <= STANDBY) {
    stopIMU();
    mode = STANDBY;

  }
}

void magReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  // if the continuous acquisition is OFF, start the IMU for a single measurment
  if (mode < ON) {
    initIMUMag();
    while (!magAvailable()) {}
    readMagConv(mag_raw);
    v_copy(mag_raw, mag_filt);
    stopIMU();
  } else {
    v_copy(mag_smooth, mag_filt);
  }

  writeMagRaw(mag_raw);
  writeMagFilt(mag_filt);

}
