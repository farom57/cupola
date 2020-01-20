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
float acc_filt[3];


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
  if (!testIMUMag() || !testIMUAcc()) {
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

  // --- mode transitions ---


  // --- mode operation ---
  if (mode == CONNECTION) {
    ledG(true);
    ledY(false);
    ledR(true);


    if (connectBLE()) {
      mode = STANDBY;
      return;
    }

    wait(1.);
  }

  if (mode >= STANDBY) {
    ledG(true);
    ledY(mode == ON);
    ledR(false);

    //central.poll();

    if (!isAlive()) {
      disconnectBLE(); // RESET !
      return;
    }
#ifdef CUPOLA
    updateSwitches();
    if (mode == ON) {
      updateMag();
    }
#else
    if (mode == ON) {
      readRemoteMag(mag_filt_remote);
      readMagConv(mag_filt);
      readAccConv(acc_filt);
      printg("Time = %ld\n\r", millis());
      v_print("mag_filt_remote", mag_filt_remote);
      v_print("mag_filt", mag_filt);
      v_print("acc_filt", acc_filt);
      printg("\n\r");
    }

    if (btn_chg() & !btn()) { // BTN released
      toggleMode();
      return;
    }
#endif

    wait(0.1);
  }


}

// Cupola

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


// Mount

void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t val;
  characteristic.readValue(val);
  if (!val) {
    printg("Remote button released\n\r");
    toggleMode();
  }

}

void toggleMode() {
  if (mode == STANDBY) {
    remoteModeON(true);
    initIMUMagAcc();
    mode = ON;
  } else if (mode == ON) {
    remoteModeON(false);
    stopIMU();
    mode = STANDBY;
  }
}
