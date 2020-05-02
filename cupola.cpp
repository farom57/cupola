#include "imu.h"
#include "utility.h"
#include "io.h"
#include "settings.h"
#include "vector.h"
#include "cupola.h"

#include "ble.h"






// Global variables

enum states state = INIT;
enum operating_modes operating_mode;
bool acc_error_flag = false;
bool mag_error_flag = false;
float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];
float mag_filt_remote[3];
float acc_filt[3];


void setup() {

  initIO();

  if (switch_1()) {
    operating_mode = DEBUG;
    while (!btn()) {}
  } else if (switch_2()) {
    operating_mode = CUPOLA;
  } else {
    operating_mode = MOUNT;
  }

  
  ledRYG(true,false,false);
  Serial.begin(57600);
  
  

  if (operating_mode == CUPOLA) {
    //IMU test (mag only in CUPOLA mode)
    initIMUMag();
    if (!testIMUMag()) {
      log_e("Failed to initialize Magnetometer!");
      ledRGB(false, true, false);
      while (1);
    }
    stopIMU(); // stop the IMU, it is activated on demand to save power

    initBLEPeripheral();
    state = CONNECTION;
  }


  if (operating_mode == MOUNT) {
    // IMU test:
    initIMUMagAcc();
    if (!testIMUMag()) {
      log_e("Failed to initialize Magnetometer!");
      ledRGB(false, true, false);
      while (1);
    }
    if (!testIMUAcc()) {
      log_e("Failed to initialize Accelerometer!");
      ledRGB(true, false, false);
      while (1);
    }
    stopIMU(); // stop the IMU, it is activated on demand to save power

    initBLECentral();
    state = CONNECTION;
  }


  if (operating_mode == DEBUG) {
    // IMU test
    initIMUMagAcc();
    if (!testIMUMag()) {
      log_e("Failed to initialize Magnetometer in debug mode. Continue");
      ledRGB(false, true, false);
    }

    if (!testIMUAcc()) {
      log_e("Failed to initialize Accelerometer in debug mode. Continue");
      ledRGB(true, false, false);
    }
    stopIMU(); // stop the IMU, it is activated on demand to save power
    initBLEPeripheral();
    state = CONNECTION;

  }
  log_i("Initialisation done");

}


void loop() {
  if (operating_mode == MOUNT) {
    loop_mount();
  }
  if (operating_mode == CUPOLA) {
    loop_cupola();
  }
  if (operating_mode == DEBUG) {
    loop_debug();
  }
}

void loop_debug() {
  BLE.poll();
  
  // state transitions
  // Connection, state change directly to ON in debug mode
  if (state == CONNECTION && connectedPeripheral()) {
    state = STANDBY;
    //writeState(state);
  }
  // Disconnection
  if (state > CONNECTION && !connectedPeripheral()) {
    state = CONNECTION;
    writeState(state);
  }
  // Request received to change state to ON
  if (state == STANDBY && readState() == ON) {
    state = ON;
    initIMUMag();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
    writeState(state);
  }
  // Request received to change state to STANDBY
  if (state == ON && readState() == STANDBY) {
    state = STANDBY;
    stopIMU();
    writeState(state);
  }
  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
    ledRGB(acc_error_flag, mag_error_flag, false);
  }

  // state operations
  if (state == CONNECTION) {
    ledRYG(true, false, true);
    connectBLEPeripheral();
    delay(100);
  }
  if (state == STANDBY) {
    ledRYG(false, false, true);
    updateSwitches();
    delay(100);
  }
  if (state == ON) {
    ledRYG(false, true, true);
    updateSwitches();
    updateMag();
    updateAcc();
    delay(100);
  }

}

void loop_mount() {
  //  if (mode == CONNECTION) {
  //    ledRYG(true, false, true);
  //    if (connectBLECentral()) {
  //      mode = STANDBY;
  //    } else {
  //      wait(1.);
  //    }
  //  }
  //TODO
}

void loop_cupola() {

  //  // state transitions
  //  if (state == CONNECTION && peripheralConnected()) {
  //    state = STANDBY;
  //    writeState(state);
  //  }
  //
  //  if (state >= STANDBY && !peripheralConnected()) {
  //    state = CONNECTION;
  //    writeState(state);
  //  }
  //
  //  if (state == STANDBY && readState() == ON) {
  //    state = ON;
  //    initIMUMag();
  //    writeMagRaw(mag_raw);
  //    writeMagFilt(mag_filt);
  //    writeState(state);
  //  }
  //
  //  if (state == ON && readState() == STANDBY) {
  //    state = STANDBY;
  //    stopIMU();
  //    writeState(state);
  //  }
  //
  //  // state operations
  //  if (state == CONNECTION) {
  //    ledRYG(true, true, true);
  //    connectBLEPeripheral();
  //  }
  //
  //    if (state == STANDBY) {
  //    ledRYG(false, false, true);
  //    updateSwitches();
  //  }
  //
  //  if (state == ON) {
  //    ledRYG(false, true, true);
  //    updateSwitches();
  //    updateMag();
  //  }

}




//if (mode >= STANDBY) {
//  ledG(true);
//  ledY(mode == ON);
//  ledR(false);
//
//  //central.poll();
//
//  if (!isAlive()) {
//    disconnectBLE(); // RESET !
//    return;
//  }
//#ifdef CUPOLA
//  updateSwitches();
//  if (mode == ON) {
//    updateMag();
//  }
//#else
//  if (mode == ON) {
//    readRemoteMagFilt(mag_filt_remote);
//    readMagConv(mag_filt);
//    readAccConv(acc_filt);
//    printg("Time = %ld\n\r", millis());
//    v_print("mag_filt_remote", mag_filt_remote);
//    v_print("mag_filt", mag_filt);
//    v_print("acc_filt", acc_filt);
//    printg("\n\r");
//  }
//
//  if (btn_chg() & !btn()) { // BTN released
//    toggleMode();
//    return;
//  }
//#endif
//
//  wait(0.1);
//}
//
//
//}

// Cupola

//void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
//  uint8_t requestedMode;
//  characteristic.readValue(requestedMode);
//  if (requestedMode >= ON) {
//    // RED LED will stay ON in case of error
//    ledR(true);
//    ledG(false);
//    ledY(false);
//
//    initIMUMag();
//    writeMagRaw(mag_raw);
//    writeMagFilt(mag_filt);
//
//    mode = ON;
//
//  } else if (requestedMode <= STANDBY) {
//    stopIMU();
//    mode = STANDBY;
//
//  }
//}

void magReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  log_d("BLE mag read event");
  // if the continuous acquisition is OFF, start the IMU for a single measurment
  if (state == STANDBY) {
    initIMUMag();
    while (!magAvailable()) {}
    readMagConv(mag_raw);
    v_copy(mag_raw, mag_filt);
    stopIMU();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
  }
  if (state == ON) {
    v_copy(mag_smooth, mag_filt);
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
  }



}


// Mount

//void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
//  uint8_t val;
//  characteristic.readValue(val);
//  if (!val) {
//    printg("Remote button released\n\r");
//    toggleMode();
//  }
//
//}

//void toggleMode() {
//  if (mode == STANDBY) {
//    setRemoteModeON(true);
//    initIMUMagAcc();
//    mode = ON;
//  } else if (mode == ON) {
//    setRemoteModeON(false);
//    stopIMU();
//    mode = STANDBY;
//  }
//}
