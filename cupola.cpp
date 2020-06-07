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

bool old_local_btn_state = false;
bool old_remote_btn_state = false;




void setup() {

  initIO();

  if (switch_1()) {
    operating_mode = DEBUG;
    while (!btn()) {}
    if (switch_1() && !switch_2() && switch_3() && !switch_4()) {
      ledRGB(true, true, true);
      resetSt();
      ledRGB(false, false, false);
    }
  } else if (switch_2()) {
    operating_mode = CUPOLA;
  } else {
    operating_mode = MOUNT;
  }


  ledRYG(true, false, false);
  Serial.begin(57600);

  loadSt();

  log_i("Compilation date: " __DATE__ " " __TIME__);

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
  checkStWritten();

  // state transitions
  // Connection, state change directly to ON in debug mode
  if (state == CONNECTION && connectedPeripheral()) {
    state = STANDBY;
    writeState(state);
    log_i("Connection, state change to STANDBY");
  }
  // Disconnection
  if ((state == STANDBY || state == ON) && !connectedPeripheral()) {
    stopIMU();
    system_reset();
    log_e("Disconnection, System reset");
  }
  // Request received to change state to ON
  if (state == STANDBY && readState() == ON) {
    state = ON;
    initIMUMagAcc();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
    writeState(state);
    log_i("State change received, state change to ON");
  }
  // Request received to change state to STANDBY
  if (state == ON && readState() == STANDBY) {
    state = STANDBY;
    stopIMU();
    writeState(state);
    log_i("State change received, state change to STANDBY");
  }

  //start mag calibration test
  if (switch_1() && switch_2() && !switch_3() && !switch_4() && btn()) {
    state = COMPASS_CALIB;
    initIMUMagAcc();
    writeState(state);
    log_i("Compass calibration mode");
    log_("Mag_x  \tMag_y  \tMag_z  \tAcc_x  \tAcc_y  \tAcc_z  \t");
    current_calib_sample = 0;
  }

  if (current_calib_sample >= CALIB_SAMPLES) {
    stopIMU();
    state = connectedPeripheral() ? STANDBY : CONNECTION;
    writeState(state);
    compassCalibCalc();
    current_calib_sample = 0;
  }

  if(state==LOG_SENSOR && btn()){
    stopIMU();
    state = connectedPeripheral() ? STANDBY : CONNECTION;
    writeState(state);
    current_calib_sample = 0;
  }
  
  if (switch_1() && !switch_2() && !switch_3() && !switch_4() && btn()) {
    state = LOG_SENSOR;
    initIMUMagAcc();
    writeState(state);
    log_i("Sensor log mode");
    log_("Mag_x  \tMag_y  \tMag_z  \tAcc_x  \tAcc_y  \tAcc_z  \t");
    current_calib_sample = 0;
  }



  ledRYG(false, false, false);
  ledRGB(false, false, false);
  

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
    float mag_calib[3];
    compassCalib(mag_raw,mag_calib);
    float heading=atan2(mag_calib[1],mag_calib[0])*360./2./PI;
    
    log_("%fdeg\t%f\t%f\t%f\t%f",heading, mag_calib[0],mag_calib[1],mag_calib[2],norm(mag_calib)); 
    delay(100);
  }
  if (state == COMPASS_CALIB) {
    ledRYG(false, true, true);
    ledRGB(true, true, true);
    sampleCalib();
    log_("%7.3f\t%7.3f\t%7.3f\t%7.3f\t%7.3f\t%7.3f\t",mag_raw[0],mag_raw[1],mag_raw[2],acc_filt[0],acc_filt[1],acc_filt[2]);
    current_calib_sample++;
    delay(CALIB_PERIOD);
  }

  if (state == LOG_SENSOR) {
    ledRYG(false, true, true);
    ledRGB(true, true, true);
    sampleCalib();
    log_d("%f\t%f\t%f\t%f\t%f\t%f\t",mag_raw[0],mag_raw[1],mag_raw[2],acc_filt[0],acc_filt[1],acc_filt[2]);
    delay(CALIB_PERIOD);
  }


  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
  }
}



void loop_mount() {
  BLE.poll();
  checkStWritten();
  // state transitions
  // Connection, state change directly to ON in debug mode
  if (state == CONNECTION && connectedCentral()) {
    state = STANDBY;
    log_i("Connection, state change to STANDBY");
  }
  // Disconnection
  if (state > CONNECTION && !connectedCentral()) {
    stopIMU();
    system_reset();
    log_e("Disconnection, System reset");
  }
  //button pressed and released
  bool btn_changed = (old_local_btn_state && !btn()) || (old_remote_btn_state && !remoteBtn());
  old_local_btn_state = btn();
  old_remote_btn_state = remoteBtn();
  if (state == STANDBY && btn_changed) {
    state = ON;
    setRemoteState(ON);
    btn_changed = false;
    log_i("Button pressed, state change to ON");
  }
  if (state == ON && btn_changed) {
    state = STANDBY;
    setRemoteState(STANDBY);
    btn_changed = false;
    log_i("Button pressed, state change to STANDBY");
  }

  // Error
  if (acc_error_flag || mag_error_flag) {
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
  }

  // state operations
  if (state == CONNECTION) {
    ledRYG(true, false, true);
    connectBLECentral();
    delay(100);
  }
  if (state == STANDBY) {
    ledRYG(false, false, true);
    delay(100);
  }
  if (state == ON) {
    ledRYG(false, true, true);
    delay(100);
  }
}

void loop_cupola() {
  BLE.poll();

  // state transitions
  // Connection, state change directly to ON in debug mode
  if (state == CONNECTION && connectedPeripheral()) {
    state = STANDBY;
    writeState(state);
    log_i("Connection, state change to STANDBY");
  }
  // Disconnection
  if (state > CONNECTION && !connectedPeripheral()) {
    stopIMU();
    system_reset();
    log_e("Disconnection, System reset");
  }
  // Request received to change state to ON
  if (state == STANDBY && readState() == ON) {
    state = ON;
    initIMUMag();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
    writeState(state);
    log_i("State change received, state change to ON");
  }
  // Request received to change state to STANDBY
  if (state == ON && readState() == STANDBY) {
    state = STANDBY;
    stopIMU();
    writeState(state);
    log_i("State change received, state change to STANDBY");
  }
  // Error
  if (acc_error_flag || mag_error_flag) {
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
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
    delay(100);
  }

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
  //log_d("BLE mag read event");
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
//  if (state == STANDBY) {
//    setRemoteState(STANDBY);
//    initIMUMagAcc();
//    state = ON;
//  } else if (state == ON) {
//    setRemoteState(STANDBY);
//    stopIMU();
//    mode = STANDBY;
//  }
//}
