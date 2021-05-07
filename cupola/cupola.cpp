#include "imu.h"
#include "utility.h"
#include "io.h"
#include "settings.h"
#include "vector.h"
#include "cupola.h"
#include "rf.h"
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
  v_print("st_compass_bias", st_compass_bias);
  v_print("st_compass_amp", st_compass_amp);
  m_print("st_compass_rot", (float*)st_compass_rot);
  log_d("st_compass_heading_bias=%f", st_compass_heading_bias);
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

  start_rf();

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
  ledRYG(false, false, false);
  ledRGB(false, false, false);
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

  // Cupola or mount calibration: DIP +X__ and push
  // ++__ for mount
  // +___ for cupola
  if (switch_1() && !switch_3() && !switch_4() && btn()) {
    // au premier appui initialiser
    // pour chaque appui: attendre 1s et capturer
    // au dernier appui, arreter et appeler la fonction pour la coupole ou la monture
    if (current_calib_sample == 0) {
      state = CALIB;
      initIMUMagAcc();
      writeState(state);
      log_i("Calibration mode");
      log_("Step\tMag_x  \tMag_y  \tMag_z  \tAcc_x  \tAcc_y  \tAcc_z  \t");
    }
    ledRGB(true, true, true);
    delay(CALIB_DELAY);
    ledRGB(false, false, false);
    sampleCalib(); // current_calib_sample is incremented
    log_d("%i\t%f\t%f\t%f\t%f\t%f\t%f\t", current_calib_sample, mag_raw[0], mag_raw[1], mag_raw[2], acc_filt[0], acc_filt[1], acc_filt[2]);

    if (current_calib_sample == CALIB_SAMPLES) {
      stopIMU();
      current_calib_sample = 0;
      state = connectedPeripheral() ? STANDBY : CONNECTION;
      writeState(state);
      if (switch_2()) {
        mountCalibCalc();
      } else {
        compassCalibCalc();
      }
    }
  }

  // Sensor log: +++_
  if (switch_1() && switch_2() && switch_3() && !switch_4() && btn() && state != LOG_SENSOR) {
    state = LOG_SENSOR;
    initIMUMagAcc();
    writeState(state);
    log_i("Sensor log mode");
    current_calib_sample = 0;
  } else if (state == LOG_SENSOR && btn()) {
    stopIMU();
    state = connectedPeripheral() ? STANDBY : CONNECTION;
    writeState(state);
    current_calib_sample = 0;
    ledRGB(false, false, false);
    delay(CALIB_DELAY);
  }

  // Cupola manual rotation: DIP +XX+ and push
  // ++_+ and push for right
  // +__+ and push for left
  // ++++ and push for up
  // +_++ and push for down
  if (switch_1()  && switch_4()) {
    if (btn()) {
      if (switch_2() && !switch_3())rf_command = RIGHT;
      if (!switch_2() && !switch_3())rf_command = LEFT;
      if (switch_2() && switch_3())rf_command = UP;
      if (!switch_2() && switch_3())rf_command = DOWN;
    } else {
      rf_command = NONE;
    }
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
    float mag_calib[3];
    compassCalib(mag_raw, mag_calib);
    float heading = DEG(atan2(mag_calib[1], mag_calib[0]));

    log_("%fdeg\t%f\t%f\t%f\t%f", heading, mag_calib[0], mag_calib[1], mag_calib[2], norm(mag_calib));
    delay(100);
  }
  if (state == CALIB) {
    ledRYG(false, true, true);
    ledRGB(false, false, true);
  }

  if (state == LOG_SENSOR) {
    ledRYG(false, true, true);
    ledRGB(false, false, true);

    readMagConv(mag_raw);
    readAccConv(acc_filt);

    log_("raw sample (acc;mag)");
    m_print("", acc_filt, 3, 1);
    m_print("", mag_raw, 3, 1);

    float mag_calibrated[3], acc_calibrated[3];
    mountCalib(mag_raw, (const float *)st_A_mag_inv, st_bias_mag, 1, mag_calibrated);
    mountCalib(acc_filt, (const float *)st_A_acc_inv, st_bias_acc, 1, acc_calibrated);

    log_("cal sample (acc;mag)");
    m_print("", acc_calibrated, 3, 1);
    m_print("", mag_calibrated, 3, 1);

    float ha_rot, dec_rot;
    float m_theo[3];
    normalize(st_ref_mag, m_theo);
    mountRot(mag_calibrated, acc_calibrated, RAD(st_lat), m_theo, st_sigma_mag, st_sigma_acc, &ha_rot, &dec_rot);

    log_("retrieved angles:");
    log_("ha_rot = %f", DEG(ha_rot));
    log_("dec_rot = %f", DEG(dec_rot));
    //log_("%f\t%f\t%f\t\t%f\t%f\t%f\t\t%f\t%f", mag_calibrated[0], mag_calibrated[1], mag_calibrated[2], acc_calibrated[0], acc_calibrated[1], acc_calibrated[2], DEG(ha_rot), DEG(dec_rot));
    delay(CALIB_DELAY);
  }


  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
  }
  delay(LOOP_PERIOD);
}



void loop_mount() {
  ledRYG(false, false, false);
  ledRGB(false, false, false);
  BLE.poll();

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

  // state operations
  if (state == CONNECTION) {
    ledRYG(true, false, true);
    connectBLECentral();
  }
  if (state == STANDBY) {
    ledRYG(false, false, true);
  }
  if (state == ON) {
    ledRYG(false, true, true);
  }

  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
  }
  delay(LOOP_PERIOD);
}

void loop_cupola() {
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
  ledRYG(false, false, false);
  ledRGB(false, false, false);

  // state operations
  if (state == CONNECTION) {
    ledRYG(true, false, true);
    connectBLEPeripheral();
  }
  if (state == STANDBY) {
    ledRYG(false, false, true);
    updateSwitches();
  }
  if (state == ON) {
    ledRYG(false, true, true);
    updateSwitches();
    updateMag();
  }

  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
    ledRGB(acc_error_flag, mag_error_flag, false);
    log_e("IMU error, acc_error_flag=%u mag_error_flag=%u", acc_error_flag, mag_error_flag);
  }
  delay(LOOP_PERIOD);
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
