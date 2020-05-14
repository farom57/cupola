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
bool old_local_btn_state = false;
bool old_remote_btn_state = false;
float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];
float mag_filt_remote[3];
float acc_filt[3];

#define N_SAMPLE 36
float test_calib_mag[3][N_SAMPLE];
float test_calib_acc[3][N_SAMPLE];
int current_sample = 0;

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
  if (state > CONNECTION && !connectedPeripheral()) {
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
  if (switch_2() && btn()) {
    state = TEST;
    initIMUMagAcc();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);
    writeState(state);
    log_i("Test calibration mode");
    current_sample = 0;
  }
  if (current_sample >= N_SAMPLE) {
    state = STANDBY;
    stopIMU();
    writeState(state);
    test_calib();
    current_sample = 0;
  }
  // Error
  if (acc_error_flag || mag_error_flag) {
    //in debug mode, error are displayed with the led but ignored
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
    updateAcc();
    delay(100);
  }
  if (state == TEST) {
    ledRYG(false, true, true);
    ledRGB(true, true, true);
    //updateSwitches();
    updateMag();
    updateAcc();
    test_calib_mag[0][current_sample] = mag_raw[0];
    test_calib_mag[1][current_sample] = mag_raw[1];
    test_calib_mag[2][current_sample] = mag_raw[2];
    test_calib_acc[0][current_sample] = acc_filt[0];
    test_calib_acc[1][current_sample] = acc_filt[1];
    test_calib_acc[2][current_sample] = acc_filt[2];
    v_print("mag: ", mag_raw);
    v_print("acc: ", acc_filt);
    current_sample++;
    delay(900);
    ledRGB(false, false, true);
    delay(100);
  }

}

void test_calib() {
  float acc_mean[3] = {0};
  float X[3] = {0};
  float Y[3] = {0};
  float Z[3] = {0};
  float dir[3] = {0};
  float XYZt[3][3] = {0};
  float M_XYZ[3][N_SAMPLE] = {0};
  m_print("acc: ", (float*)test_calib_acc, 3, N_SAMPLE);
  for (int i = 0; i < N_SAMPLE; i++) {
    acc_mean[0] += test_calib_acc[0][i];
    acc_mean[1] += test_calib_acc[1][i];
    acc_mean[2] += test_calib_acc[2][i];
  }
  acc_mean[0] /= N_SAMPLE;
  acc_mean[1] /= N_SAMPLE;
  acc_mean[2] /= N_SAMPLE;
  v_print("acc_mean: ", acc_mean);
  sv_mult(1. / norm(acc_mean), acc_mean, Z);
  v_print("Z: ", Z);
  dir[abs(Z[0]) > abs(Z[1]) ? 1 : 0] = 1.;
  v_print("dir: ", dir);
  vect_prod(Z, dir, Y);
  normalize(Y, Y);
  v_print("Y: ", Y);
  vect_prod(Y, Z, X);
  v_print("X: ", X);
  for (int i = 0; i < N_SAMPLE; i++) {
    XYZt[0][i] = X[i];
    XYZt[1][i] = Y[i];
    XYZt[2][i] = Z[i];
  }
  m_print("mag: ", (float*)test_calib_mag, 3, N_SAMPLE);
  mm_mult((float*)XYZt, (float*)test_calib_mag, (float*)M_XYZ, 3, 3, N_SAMPLE);
  m_print("M_XYZ: ", (float*)M_XYZ, 3, N_SAMPLE);
  float A[N_SAMPLE][4];
  for (int i = 0; i < N_SAMPLE; i++) {
    A[i][0] = M_XYZ[0][i] * M_XYZ[0][i];
    A[i][1] = M_XYZ[1][i] * M_XYZ[1][i];
    A[i][2] = M_XYZ[0][i];
    A[i][3] = M_XYZ[1][i];
  }

  // solving A.(a;b;c;d)=(1;...;1)
  // pseudo-solution is beta=(A' * A)^-1 * A' * (1;...;1)
  // B = A' * A
  float B[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      B[i][j] = 0;
      for (int k = 0; k < N_SAMPLE; k++) {
        B[i][j] += A[k][i] * A[k][j] ;
      }
    }
  }
  m_print("B: ",(float*)B,4,4);
  float Binv[4][4];
  inv((float*)B, (float*)Binv, 4);
  // finally beta = Binv * A' * (1;...;1) = sum along the row of Binv * A'
  float beta[4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < N_SAMPLE; j++) {
      for (int k = 0; k < 4; k++) {
        beta[i] += Binv[i][k] * A[j][k];
      }
    }
  }
  m_print("beta: ", beta, 4,1);
  float Cx, Cy, Cx0, Cy0,Cz;
  Cx0=-beta[2]/beta[0]/2.;
  Cy0=-beta[3]/beta[1]/2.;
  Cx=1./sqrt(beta[0]/(1.+Cx0*Cx0*beta[0]+Cy0*Cy0*beta[1]));
  Cy=Cx*beta[1]/beta[0];

  log_("Bias:\t%f\t%f",Cx0,Cy0);
  log_("Horz field:\t%f\t%f",Cx,Cy);
  Cz=0;
  for(int i=0;i<N_SAMPLE;i++){
    Cz+=M_XYZ[2][i];
  }
  Cz/=N_SAMPLE;
  log_("Vert field:\t%f",Cz);
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
