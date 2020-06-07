#include "imu.h"

#include <Wire.h>
#include "Arduino.h"
#include "utility.h"
#include "settings.h"
#include "cupola.h"
#include "vector.h"
#include "io.h"
#include "ble.h"

#define LSM9DS1_ADDRESS            0x6b
#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG7_XL       0x21
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28
#define LSM9DS1_OUT_TEMP_L         0x15

#define LSM9DS1_ADDRESS_M          0x1e
#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_CTRL_REG5_M        0x24
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];
float mag_filt_remote[3];
float acc_filt[3];
bool acc_error_flag = false;
bool mag_error_flag = false;


float calib_mag[3][CALIB_SAMPLES];
float calib_acc[3][CALIB_SAMPLES];
int current_calib_sample = 0;


void initIMUMag() {
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);

  delay(50);

  Wire1.begin();

  // reset
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);
  delay(10);

  // Mag
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0xd2); // Temperature compensation enable, high perf, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0x08); // high perf
  ledRGB(false, true, false);
  while (!magAvailable()) {}
  ledRGB(false, false, false);
  readMagConv(mag_raw);
  v_copy(mag_raw, mag_smooth);
  v_copy(mag_raw, mag_filt);

}

void initIMUMagAcc() {
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);

  delay(50);

  Wire1.begin();

  // reset
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);
  delay(10);
  // Acc
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0b1000000); // 238Hz, +/-2g
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG7_XL, 0b1010000); // LP cutoff: 238/100 = 2.38Hz


  // Mag
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0xd2); // Temperature compensation enable, high perf, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0x08); // high perf
  ledRGB(false, true, false);
  while (!magAvailable()) {}
  ledRGB(false, false, false);
  readMagConv(mag_raw);
  v_copy(mag_raw, mag_smooth);
  v_copy(mag_raw, mag_filt);
  ledRGB(true, false, false);
  while (!accAvailable()) {}
  ledRGB(false, false, false);
  readAccConv(acc_filt);
}


// testIMUMag
// return true in case of success
bool testIMUMag() {
  long start = millis();
  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I) != 0x3d) {
    log_e("LSM9DS1_WHO_AM_I!=0x3d");
    log_e("LSM9DS1_WHO_AM_I=%x", readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I));
    return false;
  }
  while (millis() < start + 100) {
    if (magAvailable()) {
      float meas[3];
      float norm;
      readMagConv(meas);
      norm = sqrt(meas[0] * meas[0] + meas[1] * meas[1] + meas[2] * meas[2]);
      if (norm > MAG_INIT_MAX || norm < MAG_INIT_MIN) {
        log_e("Mag field out of limits: %f %f %f (norm: %f)", meas[0], meas[1], meas[2], norm);
        return false;
      }
      return true;
    }
  }
  log_e("Mag measurments not available");
  return false;
}

bool testIMUAcc() {
  long start = millis();
  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) {
    log_e("LSM9DS1_WHO_AM_I!=0x68");
    log_e("LSM9DS1_WHO_AM_I=%x", readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I));
    return false;
  }
  while (millis() < start + 100) {
    if (accAvailable()) {
      float meas[3];
      float norm;
      readAccConv(meas);
      norm = sqrt(meas[0] * meas[0] + meas[1] * meas[1] + meas[2] * meas[2]);
      if (norm > ACC_INIT_MAX || norm < ACC_INIT_MIN) {
        log_e("Gravity field out of limits: %f %f %f (norm: %f)", meas[0], meas[1], meas[2], norm);
        return false;
      }
      return true;
    }
  }
  log_e("Acc measurments not available");
  return false;
}

void stopIMU() {
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
}

bool magAvailable() {
  return readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08;
}

bool accAvailable() {
  return readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01;
}



void readMag(int16_t data[]) {
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
}

void readMagConv(float res[]) {
  int16_t data[3];
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
  //res[0] = constrain(data[0] * 400. / 32768., -99., 99.);
  //res[1] = constrain(data[1] * 400. / 32768., -99., 99.);
  //res[2] = constrain(data[2] * 400. / 32768., -99., 99.);
  res[0] = data[0] * 400. / 32768.;
  res[1] = data[1] * 400. / 32768.;
  res[2] = data[2] * 400. / 32768.;

}

void readAcc(int16_t data[]) {
  readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, 6);
}

void readAccConv(float res[]) {
  int16_t data[3];
  readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, 6);
  res[0] = data[0] * 2. / 32768.;
  res[1] = data[1] * -2. / 32768.; //acc Y axis seems to be inverted on the board
  res[2] = data[2] * 2. / 32768.;
}



int readRegister(uint8_t slaveAddress, uint8_t address)
{
  Wire1.beginTransmission(slaveAddress);
  Wire1.write(address);
  if (Wire1.endTransmission() != 0) {
    return -1;
  }

  if (Wire1.requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return Wire1.read();
}

int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length)
{
  Wire1.beginTransmission(slaveAddress);
  Wire1.write(0x80 | address);
  if (Wire1.endTransmission(false) != 0) {
    return -1;
  }

  if (Wire1.requestFrom(slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = Wire1.read();
  }

  return 1;
}

int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
  Wire1.beginTransmission(slaveAddress);
  Wire1.write(address);
  Wire1.write(value);
  if (Wire1.endTransmission() != 0) {
    return 0;
  }

  return 1;
}

void updateMag() {
  static int errorCount = 0;
  if (magAvailable()) {
    errorCount = 0;
    readMagConv(mag_raw);
    writeMagRaw(mag_raw);
    //log_d("RAW: %f %f %f",mag_raw[0],mag_raw[1],mag_raw[2]);

    // filtering
    v_lincomb(1.0 - K_SMOOTH, mag_smooth, K_SMOOTH, mag_raw, mag_smooth);
    //log_d("SMOOTH: %f %f %f", mag_smooth[0], mag_smooth[1], mag_smooth[2]);

    // change detection
    float diff[3];
    v_sub(mag_smooth, mag_filt, diff);
    //log_d("DIFF: %f %f %f (%f)", diff[0], diff[1], diff[2], norm(diff));
    if (norm(diff) > MAG_CHANGE_THRESHOLD) {
      v_copy(mag_smooth, mag_filt);
      writeMagFilt(mag_filt);
      //log_d("MAG FILT: %f %f %f", mag_filt[0], mag_filt[1], mag_filt[2]);
    }


  } else {
    errorCount++;

    if (errorCount > 5) {
      //stopIMU();
      mag_error_flag = true;
    }
  }
}

void updateAcc() {
  static int errorCount = 0;
  if (accAvailable()) {
    errorCount = 0;
    readAccConv(acc_filt); // filtering is done inside the sensor
    writeAcc(acc_filt);
    acc_error_flag = false;
  } else {
    errorCount++;

    if (errorCount > 5) {
      //stopIMU();
      acc_error_flag = true;
    }
  }

}
// Acquire and save calibration sample
void sampleCalib() {
  int errorCount = 0;

  while (!(magAvailable()&accAvailable()) && errorCount < 5) {
    errorCount++;
    delay(100);
  }

  if (errorCount >= 5) {
    acc_error_flag = !accAvailable();
    acc_error_flag = !magAvailable();
    current_calib_sample--;
    log_e("IMU error %s:%d", __FILE__, __LINE__);
    return;
  }

  readMagConv(mag_raw);
  readAccConv(acc_filt);
  calib_mag[0][current_calib_sample] = mag_raw[0];
  calib_mag[1][current_calib_sample] = mag_raw[1];
  calib_mag[2][current_calib_sample] = mag_raw[2];
  calib_acc[0][current_calib_sample] = acc_filt[0];
  calib_acc[1][current_calib_sample] = acc_filt[1];
  calib_acc[2][current_calib_sample] = acc_filt[2];
}

// Compute compass calibration parameters
void compassCalibCalc() {
  float acc_mean[3] = {0};
  float X[3] = {0};
  float Y[3] = {0};
  float Z[3] = {0};
  float dir[3] = {0};
  float XYZt[3][3] = {0};
  float M_XYZ[3][CALIB_SAMPLES] = {0};

  // Compute vertical direction that is also the rotation axis
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    acc_mean[0] += calib_acc[0][i];
    acc_mean[1] += calib_acc[1][i];
    acc_mean[2] += calib_acc[2][i];
  }
  acc_mean[0] /= CALIB_SAMPLES;
  acc_mean[1] /= CALIB_SAMPLES;
  acc_mean[2] /= CALIB_SAMPLES;
  v_print("acc_mean: ", acc_mean);

  // Trasform the mag measurment in referential XYZ where Z is the rotation axis
  sv_mult(1. / norm(acc_mean), acc_mean, Z);
  v_print("Z: ", Z);
  dir[abs(Z[0]) < 0.5 ? 0 : 1] = 1.;
  v_print("dir: ", dir);
  vect_prod(Z, dir, Y);
  normalize(Y, Y);
  v_print("Y: ", Y);
  vect_prod(Y, Z, X);
  v_print("X: ", X);
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    XYZt[i][0] = X[i];
    XYZt[i][1] = Y[i];
    XYZt[i][2] = Z[i];
  }

  mm_mult((float*)XYZt, (float*)calib_mag, (float*)M_XYZ, 3, 3, CALIB_SAMPLES);

  // Search bias_X, bias_Y, amp_X and amp_Y such as ((M_X-bias_X)/amp_X)² + ((M_Y-bias_Y)/amp_Y)² = 1

  // Firt solve a*M_X² + b*M_Y² + c*M_X + d*M_Y = 1 for all the samples
  // Matricialy A*beta=ONES with A=[M_X² M_Y² M_X M_Y] beta=[a;b;c;d] ONES=[1;1;1;1;...]
  // optimal solution is beta = Pinv(A)*ONES=(A' * A)^-1 * A' * [1;1;1;1;...]

  // Build A
  float A[CALIB_SAMPLES][4];
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    A[i][0] = M_XYZ[0][i] * M_XYZ[0][i];
    A[i][1] = M_XYZ[1][i] * M_XYZ[1][i];
    A[i][2] = M_XYZ[0][i];
    A[i][3] = M_XYZ[1][i];
  }


  // Compute pseudo-solution beta=(A' * A)^-1 * A' * (1;...;1)
  // B = A' * A
  float B[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      B[i][j] = 0;
      for (int k = 0; k < CALIB_SAMPLES; k++) {
        B[i][j] += A[k][i] * A[k][j] ;
      }
    }
  }

  // inverse B
  float Binv[4][4];
  inv((float*)B, (float*)Binv, 4);

  // finally beta = Binv * A' * (1;...;1) = sum along the row of Binv * A'
  float beta[4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < CALIB_SAMPLES; j++) {
      for (int k = 0; k < 4; k++) {
        beta[i] += Binv[i][k] * A[j][k];
      }
    }
  }

  // compute bias_X, bias_Y, amp_X and amp_Y from beta
  m_print("beta: ", beta, 4, 1);
  float amp_X, amp_Y, amp_Z, bias_X, bias_Y;
  bias_X = -beta[2] / beta[0] / 2.;
  bias_Y = -beta[3] / beta[1] / 2.;
  amp_X = 1. / sqrt(beta[0] / (1. + bias_X * bias_X * beta[0] + bias_Y * bias_Y * beta[1]));
  amp_Y = amp_X * beta[1] / beta[0];

  log_("Bias:\t%f\t%f", bias_X, bias_Y);

  amp_Z = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    amp_Z += M_XYZ[2][i];
  }
  amp_Z /= CALIB_SAMPLES;
  log_d("Vert field:\t%f", amp_Z);
  log_d("Horizontal field:\t%f\t%f", amp_X, amp_Y);

  // check_validity: horizontal field should be around 23.7 for Cannes, France
  if (amp_X > 10 && amp_X < 70 && amp_Y > 10 && amp_Y < 70 && amp_X / amp_Y < 1.1 && amp_X / amp_Y > 0.9) {
    st_compass_bias_x = bias_X;
    st_compass_bias_y = bias_Y;
    st_compass_amp_x = amp_X;
    st_compass_amp_y = amp_Y;
    st_compass_amp_z = amp_Z;
    m_copy((float*)XYZt, (float*)st_compass_rot);
    saveCompassCalib();
  } else {
    log_e("Invalid compass calibration parameters %s:%d", __FILE__, __LINE__);
  }
}

void compassCalib(float in[], float res[]){
  mv_mult((float*)st_compass_rot, in, res);
  res[0]=(res[0]-st_compass_bias_x)/st_compass_amp_x;
  res[1]=(res[1]-st_compass_bias_y)/st_compass_amp_y;
  res[2]=res[2]/st_compass_amp_z;
}
