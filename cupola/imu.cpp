#include "imu.h"

#include <Wire.h>
#include <Arduino.h>
#include "utility.h"
#include "cupola.h"
#include "io.h"
#include "ble.h"
#include "rf.h"

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
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0b11010000); // Temperature compensation enable, high perf, 10 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0x08); // high perf
  ledRGB(false, true, false);
  while (!magAvailable()) {}
  ledRGB(false, false, false);
  readMagConv(mag_raw);
  v_copy(mag_raw, mag_smooth);

}



// testIMUMag
// return true in case of success
bool testIMUMag() {
  unsigned long start = millis();
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


void stopIMU() {
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
}

bool magAvailable() {
  return readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08;
}



void readMag(int16_t data[]) {
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
}

void readMagConv(float res[]) {
  int16_t data[3];
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
  res[0] = data[0] * 400. / 32768.;
  res[1] = data[1] * 400. / 32768.;
  res[2] = data[2] * 400. / 32768.;

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
  ledRGB(false, true, false);
  if (!magAvailable()) {
    return;
  }
  ledRGB(false, false, false);

  readMagConv(mag_raw);
  writeMagRaw(mag_raw);
  

  // checking if the magnetic field has the expected norm, otherwise reject the value
  float norm = sqrt(mag_raw[0] * mag_raw[0] + mag_raw[1] * mag_raw[1] + mag_raw[2] * mag_raw[2]);
  //log_d("RAW: %f %f %f %f",mag_raw[0],mag_raw[1],mag_raw[2],norm);
  if (norm > MAG_INIT_MAX || norm < MAG_INIT_MIN) {
    ledRGB(true, true, false);
    return;
  }

  // filtering
  v_lincomb(1.0 - K_SMOOTH, mag_smooth, K_SMOOTH, mag_raw, mag_smooth);
  writeMagFilt(mag_smooth);

}
