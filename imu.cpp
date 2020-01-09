#include "imu.h"

#include <Wire.h>
#include "Arduino.h"
#include "utility.h"
#include "settings.h"
#include "cupola.h"
#include "vector.h"
#include "io.h"
#include "ble.h"

#define LSM9DS1_ADDRESS_M          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_CTRL_REG5_M        0x24
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28




void initIMUMag() {
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);

  delay(50);

  Wire1.begin();

  // reset
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

  delay(10);
  //writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0x80); // Temperature compensation enable, Low-power, 0.625 Hz
  //writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  //writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x20); // LP & Continuous conversion mode
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0xd2); // Temperature compensation enable, high perf, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG4_M, 0x08); // high perf

  while (!magAvailable()) {}
  readMagConv(mag_raw);
  v_copy(mag_raw, mag_smooth);
  v_copy(mag_raw, mag_filt);


}

// testIMUMag
// return true in case of success
bool testIMUMag() {
  long start = millis();
  while (millis() < start + 100) {
    if (magAvailable()) {
      float meas[3];
      float norm;
      readMagConv(meas);
      norm = sqrt(meas[0] * meas[0] + meas[1] * meas[1] + meas[2] * meas[2]);
      if (norm > MAG_FIELD_MAX || norm < MAG_FIELD_MIN) {
        printg("Mag field out of limits: %f %f %f (norm: %f)\n", meas[0], meas[1], meas[2], norm);
        return false;
      }
      return true;
    }
  }
  printg("Mag measurments not available");
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
  res[0] = constrain(data[0] * 400. / 32768., -99., 99.);
  res[1] = constrain(data[1] * 400. / 32768., -99., 99.);
  res[2] = constrain(data[2] * 400. / 32768., -99., 99.);
  //res[0]=data[0]*400./32768.;
  //res[1]=data[1]*400./32768.;
  //res[2]=data[2]*400./32768.;

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
    digitalWrite(LEDR, HIGH);
    errorCount = 0;
    readMagConv(mag_raw);
    writeMagRaw(mag_raw);
    //printg("RAW: %f %f %f\n",mag_raw[0],mag_raw[1],mag_raw[2]);
    
    // filtering
    v_lincomb(1.0 - K_SMOOTH, mag_smooth, K_SMOOTH, mag_raw, mag_smooth);
    printg("SMOOTH: %f %f %f\n",mag_smooth[0],mag_smooth[1],mag_smooth[2]);
        
    // change detection
    float diff[3];
    v_sub(mag_smooth, mag_filt, diff);
    printg("DIFF: %f %f %f (%f)\n",diff[0],diff[1],diff[2],norm(diff));
    if (norm(diff) > MAG_CHANGE_THRESHOLD) {
      v_copy(mag_smooth, mag_filt);
      writeMagFilt(mag_filt);
      //printg("FILT: %f %f %f\n",mag_filt[0],mag_filt[1],mag_filt[2]);
    }
    

  } else {
    errorCount++;
    digitalWrite(LEDR, LOW);
    if (errorCount > 5) {
      ledR(true);
      ledY(false);
      ledG(true);
      digitalWrite(LEDB, LOW);
      stopIMU();
      mode = INIT;
    }
  }
}
