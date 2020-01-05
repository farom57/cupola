#include "imu.h"

#include <Wire.h>
#include "Arduino.h"
#include "utility.h"
#include "settings.h"


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
}

// testIMUMag
// return true in case of success
bool testIMUMag(){
  long start = millis();
  while(millis() < start + 100){
    if(magAvailable()){
      float meas[3];
      float norm;
      readMagConv(meas);
      norm = sqrt(meas[0]*meas[0]+meas[1]*meas[1]+meas[2]*meas[2]);
      if(norm>MAG_FIELD_MAX || norm<MAG_FIELD_MIN){
        printg("Mag field out of limits: %f %f %f (norm: %f)\n",meas[0],meas[1],meas[2],norm);
        return false;
      }
      return true;
    }
  }
  printg("Mag measurments not available");
  return false;
}

void stopIMU(){
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
}

bool magAvailable(){
  return readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08;
}
void readMag(int16_t data[]){
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
}

void readMagConv(float res[]){
  int16_t data[3];
  readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, 6);
  res[0]=data[0]*400./32768.;
  res[1]=data[1]*400./32768.;
  res[2]=data[2]*400./32768.;
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
