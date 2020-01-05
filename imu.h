#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

#define LSM9DS1_ADDRESS_M          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_CTRL_REG5_M        0x24
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

#define MAG_FIELD_MIN 37.
#define MAG_FIELD_MAX 57.

void initIMUMag();
void stopIMU();
bool testIMUMag();
bool magAvailable();
void readMag(int16_t data[]);
void readMagConv(float res[]);

int readRegister(uint8_t slaveAddress, uint8_t address);
int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

#endif
