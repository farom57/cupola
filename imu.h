#ifndef IMU_H
#define IMU_H

#include "Arduino.h"


#define MAG_FIELD_MIN 32.
#define MAG_FIELD_MAX 62.

#define K_SMOOTH 0.1
#define MAG_CHANGE_THRESHOLD 2.


void initIMUMag();
void stopIMU();
bool testIMUMag();
bool magAvailable();
void readMag(int16_t data[]);
void readMagConv(float res[]);
void updateMag();

int readRegister(uint8_t slaveAddress, uint8_t address);
int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);



#endif
