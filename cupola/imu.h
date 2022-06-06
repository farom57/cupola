#ifndef IMU_H
#define IMU_H

#include <Arduino.h>


#define MAG_INIT_MIN 22.
#define MAG_INIT_MAX 72.

#define K_SMOOTH 0.2

#define DEG(a)  a*360./2./PI
#define RAD(a)  a*2.*PI/360.



extern float mag_raw[3];
extern float mag_smooth[3];


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
