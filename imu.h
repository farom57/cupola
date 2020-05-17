#ifndef IMU_H
#define IMU_H

#include "Arduino.h"


#define MAG_INIT_MIN 22.
#define MAG_INIT_MAX 72.

#define ACC_INIT_MIN 0.95
#define ACC_INIT_MAX 1.05

#define K_SMOOTH 0.1
#define MAG_CHANGE_THRESHOLD 2.

#define CALIB_SAMPLES 10
#define CALIB_PERIOD 1000

extern bool acc_error_flag;
extern bool mag_error_flag;

extern float mag_raw[3];
extern float mag_smooth[3];
extern float mag_filt[3];
extern float mag_filt_remote[3];
extern float acc_filt[3];
extern int current_calib_sample;

void initIMUMag();
void initIMUMagAcc();
void stopIMU();
bool testIMUMag();
bool testIMUAcc();
bool magAvailable();
bool accAvailable();

void readMag(int16_t data[]);
void readMagConv(float res[]);
void readAcc(int16_t data[]);
void readAccConv(float res[]);

void updateMag();
void updateAcc();

int readRegister(uint8_t slaveAddress, uint8_t address);
int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

void sampleCalib();   // Acquire and save calibration sample
void compassCalibCalc();  // Compute compass calibration parameters
void compassCalib(float in[], float res[]); // correct res

#endif
