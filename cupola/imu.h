#ifndef IMU_H
#define IMU_H

#include <Arduino.h>


#define MAG_INIT_MIN 22.
#define MAG_INIT_MAX 72.

#define ACC_INIT_MIN 0.95
#define ACC_INIT_MAX 1.05

#define K_SMOOTH 0.2
#define MAG_CHANGE_THRESHOLD 0.5

#define CALIB_SAMPLES 9
#define CALIB_DELAY 1000

#define DEG(a)  a*360./2./PI
#define RAD(a)  a*2.*PI/360.

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
float heading(float in[]); // return the heading in rad

void mountCalib(const float raw[], const float invA[], const float bias[], int N, float res[]); // calibrate mount measurements
float mountCalibCalc();// Compute mount calibration
float mountCalibCalc(const float raw[3][CALIB_SAMPLES], const float angles[3][CALIB_SAMPLES], const float theory[3], float A[3][3], float bias[3]);// Compute mount calibration parameters A and bias, return sigma
void mountRot(const float mag[3], const float acc[3], float lat, const float mag_ref[3], float sigma_mag, float sigma_acc, float *ha_rot, float *dec_rot);//Estimate the rotation of the mount
void mountRot(const float mag[], const float acc[], float lat, const float mag_ref[3], float sigma_mag, float sigma_acc, float ha_rot[], float dec_rot[], int N); //Estimate the rotation of the mount

void test();

#endif
