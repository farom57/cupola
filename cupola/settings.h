#ifndef SETTINGS_H
#define SETTINGS_H

//<obsolete>define CUPOLA or MOUNT depending on the module
//#define CUPOLA
//#define MOUNT

#include <Arduino.h>

// PIN
#define PIN_S4 5 // D5 = P1_13
#define PIN_S3 6
#define PIN_S2 9
#define PIN_S1 10
#define PIN_BTN PIN_A4
#define PIN_LED1 2
#define PIN_LED2 3
#define PIN_LED3 4
#define PIN_RX_ENABLE PIN_A6
#define PIN_TX_DATA PIN_A0
#define PIN_RX_DATA PIN_A7
#define PIN_BAT PIN_A5 // A5 = P0_2 = AIN0

#define CONNECTION_KEEPALIVE_TIMEOUT 10000L
#define CONNECTION_KEEPALIVE_TIMEOUT2 20000L
#define LOOP_PERIOD 100

#define DEBOUNCE_DELAY 50

#define ST_NB 13
#define ST_KEY_LEN 32
#define ST_VAL_LEN 200

#define DEFAULT_LAT 43.56
#define DEFAULT_MAG_X -0.9983 // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
#define DEFAULT_MAG_Y -23.7225
#define DEFAULT_MAG_Z -40.3228

#define HEADING_THRESHOLD 3.
#define MOVING_SPEED 3. //deg/s
#define MOVING_NOISE 1.5 //deg/s, standard deviation of process noise when moving
#define REST_NOISE 0.1 //deg/s, standard deviation of process noise when static
#define MEASUREMENT_NOISE 0.86 //deg, standard deviation of measurement

enum setting_type { INT, FLOAT, FLOAT3, FLOAT9};

struct setting {
  char key[ST_KEY_LEN];
  void* ptr;
  enum setting_type type;
};

extern int st_startup;
extern float st_compass_bias[3];
extern float st_compass_amp[3];
extern float st_compass_rot[3][3];
extern float st_compass_heading_bias;
extern float st_lat;
extern float st_ref_mag[3];
extern float st_A_mag_inv[3][3];
extern float st_A_acc_inv[3][3];
extern float st_bias_mag[3];
extern float st_bias_acc[3];
extern float st_sigma_acc;
extern float st_sigma_mag;

extern struct setting settings[ST_NB];

void resetSt();                                 // reset setting storage
void loadSt();                                  // load setings from the flash
void defaultSt();                               // set default settings
void saveAllSt();                               // Save all settings
void saveSt(const char* key, const char* val, int len); // save setting
void saveFloatSt(const char* key, float val);   // save setting
void saveFloat9St(const char* key, float val[9]);   // save setting
void saveFloat3St(const char* key, float val[3]);   // save setting
void saveIntSt(const char* key, int val);       // save setting
int readSt(const char* key, char* val);         // read the setting and store the result in val char array. The number of char read is returned
bool readIntSt(const char* key, int* val);      // read the setting and store the result in val. In case of error, val is not modified and false is returned
bool readFloatSt(const char* key, float* val);  // read the setting and store the result in val. In case of error, val is not modified and false is returned
void readFloat9St(const char* key, float val[9]);    // read setting
void readFloat3St(const char* key, float val[3]);   // read setting

//void stReadHandler(BLEDevice central, BLECharacteristic characteristic);
//void stWriteHandler(BLEDevice central, BLECharacteristic characteristic);



void initBLESettings();

#endif
