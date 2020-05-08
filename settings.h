#ifndef SETTINGS_H
#define SETTINGS_H

//<obsolete>define CUPOLA or MOUNT depending on the module
//#define CUPOLA
//#define MOUNT

#include "Arduino.h"

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

#define DEBOUNCE_DELAY 50

#define ST_NB 4
#define ST_KEY_LEN 32
#define ST_VAL_LEN 32

extern char st_keys[ST_NB][ST_KEY_LEN];

void resetSt();                                 // reset setting storage
void loadSt();                                  // load setings from the flash
void defaultSt();                               // set default settings
void saveSt(const char* key, const char* val, int len); // save setting
int readSt(const char* key, char* val);         // read the setting and store the result in val char array. The number of char read is returned
bool readIntSt(const char* key, int* val);      // read the setting and store the result in val. In case of error, val is not modified and false is returned
bool readFloatSt(const char* key, float* val);  // read the setting and store the result in val. In case of error, val is not modified and false is returned

//void stReadHandler(BLEDevice central, BLECharacteristic characteristic);
//void stWriteHandler(BLEDevice central, BLECharacteristic characteristic);



void initBLESettings();

#endif
