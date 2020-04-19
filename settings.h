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



void defaultSettings();

// load saved settings if existing
// return true if success, false otherwise
bool loadSettings();

void saveSettings();

void initBLESettings();

#endif
