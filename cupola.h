#ifndef CUPOLA_H
#define CUPOLA_H



// Global variables
enum states {ERR, INIT, SLEEP, CONNECTION, STANDBY, ON, COMPASS_CALIB};
extern enum states state;
enum operating_modes {TBD, CUPOLA, MOUNT, DEBUG};
extern enum operating_modes operating_mode;





void setup();
void loop();
void loop_debug();
void loop_mount();
void loop_cupola();

// defined in ble.h but implemented in cupola.cpp:
//   void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic);
//   void magReadHandler(BLEDevice central, BLECharacteristic characteristic);
//   void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic);

#endif
