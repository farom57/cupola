#ifndef CUPOLA_H
#define CUPOLA_H



// Global variables
enum states {ERR, INIT, SLEEP, CONNECTION, STANDBY, ON, MOVE, TEST};
extern enum states state;
enum operating_modes {TBD, CUPOLA, MOUNT, DEBUG};
extern enum operating_modes operating_mode;
extern bool acc_error_flag;
extern bool mag_error_flag;

extern float mag_raw[3];
extern float mag_smooth[3];
extern float mag_filt[3];
extern float mag_filt_remote[3];
extern float acc_filt[3];




void setup();
void loop();
void loop_debug();
void loop_mount();
void loop_cupola();

void test_calib();
//void toggleMode();

// defined in ble.h but implemented in cupola.cpp:
//   void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic);
//   void magReadHandler(BLEDevice central, BLECharacteristic characteristic);
//   void btnChangedHandler(BLEDevice central, BLECharacteristic characteristic);

#endif
