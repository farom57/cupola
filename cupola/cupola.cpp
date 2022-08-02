#include "imu.h"
#include "utility.h"
#include "io.h"
#include "cupola.h"
#include "rf.h"
#include "ble.h"
#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>

#define LOOP_PERIOD 100
#define AUTO_SHUTDOWN 600000


// Global variables

enum states state = INIT;
boolean debug_mode = false;
boolean manual_command = false;


// LED RGB code:
// 000 black: no error
// 001 blue: ble error
// 010 green: IMU mag error
// 011 green-blue: RF error
// 100 red: waiting press for init in debug mode
// 110 yellow: bad magnetic field 


// Dip switches:
// +XXX: debug mode
// X+XX: manual rotation
//    X+__ and push for right
//    X+_+ and push for left
//    X++_ and push for down
//    X+++ and push for up
//    Unset switch n°2 to maintain rotation

void setup() {

  initIO();

  if (switch_1()) {
    debug_mode = true;
    ledRGB(true, false, false);
//    PluggableUSBD().begin();
//    SerialUSB.begin(115200);
    while (!btn()) {}
  }


  ledRYG(true, false, false);
  Serial.begin(57600);
  log_i("Compilation date: " __DATE__ " " __TIME__);


  initIMUMag();
  if (!testIMUMag()) {
    log_e("Failed to initialize Magnetometer in debug mode. Continue");
    ledRGB(false, true, false);
    while(true){}
  }
  stopIMU(); // stop the IMU, it is activated on demand to save power

  ledRGB(false, false, true);
  initBLEPeripheral();
  ledRGB(false, false, false);
  state = CONNECTION;

  ledRGB(false, true, true);
  start_rf();
  set_rf_cmd(NONE);
  ledRGB(false, false, false);

  log_i("Initialisation done");

}


void loop() {

  ledRYG(state != STANDBY, rf_command != NONE, state == STANDBY);

  ledRGB(false, false, true);
  BLE.poll();
  ledRGB(false, false, false);


  // state transitions
  // Connection, state change directly to ON in debug mode
  if (state == CONNECTION && connectedPeripheral()) {
    state = STANDBY;
    writeState(state);
    initIMUMag();
    log_i("Connection, state change to STANDBY");
  }
  
  // Disconnection
  if (state == STANDBY && !connectedPeripheral()) {
    stopIMU();
    log_e("Disconnection, System reset");
    //NVIC_SystemReset();
    shutdown();  
  }



  enum rf_commands cmd = readRfCmd();
  if (cmd != rf_command) {
    log_i("rf command changed from ble:%i", (byte)cmd);
    manual_command = false;
    set_rf_cmd(cmd);
  }
  
  // Cupola manual rotation: DIP X+XX and push
  // X+__ and push for right
  // X+_+ and push for left
  // X++_ and push for down
  // X+++ and push for up
  if (switch_2()) {
    if (btn()) {
      if (!switch_3() && !switch_4())   set_rf_cmd(RIGHT);
      if (!switch_3() && switch_4())  set_rf_cmd(LEFT);
      if (switch_3() && !switch_4())    set_rf_cmd(DOWN);
      if (switch_3() && switch_4())   set_rf_cmd(UP);
      manual_command = true;
    } else if (manual_command) {
      set_rf_cmd(NONE);
      manual_command = false;
    }
  } 

  // state operations
  if (state == CONNECTION) {
    connectBLEPeripheral();

    if(millis()>AUTO_SHUTDOWN && !debug_mode){
      shutdown(); 
    }

  }
  if (state == STANDBY) {
    updateSwitches();
    updateMag();
    
  }




  
  
  delay(LOOP_PERIOD);
}





// called if manual rf command changed
void rfCmdHandler(BLEDevice central, BLECharacteristic characteristic) {
  const enum rf_commands cmd = (enum rf_commands)(*characteristic.value());
  log_i("received rf command by BLE: %i", (byte)cmd);
  manual_command = false;
  set_rf_cmd(cmd);
}
