#include "imu.h"
#include "utility.h"
#include "io.h"
#include "settings.h"
#include "vector.h"
#include "cupola.h"
#include "ble.h"


//#include "ble.h"




// Global variables
enum modes mode = INIT;
float mag_raw[3];
float mag_smooth[3];
float mag_filt[3];
BLEDevice central;


void setup() {

  initIO();

  ledR(true);

  Serial.begin(57600);

  // in debug mode wait btn to initialize
  if (switch_1()) {
    while (!btn()) {}
  }

  //IMU self test
  initIMUMag();
  if (!testIMUMag()) {
    Serial.println("Failed to initialize IMU!");
    digitalWrite(LEDG, LOW);
    while (1);
  }
  stopIMU();

  initBLEPeripherial();


  mode = ADVERTISE;
  ledR(true);
  ledG(true);

}

void loop() {
  static long connectionLastAlive = 2147483647L;
  // --- mode transitions ---
  // transition from INIT to ADVERTISE is managed in setup()
  if (mode == ADVERTISE && central && central.connected()) {
    mode = STANDBY;
  }
  // TODO: transition to SLEEP
  if (mode == STANDBY && (!central.connected() || millis() > connectionLastAlive + CONNECTION_TIMEOUT)) {
    mode = ADVERTISE;
    //TODO: force disconnect
  }
  // transitions between STANDBY and ON are managed in modeChangedHandler()
  
  // --- mode operation ---
  if (mode == ADVERTISE){
    ledG(true);
    ledY(false);
    ledR(true);
    
    central = BLE.central();
    if(central && central.connected()){
      mode = STANDBY;
      return;
    }
    
    wait(1.);
  }

  if (mode == STANDBY){
    ledG(true);
    ledY(false);
    ledR(false);
    
    if(!central.connected()){
      mode = ADVERTISE;
      return;
    }
    if(millis() > connectionLastAlive + CONNECTION_TIMEOUT){
      mode = ADVERTISE;
      BLE.disconnect();
      return;
    }
        
    updateSwitches();

    wait(0.1);
  }

    if (mode == ON){
    ledG(true);
    ledY(true);
    ledR(false);
    
    if(!central.connected()){
      mode = ADVERTISE;
      return;
    }
    if(millis() > connectionLastAlive + CONNECTION_TIMEOUT){
      mode = ADVERTISE;
      BLE.disconnect();
      return;
    }
        
    updateMag();
    updateSwitches();
    

    wait(0.05);
  }

}

void modeChangedHandler(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t requestedMode;
  characteristic.readValue(requestedMode);
  if (requestedMode >= ON) {
        // RED LED will stay ON in case of error
    ledR(true);
    ledG(false);
    ledY(false);

    initIMUMag();
    writeMagRaw(mag_raw);
    writeMagFilt(mag_filt);

    mode = ON;
    
  } else if (requestedMode <= STANDBY) {
    stopIMU();
    mode = STANDBY;
    
  }
}

void magReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  // if the continuous acquisition is OFF, start the IMU for a single measurment
  if (mode < ON) {
    initIMUMag();
    while (!magAvailable()) {}
    readMagConv(mag_raw);
    v_copy(mag_raw, mag_filt);
    stopIMU();
  }else{
    v_copy(mag_smooth, mag_filt);
  }

  writeMagRaw(mag_raw);
  writeMagFilt(mag_filt);

}
