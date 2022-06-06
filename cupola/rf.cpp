#include "rf.h"
#include "utility.h"
#include "ble.h"
#include "io.h"
#include "mbed.h"

using namespace std::chrono;



enum rf_commands rf_command;
bool  rf_started = false;

// 0 = short pulse 1 = long pulse 2 = silent
const int rf_symbols[7][32] = {
  {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2},
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2}
};

mbed::Ticker flipper;

int pos;

void start_rf() {
  if(rf_started){
    log_w("rf already started");
    return;
  }  
  flipper.attach(&flip, 449us);
  rf_command = NONE;
  pos = 0;
  rf_started = true;

}
void stop_rf() {
  if(!rf_started){
    log_w("rf already stopped");
    return;
  }  
  flipper.detach();
  rf_command = NONE;
  digitalWrite(PIN_TX_DATA, LOW);
  rf_started = false;

}

void flip() {
  int symbol = rf_symbols[rf_command][pos >> 2];
  if(symbol>=2){
    digitalWrite(PIN_TX_DATA, LOW);
  }else if (pos%4 == 0){
    digitalWrite(PIN_TX_DATA, HIGH);
  }else if ((symbol==0 && pos%4==1)||(symbol==1 && pos%4==3)){
    digitalWrite(PIN_TX_DATA, LOW);
  }
  
  pos++;
  if(pos>=32*4){
    pos=0;
  }
}

void set_rf_cmd(rf_commands cmd){
  if((int)cmd>0 && (int)cmd<=6){
    log_i("Starting RF");
    rf_command = cmd;
    
    start_rf();
  }else if(cmd==NONE){
    stop_rf();
    log_i("Stopping RF");
  }else{
    log_w("Unknown rf command %i",cmd);
  }
  

  writeRfCmd(rf_command);
}
