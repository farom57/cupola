#include "rf.h"
#include "settings.h"
#include "mbed.h"

enum rf_commands rf_command;

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
  flipper.attach(&flip, RF_PERIOD);
  rf_command = NONE;
  pos = 0;
}
void stop_rf() {
  flipper.detach();
  rf_command = NONE;
  digitalWrite(PIN_TX_DATA, LOW);
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
