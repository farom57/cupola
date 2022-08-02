#ifndef IO_H
#define IO_H

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

void initIO();

void ledG(bool state);
void ledY(bool state);
void ledR(bool state);
void ledRYG(bool stateR,bool stateY,bool stateG);
void ledRGB(bool stateR,bool stateG,bool stateB);


bool switch_i(int i);
bool switch_1();
bool switch_2();
bool switch_3();
bool switch_4();
bool btn();

bool switch_i_chg(int i);
bool switch_1_chg();
bool switch_2_chg();
bool switch_3_chg();
bool switch_4_chg();
bool btn_chg();

void send_low();
void send_high();
void pause();

float readTemp();
void shutdown();


#endif
