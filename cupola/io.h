#ifndef IO_H
#define IO_H


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

#endif
