#ifndef RF_H
#define RF_H

// UNIWERSAL cupola protocol
// 433.9MHz with OOK modulation
// Symbols:
//  - Short pulse (0) : 448.9us ON then 3*448.9=1346.7us OFF
//  - Long pulse  (1) : 3*448.9=1346.7us ON then 448.9us OFF
// Sequences of 25 sybols separated by 7*448.9us. In function of the button pressed on the remote:
// UP:     0101010101010000000000110 
// DOWN:   0101010101010000001100000
// LEFT:   0101010101010000000011000
// RIGHT:  0101010101010000110000000
// SQUARE: 0101010101010011000000000
// HORN:   0101010101011100000000000


#define RF_PERIOD 0.0004489

enum rf_commands {NONE, UP, DOWN, LEFT, RIGHT, SQUARE, HORN};
extern enum rf_commands rf_command;


void start_rf();
void stop_rf();

void set_rf_cmd(rf_commands cmd);

void flip();


#endif
