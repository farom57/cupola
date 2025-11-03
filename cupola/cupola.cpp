#include <Arduino.h>
#define BTN_UP 2
#define BTN_LEFT 3
#define BTN_RIGHT 4
#define BTN_DOWN 5
#define BTN_STOP 6
#define BTN_LIGHT 7
#define OUT_R1 8
#define OUT_R2 9
#define OUT_OPEN 13
#define OUT_CLOSE 12
#define OUT_LEFT 10
#define OUT_RIGHT 11
#define OUT_BUZZER A0
#define IN_PHOT_1 A1
#define IN_PHOT_2 A2
#define IN_HOME A3
#define IN_OPEN A4
#define IN_CLOSED A5

#define PRESS_TIMEOUT 10
#define LONG_PRESS_TIMEOUT 1000
#define RELEASE_TIMEOUT 10
#define TOTAL_STEPS 48 //4320



#include "cupola.h"
enum DOME_CMD dome_cmd;
enum COVER_CMD cover_cmd;
enum LIGHT_STATE light_state;

int btn_counter[6]={0};
int step_counter = 0;
int step_home = -1;
int target = -1;
bool prev_home;


int val = 0;

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(10);

  pinMode(BTN_UP, INPUT);
  pinMode(BTN_LEFT, INPUT);
  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(BTN_STOP, INPUT);
  pinMode(BTN_LIGHT, INPUT);
  pinMode(IN_PHOT_1, INPUT);
  pinMode(IN_PHOT_2, INPUT);
  pinMode(IN_HOME, INPUT);
  pinMode(IN_OPEN, INPUT);
  pinMode(IN_CLOSED, INPUT);

  pinMode(OUT_R1, OUTPUT);
  pinMode(OUT_R2, OUTPUT);
  pinMode(OUT_OPEN, OUTPUT);
  pinMode(OUT_CLOSE, OUTPUT);
  pinMode(OUT_LEFT, OUTPUT);
  pinMode(OUT_RIGHT, OUTPUT);
  pinMode(OUT_BUZZER, OUTPUT); //used by analogWave

  dome_cmd = DOME_MANUAL;
  cover_cmd = COVER_MANUAL;
  light_state = WHITE;
  prev_home = digitalRead(IN_HOME);

}




void loop() {
  
  // Light button. If the light is OFF it turn red with a short press or white with a long press
  if(digitalRead(BTN_LIGHT) == HIGH){
    btn_counter[5]  = 0;
  }else{
    btn_counter[5] ++;
    if(btn_counter[5] == PRESS_TIMEOUT){
      if(light_state == OFF){
        light_state = RED;
      }else{
        light_state = OFF;
      }
    }
    if(btn_counter[5] == LONG_PRESS_TIMEOUT){
      if(light_state == RED){
        light_state = WHITE;
      }
    }
  }

  // Step counting 
  int old_step_counter = step_counter;
  if(digitalRead(IN_PHOT_1) && digitalRead(IN_PHOT_2)){
    step_counter = ((step_counter+1) & 0xFFFFFFFC) | 0;
  }else if(digitalRead(IN_PHOT_1) && !digitalRead(IN_PHOT_2)){
    step_counter = (step_counter & 0xFFFFFFFC) | 1;
  }else if(!digitalRead(IN_PHOT_1) && !digitalRead(IN_PHOT_2)){
    step_counter = (step_counter & 0xFFFFFFFC) | 2;
  }else if(!digitalRead(IN_PHOT_1) && digitalRead(IN_PHOT_2)){
    step_counter = ((step_counter-1) & 0xFFFFFFFC) | 3;
  }

  // check home reached
  if(old_step_counter != step_counter && prev_home != digitalRead(IN_HOME)){
    if(prev_home == false && step_counter > old_step_counter){
      step_home = old_step_counter;
    }
    prev_home = !prev_home;
  }

  // check target reached
  if(old_step_counter != step_counter && target >= 0){
    if((step_counter >= target && target > old_step_counter)||(step_counter <= target && target < old_step_counter)){
      dome_cmd = DOME_MANUAL;
      target = -1;
      Serial.println("Target reached");
    }
  }

  if(step_counter < 0){
    step_counter = step_counter + TOTAL_STEPS;
  }
  if(step_counter >= TOTAL_STEPS){
    step_counter = step_counter - TOTAL_STEPS;
  }

  /*
  if(old_step_counter != step_counter){
    Serial.print("Step:");
    Serial.print(step_counter);
    Serial.print(" phot_1:");
    Serial.print(digitalRead(IN_PHOT_1));
    Serial.print(" phot_2:");
    Serial.println(digitalRead(IN_PHOT_2));
  }
  */

  // Stop if any button except light is pressed
  if(!digitalRead(BTN_UP) || !digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT) || !digitalRead(BTN_DOWN) || !digitalRead(BTN_STOP))
  {
    dome_cmd = DOME_MANUAL;
    cover_cmd = COVER_MANUAL;
    target = -1;
  }



  // Apply output
  if((dome_cmd==DOME_MANUAL && !digitalRead(BTN_LEFT) && digitalRead(BTN_RIGHT)) || dome_cmd==LEFT ){
    digitalWrite(OUT_LEFT,HIGH);
    digitalWrite(OUT_RIGHT,LOW);
  }else if((dome_cmd==DOME_MANUAL && digitalRead(BTN_LEFT) && !digitalRead(BTN_RIGHT)) || dome_cmd==RIGHT ){
    digitalWrite(OUT_LEFT,LOW);
    digitalWrite(OUT_RIGHT,HIGH);
  }else{
    digitalWrite(OUT_LEFT,LOW);
    digitalWrite(OUT_RIGHT,LOW);
  }

  if((cover_cmd==COVER_MANUAL && !digitalRead(BTN_UP) && digitalRead(BTN_DOWN)) || cover_cmd==OPEN ){
    digitalWrite(OUT_OPEN,HIGH);
    digitalWrite(OUT_CLOSE,LOW);
  }else if((cover_cmd==COVER_MANUAL && digitalRead(BTN_UP) && !digitalRead(BTN_DOWN)) || cover_cmd==CLOSE ){
    digitalWrite(OUT_OPEN,LOW);
    digitalWrite(OUT_CLOSE,HIGH);
  }else{
    digitalWrite(OUT_OPEN,LOW);
    digitalWrite(OUT_CLOSE,LOW);
  }

  if(light_state==RED){
    digitalWrite(OUT_R1,HIGH);
    digitalWrite(OUT_R2,LOW);
  }else if(light_state==WHITE){
    digitalWrite(OUT_R1,HIGH);
    digitalWrite(OUT_R2,HIGH);
  }else{
    digitalWrite(OUT_R1,LOW);
    digitalWrite(OUT_R2,LOW);
  }


  comm();
  
  delay(1);
}

void comm(){
  char inByte;
  if(Serial.available() > 0){
    inByte = Serial.read();
    switch(inByte){
      case '?':
        Serial.println("?: help");
        Serial.println("h: get home step");
        Serial.println("s: get current step");
        Serial.println("i: get current inputs");
        Serial.println("l: go left");
        Serial.println("r: go right");
        Serial.println("o: open");
        Serial.println("c: close");
        Serial.println("x: stop");
        Serial.println("b: turn off light");
        Serial.println("n: light red");
        Serial.println("w: light white");
        Serial.println("txxx: set target to xxx");
        break;
      case 's':
        Serial.println(step_counter);
        break;
      case 'i':
        int i;
        i += (digitalRead(IN_HOME) & 1) << 4;
        i += (digitalRead(IN_OPEN) & 1) << 3;
        i += (digitalRead(IN_CLOSED) & 1) << 2;
        i += (digitalRead(IN_PHOT_1) & 1) << 1;
        i += (digitalRead(IN_PHOT_2) & 1) << 0;
        i += (digitalRead(BTN_UP) & 1) << 13;
        i += (digitalRead(BTN_LEFT) & 1) << 12;
        i += (digitalRead(BTN_RIGHT) & 1) << 11;
        i += (digitalRead(BTN_DOWN) & 1) << 10;
        i += (digitalRead(BTN_STOP) & 1) << 9;
        i += (digitalRead(BTN_LIGHT) & 1) << 8;
        Serial.println(i, HEX);
        break;
      case 'h':
        Serial.println(step_home);
        break;
      case 'l':
        dome_cmd = LEFT;
        break;
      case 'r':
        dome_cmd = RIGHT;
        break;
      case 'o':
        cover_cmd = OPEN;
        break;
      case 'c':
        cover_cmd = CLOSE;
        break;
      case 'x':
        dome_cmd = DOME_MANUAL;
        cover_cmd = COVER_MANUAL;
        target = -1;
        break;
      case 'b':
        light_state = OFF;
        break;
      case 'n':
        light_state = RED;
        break;
      case 'w':
        light_state = WHITE;
        break;
      case 't':
        int tmp = Serial.parseInt();
        int delta;
        if(tmp > 0){
          target = tmp;
          delta = target - step_counter;
          if(delta > TOTAL_STEPS/2){
            delta = delta - TOTAL_STEPS;
          }
          if(delta < -TOTAL_STEPS/2){
            delta = delta + TOTAL_STEPS;
          }
          Serial.print("delta:");
          Serial.println(delta);
          if(delta > 0){
            dome_cmd = LEFT;
          }else{
            dome_cmd = RIGHT;
          }
        }else{
          dome_cmd = DOME_MANUAL;
          cover_cmd = COVER_MANUAL;
          target = -1;
        }
        Serial.println(target);
    }
  }
}
