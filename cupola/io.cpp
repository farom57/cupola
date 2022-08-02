#include <Arduino.h>
#include "io.h"
#include "nrf_temp.h"

#define DEBOUNCE_DELAY 50



bool oldSwitch1;
bool oldSwitch2;
bool oldSwitch3;
bool oldSwitch4;
bool oldBtn;

void initIO() {
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
  pinMode(PIN_RX_ENABLE, OUTPUT);
  pinMode(PIN_TX_DATA, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  pinMode(PIN_RX_DATA, INPUT);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);
  pinMode(PIN_S3, INPUT);
  pinMode(PIN_S4, INPUT);
  pinMode(PIN_BTN, INPUT);
  pinMode(PIN_BAT, INPUT);
  oldSwitch1 = switch_1();
  oldSwitch2 = switch_2();
  oldSwitch3 = switch_3();
  oldSwitch4 = switch_4();
  oldBtn = btn();

  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
  digitalWrite(PIN_RX_ENABLE, LOW);
  digitalWrite(PIN_TX_DATA, LOW);
  digitalWrite(LED_PWR, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  digitalWrite(PIN_LED3, HIGH);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void ledG(bool state) {
  digitalWrite(PIN_LED1, !state);
}

void ledY(bool state) {
  digitalWrite(PIN_LED2, !state);
}

void ledR(bool state) {
  digitalWrite(PIN_LED3, !state);
}

void ledRYG(bool stateR, bool stateY, bool stateG) {
  digitalWrite(PIN_LED1, !stateG);
  digitalWrite(PIN_LED2, !stateY);
  digitalWrite(PIN_LED3, !stateR);
}
void ledRGB(bool stateR, bool stateG, bool stateB) {
  digitalWrite(LEDR, !stateR);
  digitalWrite(LEDG, !stateG);
  digitalWrite(LEDB, !stateB);
}

bool switch_1() {
  bool val = !digitalRead(PIN_S1);
  pinMode(PIN_S1, INPUT);
  return val;
}

bool switch_2() {
  bool val = !digitalRead(PIN_S2);
  pinMode(PIN_S2, INPUT);
  return val;
}

bool switch_3() {
  bool val = !digitalRead(PIN_S3);
  pinMode(PIN_S3, INPUT);
  return val;
}

bool switch_4() {
  bool val = !digitalRead(PIN_S4);
  pinMode(PIN_S4, INPUT);
  return val;
}

bool btn() {
  bool val = !digitalRead(PIN_BTN);
  pinMode(PIN_BTN, INPUT);
  return val;
}

bool switch_i(int i) {
  switch (i) {
    case 0:
      return btn();
      break;
    case 1:
      return switch_1();
      break;
    case 2:
      return switch_2();
      break;
    case 3:
      return switch_3();
      break;
    case 4:
      return switch_4();
      break;
    default:
      return false;
      break;
  }
}

bool switch_1_chg() {
  if (switch_1() != oldSwitch1) {
    delay(DEBOUNCE_DELAY);
    if (switch_1() != oldSwitch1) {
      oldSwitch1 = switch_1();
      return true;
    }
  }
  return false;
}

bool switch_2_chg() {
  if (switch_2() != oldSwitch2) {
    delay(DEBOUNCE_DELAY);
    if (switch_2() != oldSwitch2) {
      oldSwitch2 = switch_2();
      return true;
    }
  }
  return false;
}
bool switch_3_chg() {
  if (switch_3() != oldSwitch3) {
    delay(DEBOUNCE_DELAY);
    if (switch_3() != oldSwitch3) {
      oldSwitch3 = switch_3();
      return true;
    }
  }
  return false;
}
bool switch_4_chg() {
  if (switch_4() != oldSwitch4) {
    delay(DEBOUNCE_DELAY);
    if (switch_4() != oldSwitch4) {
      oldSwitch4 = switch_4();
      return true;
    }
  }
  return false;
}
bool btn_chg() {
  if (btn() != oldBtn) {
    delay(DEBOUNCE_DELAY);
    if (btn() != oldBtn) {
      oldBtn = btn();
      return true;
    }
  }
  return false;
}

float readTemp() {
  //nrf_temp_init();
  //delay(1);
  //int32_t raw = nrf_temp_read();
  //return raw*0.25;
  int raw;
  NRF_TEMP->TASKS_START = 0x1;
  while (!NRF_TEMP->EVENTS_DATARDY) {}
  raw = NRF_TEMP->TEMP;
  NRF_TEMP->EVENTS_DATARDY = 0;
  NRF_TEMP->TASKS_STOP = 0x1;
  return (float)raw * 0.25;
}

void send_low() {
  // 0
  digitalWrite(PIN_TX_DATA, HIGH);
  delayMicroseconds(479);
  digitalWrite(PIN_TX_DATA, LOW);
  delayMicroseconds(1315);
}
void send_high() {
  // 1
  digitalWrite(PIN_TX_DATA, HIGH);
  delayMicroseconds(1315);
  digitalWrite(PIN_TX_DATA, LOW);
  delayMicroseconds(479);
}

void pause(){
  digitalWrite(PIN_TX_DATA, LOW);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
  delayMicroseconds(1794);
}

void shutdown(){
  digitalWrite(LED_PWR, LOW); // turn off power LED
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW); // turn off sensors
  digitalWrite(PIN_RX_ENABLE, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  digitalWrite(PIN_LED3, HIGH);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(PIN_TX_DATA, LOW);

  pinMode(PIN_ENABLE_I2C_PULLUP, INPUT);

  
  nrf_gpio_cfg_sense_set(31,NRF_GPIO_PIN_SENSE_LOW);
  NRF_POWER->SYSTEMOFF=1;
}
