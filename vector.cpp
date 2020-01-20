#include <math.h>
#include "Arduino.h"
#include "vector.h"
#include "utility.h"


float norm(float v[]) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float norm2(float v[]) {
  return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void normalize(float v[], float res[]) {
  float n = norm(v);
  sv_mult(1. / n, v, res);
}

void m_copy(float m[], float res[]) {
  res[0] = m[0];
  res[1] = m[1];
  res[2] = m[2];
  res[3] = m[3];
  res[4] = m[4];
  res[5] = m[5];
  res[6] = m[6];
  res[7] = m[7];
  res[8] = m[8];
}

void v_copy(float v[], float res[]) {
  res[0] = v[0];
  res[1] = v[1];
  res[2] = v[2];
}

void mm_mult(float m[], float n[], float res[]) {
  res[0] = m[0] * n[0] + m[1] * n[3] + m[2] * n[6];
  res[1] = m[0] * n[1] + m[1] * n[4] + m[2] * n[7];
  res[2] = m[0] * n[2] + m[1] * n[5] + m[2] * n[8];
  res[3] = m[3] * n[0] + m[4] * n[3] + m[5] * n[6];
  res[4] = m[3] * n[1] + m[4] * n[4] + m[5] * n[7];
  res[5] = m[3] * n[2] + m[4] * n[5] + m[5] * n[8];
  res[6] = m[6] * n[0] + m[7] * n[3] + m[8] * n[6];
  res[7] = m[6] * n[1] + m[7] * n[4] + m[8] * n[7];
  res[8] = m[6] * n[2] + m[7] * n[5] + m[8] * n[8];
}

void mv_mult(float m[], float v[], float res[]) {
  res[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  res[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  res[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void sv_mult(float s, float v[], float res[]) {
  res[0] = s * v[0];
  res[1] = s * v[1];
  res[2] = s * v[2];
}

void v_lincomb(float a, float u[], float b, float v[], float res[]) {
  res[0] = a * u[0] + b * v[0];
  res[1] = a * u[1] + b * v[1];
  res[2] = a * u[2] + b * v[2];
}


void sm_mult(float s, float m[], float res[]) {
  res[0] = s * m[0];
  res[1] = s * m[1];
  res[2] = s * m[2];
  res[3] = s * m[3];
  res[4] = s * m[4];
  res[5] = s * m[5];
  res[6] = s * m[6];
  res[7] = s * m[7];
  res[8] = s * m[8];
}

void transpose(float m[], float res[]) {
  res[0] = m[0];
  res[1] = m[3];
  res[2] = m[6];
  res[3] = m[1];
  res[4] = m[4];
  res[5] = m[7];
  res[6] = m[2];
  res[7] = m[5];
  res[8] = m[8];
}

float scalar_prod(float u[], float v[]) {
  return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void vect_prod(float u[], float v[], float res[]) {
  res[0] = u[1] * v[2] - u[2] * v[1];
  res[1] = u[2] * v[0] - u[0] * v[2];
  res[2] = u[0] * v[1] - u[1] * v[0];
}

void v_add(float u[], float v[], float res[]) {
  res[0] = u[0] + v[0];
  res[1] = u[1] + v[1];
  res[2] = u[2] + v[2];
}

void v_sub(float u[], float v[], float res[]) {
  res[0] = u[0] - v[0];
  res[1] = u[1] - v[1];
  res[2] = u[2] - v[2];
}

void vect_cat(float u[], float v[], float w[], float res[]) {
  res[0] = u[0];
  res[1] = v[0];
  res[2] = w[0];
  res[3] = u[1];
  res[4] = v[1];
  res[5] = w[1];
  res[6] = u[2];
  res[7] = v[2];
  res[8] = w[2];
}

void vect_extract(float m[], uint8_t col, float res[]) {
  res[0] = m[col];
  res[1] = m[col + 3];
  res[2] = m[col + 6];
}

void v_print(const char msg[], float v[]) {
  int i = 0;
  char buf[10];



  while (msg[i] != 0) {
    printg("%c", msg[i]);
    i++;
  }
  printg("%7.3f\n\r", v[0]);

  for (int j = 0; j < i; j++) {
    printg(" ");
  }
  printg("%7.3f\n\r", v[1]);

  for (int j = 0; j < i; j++) {
    printg(" ");
  }
  printg("%7.3f\n\r", v[2]);
}

void m_print(const char msg[], float m[]) {
  int i = 0;
  char buf[10];

  while (msg[i] != 0) {
    printg("%c", msg[i]);
    i++;
  }
  printg("%7.3f\t%7.3f\t%7.3f\n\r", m[0], m[1], m[2]);

  for (int j = 0; j < i; j++) {
    printg(" ");
  }
  printg("%7.3f\t%7.3f\t%7.3f\n\r", m[0], m[1], m[2]);

  for (int j = 0; j < i; j++) {
    printg(" ");
  }
  printg("%7.3f\t%7.3f\t%7.3f\n\r", m[0], m[1], m[2]);
}

void rotx(float angle, float res[]) {
  float cs = cos(angle);
  float sn = sin(angle);
  res[0] = 1;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;
  res[4] = cs;
  res[5] = -sn;
  res[6] = 0;
  res[7] = sn;
  res[8] = cs;
}

void roty(float angle, float res[]) {
  float cs = cos(angle);
  float sn = sin(angle);
  res[0] = cs;
  res[1] = 0;
  res[2] = sn;
  res[3] = 0;
  res[4] = 1;
  res[5] = -0;
  res[6] = -sn;
  res[7] = 0;
  res[8] = cs;
}

void rotz(float angle, float res[]) {
  float cs = cos(angle);
  float sn = sin(angle);
  res[0] = cs;
  res[1] = -sn;
  res[2] = 0;
  res[3] = sn;
  res[4] = cs;
  res[5] = 0;
  res[6] = 0;
  res[7] = 0;
  res[8] = 1;
}

void print_degstr(float rad) {
  char str[20];
  while (rad < -PI)
    rad += 2 * PI;
  while (rad >= PI)
    rad -= 2 * PI;
  float deg = rad * 360. / 2. / PI;
  float abs_deg = abs(deg);
  int d, m, s;
  d = abs_deg;
  m = (abs_deg - d) * 60;
  s = (abs_deg * 60. - d * 60. - m) * 60;
  d = deg;
  sprintf(str, "%d:%d:%d", d, m, s);
  Serial.println(str);
}


void print_hourstr(float rad) {
  char str[20];
  while (rad < 0)
    rad += 2 * PI;
  while (rad >= PI * 2)
    rad -= 2 * PI;
  float deg = rad * 24. / 2. / PI;
  float abs_deg = abs(deg);
  int d, m, s;
  d = abs_deg;
  m = (abs_deg - d) * 60;
  s = (abs_deg * 60. - d * 60. - m) * 60;
  d = deg;
  sprintf(str, "%d:%d:%d", d, m, s);
  Serial.println(str);
}

/*void calibrate_mag(int16_t mag_ADC[], float res[]){
  res[1]=-(((float)mag_ADC[0]-magbias[0])*magGain[0]);
  res[0]=(((float)mag_ADC[1]-magbias[1])*magGain[1]);
  res[2]=-(((float)mag_ADC[2]-magbias[2])*magGain[2]);

  }

  void calibrate_acc(int16_t acc_ADC[], float res[]){
  res[1]=((float)acc_ADC[0]-accbias[0])*accGain[0];
  res[0]=-(((float)acc_ADC[1]-accbias[1])*accGain[1]);
  res[2]=((float)acc_ADC[2]-accbias[2])*accGain[2];
  }

  void hadec_calc(float acc_cal[], float mag_cal[]){

  }*/
