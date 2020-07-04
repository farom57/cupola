#include <math.h>
#include "Arduino.h"
#include "vector.h"
#include "utility.h"
#include "imu.h"


float norm(const float v[]) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float norm2(const float v[]) {
  return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void normalize(const float v[], float res[]) {
  float n = norm(v);
  sv_mult(1. / n, v, res);
}

void m_copy(const float m[], float res[]) {
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

void v_copy(const float v[], float res[]) {
  res[0] = v[0];
  res[1] = v[1];
  res[2] = v[2];
}



void mm_mult(const float m[], const float n[], float res[]) {
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

void mm_mult(const float m[3][3], const float n[3][3], float res[3][3]) {
  mm_mult((const float*)m, (const float*)n, (float*)res);
}

void mv_mult(const float m[], const float v[], float res[]) {
  res[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  res[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  res[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void mv_mult(const float m[3][3], const float v[3], float res[3]) {
  mv_mult((const float*) m, (const float*) v, (float*) res);
}

void sv_mult(float s, const float v[], float res[]) {
  res[0] = s * v[0];
  res[1] = s * v[1];
  res[2] = s * v[2];
}

void v_lincomb(float a, const float u[], float b, const float v[], float res[]) {
  res[0] = a * u[0] + b * v[0];
  res[1] = a * u[1] + b * v[1];
  res[2] = a * u[2] + b * v[2];
}


void sm_mult(float s, const float m[], float res[]) {
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

void transpose(const float m[], float res[]) {
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

void transpose(const float m[3][3], float res[3][3]) {
  transpose((const float*)m, (float*)res);
}

float scalar_prod(const float u[], const float v[]) {
  return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void vect_prod(const float u[], const float v[], float res[]) {
  res[0] = u[1] * v[2] - u[2] * v[1];
  res[1] = u[2] * v[0] - u[0] * v[2];
  res[2] = u[0] * v[1] - u[1] * v[0];
}

void v_add(const float u[], const float v[], float res[]) {
  res[0] = u[0] + v[0];
  res[1] = u[1] + v[1];
  res[2] = u[2] + v[2];
}

void v_sub(const float u[], const float v[], float res[]) {
  res[0] = u[0] - v[0];
  res[1] = u[1] - v[1];
  res[2] = u[2] - v[2];
}

void vect_cat(const float u[], const float v[], const float w[], float res[]) {
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

void vect_extract(const float m[], uint8_t col, float res[]) {
  res[0] = m[col];
  res[1] = m[col + 3];
  res[2] = m[col + 6];
}

void v_print(const char msg[], const float v[]) {
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

void m_print(const char msg[], const float m[]) {
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
  printg("%7.3f\t%7.3f\t%7.3f\n\r", m[3], m[4], m[5]);

  for (int j = 0; j < i; j++) {
    printg(" ");
  }
  printg("%7.3f\t%7.3f\t%7.3f\n\r", m[6], m[7], m[8]);
}

void rotx(float angle, float res[3][3]) {
  float cs = cos(angle);
  float sn = sin(angle);
  res[0][0] = 1;
  res[0][1] = 0;
  res[0][2] = 0;
  res[1][0] = 0;
  res[1][1] = cs;
  res[1][2] = -sn;
  res[2][0] = 0;
  res[2][1] = sn;
  res[2][2] = cs;
}

void roty(float angle, float res[3][3]) {
  float cs = cos(angle);
  float sn = sin(angle);

  res[0][0] = cs;
  res[0][1] = 0;
  res[0][2] = sn;
  res[1][0] = 0;
  res[1][1] = 1;
  res[1][2] = 0;
  res[2][0] = -sn;
  res[2][1] = 0;
  res[2][2] = cs;

}

void rotz(float angle, float res[3][3]) {
  float cs = cos(angle);
  float sn = sin(angle);

  res[0][0] = cs;
  res[0][1] = -sn;
  res[0][2] = 0;
  res[1][0] = sn;
  res[1][1] = cs;
  res[1][2] = 0;
  res[2][0] = 0;
  res[2][1] = 0;
  res[2][2] = 1;

}

void tel2topo(float lat, float ha_rot, float dec_rot, float res[3][3]) {
  //topo = wsz = [ouest sud zenith]
  //   rotation d'axe x et d'angle pi/2-lat
  // pole = wdp = [ouest midi pole_nord]
  //   rotation d'axe z et d'angle ha_rot
  // ha = _cp = [_|_ CP2instrum pone_nord]
  //   rotation d'axe y et d'angle dec_rot
  // tel = _ci = [_|_ CP2instrum target]
  float wdp_wsz[3][3];
  float _cp_wdp[3][3];
  float _ci__cp[3][3];
  float tmp[3][3];

  rotx(PI / 2 - lat, wdp_wsz);
  rotz(ha_rot, _cp_wdp);
  roty(dec_rot, _ci__cp);

  //res=wdp_wsz*_cp_wdp*_ci__cp;
  mm_mult(_cp_wdp, _ci__cp, tmp);
  mm_mult(wdp_wsz, tmp, res);
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




float norm(const float v[], int sz) {
  return sqrt(norm2(v, sz));
}

float norm2(const float v[], int sz) {
  float res = 0;
  for (int i = 0; i < sz; i++) {
    res += v[i] * v[i];
  }
  return res;
}

void normalize(const float v[], float res[], int sz) {
  float n = norm(v);
  sv_mult(1. / n, v, res, sz);
}

void m_copy(const float m[], float res[], int sz) {
  for (int i = 0; i < sz; i++) {
    res[i] = m[i];
  }
}

void v_copy(const float v[], float res[], int sz) {
  for (int i = 0; i < sz; i++) {
    res[i] = v[i];
  }
}

void mm_mult(const float m[], const float n[], float res[], int sz1, int sz2, int sz3) {
  for (int i = 0; i < sz1; i++) {
    for (int j = 0; j < sz3; j++) {
      res[sz3 * i + j] = 0;
      for (int k = 0; k < sz2; k++) {
        res[sz3 * i + j] += m[sz2 * i + k] * n[sz3 * k + j] ;
      }
    }
  }
}

void mv_mult(const float m[], const float v[], float res[], int sz1, int sz2) {
  mm_mult(m, v, res, sz1, sz2, 1);
}

void sv_mult(float s, const float v[], float res[], int sz) {
  for (int i = 0; i < sz; i++) {
    res[i] = s * v[i];
  }
}

void m_print(const char msg[], const float m[], int sz1, int sz2) {
  int p = 0;
  char buf[10];

  while (msg[p] != 0) {
    printg("%c", msg[p]);
    p++;
  }
  for (int j = 0; j < sz2; j++) {
    printg("%7.3f\t", m[j]);
  }
  printg("\n\r");

  for (int i = 1; i < sz1; i++) {
    for (int j = 0; j < p; j++) {
      printg(" ");
    }
    for (int j = 0; j < sz2; j++) {
      printg("%7.3f\t", m[sz2 * i + j]);
    }
    printg("\n\r");
  }
}

// matrix inversion
// WARNING: A is modified in the process (transformed to the unit matrix through the Gauss-Jordan algorithm)
void inv(float A[], float I[], int sz) {

  for (int i = 0; i < sz; i++) {
    for (int j = 0; j < sz; j++) {
      I[i * sz + j] = (i == j ? 1 : 0);
    }
  }

  //m_print("A: ", (float*)A, sz, sz);
  //m_print("I: ", (float*)I, sz, sz);

  int r = 0;

  for (int j = 0; j < sz; j++) {
    int k = 0;
    float val_max = 0;
    //log_("j=%d", j);
    for (int i = r; i < sz; i++) {
      if (abs(A[i * sz + j]) > val_max) {
        val_max = abs(A[i * sz + j]);
        k = i;
      }
    }
    float pivot = A[k * sz + j];
    //log_(" k=%d pivot=%f", k, pivot);
    if (val_max > 0) {
      for (int i = 0; i < sz; i++) {
        A[k * sz + i] /= pivot;
        I[k * sz + i] /= pivot;
      }
      if (k != r) {
        //log_(" k!=r: k=%d r=%d", k, r);
        float tmp;
        for (int i = 0; i < sz; i++) {
          tmp = A[r * sz + i];
          A[r * sz + i] = A[k * sz + i];
          A[k * sz + i] = tmp;
          tmp = I[r * sz + i];
          I[r * sz + i] = I[k * sz + i];
          I[k * sz + i] = tmp;
        }

      }
      //m_print("A: ", (float*)A,sz,sz);
      //m_print("I: ", (float*)I,sz,sz);
      //log_("reduction");
      for (int i = 0; i < sz; i++) {
        if (i != r) {
          float tmp = A[i * sz + j];
          for (int l = 0; l < sz; l++) {
            A[i * sz + l] = A[i * sz + l] - A[r * sz + l] * tmp;
            I[i * sz + l] = I[i * sz + l] - I[r * sz + l] * tmp;
          }
        }
      }
      //m_print("A: ", (float*)A,sz,sz);
      //m_print("I: ", (float*)I,sz,sz);
      r = r + 1;
    }
  }
  //m_print("A: ", (float*)A, sz, sz);
  //m_print("I: ", (float*)I, sz, sz);
}





void m_def(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22, float res[3][3]) {
  res[0][0] = a00;
  res[0][1] = a01;
  res[0][2] = a02;
  res[1][0] = a10;
  res[1][1] = a11;
  res[1][2] = a12;
  res[2][0] = a20;
  res[2][1] = a21;
  res[2][2] = a22;
}

void v_def(float a0, float a1, float a2, float res[3]) {
  res[0] = a0;
  res[1] = a1;
  res[2] = a2;
}

void transpose(const float m[], float res[], int sz1, int sz2) {
  for (int i = 0; i < sz1; i++) {
    for (int j = 0; j < sz2; j++) {
      res[j * sz1 + i] = m[i * sz2 + j];
    }
  }
}
