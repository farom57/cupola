#ifndef VECTOR_H
#define VECTOR_H
#include "utility.h"

float norm(const float v[]);
float norm2(const float v[]); // =norm^2
void normalize(const float v[], float res[]);
void m_copy(const float m[], float res[]);
void v_copy(const float v[], float res[]);
void mm_mult(const float m[], const float n[], float res[]);
void mm_mult(const float m[3][3], const float n[3][3], float res[3][3]);
void mv_mult(const float m[], const float v[], float res[]);
void mv_mult(const float m[3][3], const float v[3], float res[3]);
void sv_mult(float s, const float v[], float res[]);
void v_lincomb(float a, const float u[], float b, const float v[], float res[]); // a*u+b*v
void sm_mult(float s, const float m[], float res[]);
void transpose(const float m[], float res[]);
void transpose(const float m[3][3], float res[3][3]);
float scalar_prod(const float u[], const float v[]);
void vect_prod(const float u[], const float v[], float res[]);
void v_add(const float u[], const float v[], float res[]);
void v_sub(const float u[], const float v[], float res[]); //u-v
void vect_cat(const float u[], const float v[], const float w[], float res[]);
void v_print(const char msg[], const float v[]);
void m_print(const char msg[], const float m[]);
void rotx(float angle, float res[3][3]);
void roty(float angle, float res[3][3]);
void rotz(float angle, float res[3][3]);
void tel2topo(float lat, float ha_rot, float dec_rot, float res[3][3]);
void print_degstr(float rad);
void m_def(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22, float res[3][3]);
void v_def(float a0, float a1, float a2, float res[3]);

float norm(const float v[], int sz);
float norm2(const float v[], int sz); // =norm^2
void normalize(const float v[], const float res[], int sz);
void m_copy(const float m[], float res[], int sz);
void v_copy(const float v[], float res[], int sz);
void mm_mult(const float m[], const float n[], float res[], int sz1, int sz2, int sz3);
void mv_mult(const float m[], const float v[], float res[], int sz1, int sz2);
void sv_mult(float s, const float v[], float res[], int sz);
void m_print(const char msg[], const float m[], int sz1, int sz2);
void inv(float A[], float I[], int sz);
void transpose(const float m[], float res[], int sz1, int sz2);

//solve A.x=b (pseudosolution)
// A[rows,cols] b[rows] x[cols]
template <size_t rows, size_t cols> void psolve(const float (&A)[rows][cols], const float b[], float x[]) {
  // Compute pseudo-solution beta=(A' * A)^-1 * A' * b

  float B[cols][cols];
  float Binv[cols][cols];
  float At[cols][rows]; //=A'
  float tmp[cols];
  float tmp2[cols][cols];
  transpose((const float*)A, (float*)At, rows, cols);
  mm_mult((const float*)At, (const float*)A, (float*)B, cols, rows, cols); // B = A' * A
  m_copy((const float*)B, (float*)tmp2, cols * cols); inv((float*)tmp2, (float*)Binv, cols); // Binv = inv(B)
  mv_mult((const float*)At, (const float*)b, tmp, cols, rows); // tmp = A' * b
  mv_mult((const float*)Binv, (const float*)tmp, x, cols, cols); // x = Binv * tmp

  for (int i = 0; i < cols; i++) {
    x[i] = 0;
    for (int j = 0; j < rows; j++) {
      for (int k = 0; k < cols; k++) {
        x[i] += Binv[i][k] * A[j][k] * b[j];
      }
    }
  }
}


/*void v_lincomb(float a, float u[], float b, float v[], float res[]); // a*u+b*v
  void sm_mult(float s, float m[], float res[]);
  void transpose(float m[], float res[]);
  float scalar_prod(float u[], float v[]);
  void vect_prod(float u[], float v[], float res[]);
  void v_add(float u[], float v[], float res[]);
  void v_sub(float u[], float v[], float res[]); //u-v
  void vect_cat(float u[], float v[], float w[], float res[]);
  void v_print(const char msg[], float v[]);
  void m_print(const char msg[], float m[]);
  void rotx(float angle, float res[]);
  void print_degstr(float rad);
  void print_hourstr(float rad);*/


#endif
