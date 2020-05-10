#ifndef VECTOR_H
#define VECTOR_H

float norm(float v[]);
float norm2(float v[]); // =norm^2
void normalize(float v[], float res[]);
void m_copy(float m[], float res[]);
void v_copy(float v[], float res[]);
void mm_mult(float m[], float n[], float res[]);
void mv_mult(float m[], float v[], float res[]);
void sv_mult(float s, float v[], float res[]);
void v_lincomb(float a, float u[], float b, float v[], float res[]); // a*u+b*v
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
void print_hourstr(float rad);

void test_math();

#endif
