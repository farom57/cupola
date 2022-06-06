#ifndef UTLITY_H
#define UTLITY_H

// different error message level, line end added automaticaly
void log_e(const char *fmt, ... );
void log_w(const char *fmt, ... );
void log_i(const char *fmt, ... );
void log_d(const char *fmt, ... );
void log_(const char *fmt, ... );
void printg(const char *fmt, ... );
void v_copy(const float v[], float res[]);
void v_lincomb(float a, const float u[], float b, const float v[], float res[]);


#endif
