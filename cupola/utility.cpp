#include "utility.h"

#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>

void log_e(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.print("E: ");
  Serial.println(buf);
}
void log_w(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.print("W: ");
  Serial.println(buf);
}
void log_i(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.print("I: ");
  Serial.println(buf);
}
void log_d(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.print("D: ");
  Serial.println(buf);
}

void log_(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.println(buf);
}

void printg(const char *fmt, ... ) {
  char buf[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 64, fmt, args);
  va_end (args);
  Serial.print(buf);
}
