#include "settings.h"
#include "utility.h"
#include "KVStore.h"
#include "kvstore_global_api.h"

char st_keys[ST_NB][ST_KEY_LEN] = {"/kv/startup", "/kv/test_int", "/kv/test_float", "/kv/test_string"};
int kv_startup = 0;
int kv_test_int = 0;
float kv_test_float = 0.0;
char kv_test_string[ST_VAL_LEN] = {""};

// reset setting storage
void resetSt() {
  log_i("Reset KV storage");
  kv_reset("/kv/");
}

// load setings from the flash
void loadSt() {
  defaultSt();
  readIntSt(st_keys[0], &kv_startup);
  readIntSt(st_keys[1], &kv_test_int);
  readFloatSt(st_keys[2], &kv_test_float);
  readSt(st_keys[3], kv_test_string);

  kv_startup++;
  char buf[ST_VAL_LEN];
  sprintf(buf, "%d", kv_startup);
  saveSt(st_keys[0], buf, strlen(buf));
  //log_d("%s=%i", st_keys[0], kv_startup);

}


// set default settings
void defaultSt() {

}


// save setting
void saveSt(const char* key, const char* val, int len) {
  int res;
  char tmp[ST_VAL_LEN];
  
  // force zero trailling
  len = min(len, ST_VAL_LEN - 1);
  memcpy(tmp, val, len);
  tmp[len] = 0;

  log_i("Saving setting %s=%s", key, tmp);
  
  res = kv_set(key, tmp, len + 1, 0);
  if (res != 0) {
    log_e("Error during setting saving. Error code: %d", res);
  }
}


// read the setting and store the result in val char array. The number of char read is returned
int readSt(const char* key, char* val) {
  int res;
  size_t actual_size;
  res = kv_get(key, val, ST_KEY_LEN, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, val, actual_size, res);
  }
  return actual_size;
}


// read the setting and store the result in val. In case of error, val is not modified and false is returned
bool readIntSt(const char* key, int* val) {
  int tmp;
  int res;
  char buf[ST_VAL_LEN];
  size_t actual_size;
  res = kv_get(key, buf, ST_KEY_LEN, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return false;
  }
  res = sscanf(buf, "%i", &tmp);
  if (res == 1) {
    *val = tmp;
    return true;
  } else {
    log_w("Error while reading %s = %s. Incorrect formating", key, buf);
    char err[] = "format error";
    res = kv_set(key, err, strlen(err) + 1, 0);
    return false;
  }
}


// read the setting and store the result in val. In case of error, val is not modified and false is returned
bool readFloatSt(const char* key, float* val) {
  float tmp;
  int res;
  char buf[ST_VAL_LEN];
  size_t actual_size;
  res = kv_get(key, buf, ST_KEY_LEN, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return false;
  }
  res = sscanf(buf, "%f", &tmp);
  if (res == 1) {
    *val = tmp;
    return true;
  } else {
    log_w("Error while reading %s = %s. Incorrect formating", key, buf);
    char err[] = "format error";
    res = kv_set(key, err, strlen(err) + 1, 0);
    return false;
  }
}
