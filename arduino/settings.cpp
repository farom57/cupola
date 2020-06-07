#include "settings.h"
#include "utility.h"
#include "KVStore.h"
#include "kvstore_global_api.h"

char st_keys[ST_NB][ST_KEY_LEN] = {
  "/kv/startup",
  "/kv/compass_bias_x", "/kv/compass_bias_y",
  "/kv/compass_amp_x", "/kv/compass_amp_y", "/kv/compass_amp_z",
  "/kv/compass_rot_00", "/kv/compass_rot_01", "/kv/compass_rot_02", "/kv/compass_rot_10", "/kv/compass_rot_11", "/kv/compass_rot_12", "/kv/compass_rot_20", "/kv/compass_rot_21", "/kv/compass_rot_22"
};
int st_startup;
float st_compass_bias_x;
float st_compass_bias_y;
float st_compass_amp_x;
float st_compass_amp_y;
float st_compass_amp_z;
float st_compass_rot[3][3];


// reset setting storage
void resetSt() {
  log_i("Reset KV storage");
  kv_reset("/kv/");
}

// load setings from the flash
void loadSt() {
  defaultSt();
  readIntSt(st_keys[0], &st_startup);
  readFloatSt(st_keys[1], &st_compass_bias_x);
  readFloatSt(st_keys[2], &st_compass_bias_y);
  readFloatSt(st_keys[3], &st_compass_amp_x);
  readFloatSt(st_keys[4], &st_compass_amp_y);
  readFloatSt(st_keys[5], &st_compass_amp_z);
  readFloatSt(st_keys[6], &st_compass_rot[0][0]);
  readFloatSt(st_keys[7], &st_compass_rot[0][1]);
  readFloatSt(st_keys[8], &st_compass_rot[0][2]);
  readFloatSt(st_keys[9], &st_compass_rot[1][0]);
  readFloatSt(st_keys[10], &st_compass_rot[1][1]);
  readFloatSt(st_keys[11], &st_compass_rot[1][2]);
  readFloatSt(st_keys[12], &st_compass_rot[2][0]);
  readFloatSt(st_keys[13], &st_compass_rot[2][1]);
  readFloatSt(st_keys[14], &st_compass_rot[2][2]);


  st_startup++;
  char buf[ST_VAL_LEN];
  sprintf(buf, "%d", st_startup);
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

  errno=0;
  tmp=atof(buf);
  if (errno == 0) {
    *val = tmp;
    return true;
  } else {
    log_w("Error while reading %s = %s. Incorrect formating %d %f", key, buf, errno, tmp);
    char err[] = "format error";
    res = kv_set(key, err, strlen(err) + 1, 0);
    return false;
  }
}

// save setting
void saveFloatSt(const char* key, float val) {
  char buf[ST_VAL_LEN];
  sprintf(buf, "%.9g", val);
  saveSt(key, buf, strlen(buf));
}

void saveCompassCalib() {
  saveFloatSt(st_keys[1], st_compass_bias_x);
  saveFloatSt(st_keys[2], st_compass_bias_y);
  saveFloatSt(st_keys[3], st_compass_amp_x);
  saveFloatSt(st_keys[4], st_compass_amp_y);
  saveFloatSt(st_keys[5], st_compass_amp_z);
  saveFloatSt(st_keys[6], st_compass_rot[0][0]);
  saveFloatSt(st_keys[7], st_compass_rot[0][1]);
  saveFloatSt(st_keys[8], st_compass_rot[0][2]);
  saveFloatSt(st_keys[9], st_compass_rot[1][0]);
  saveFloatSt(st_keys[10], st_compass_rot[1][1]);
  saveFloatSt(st_keys[11], st_compass_rot[1][2]);
  saveFloatSt(st_keys[12], st_compass_rot[2][0]);
  saveFloatSt(st_keys[13], st_compass_rot[2][1]);
  saveFloatSt(st_keys[14], st_compass_rot[2][2]);
  loadSt();
}
