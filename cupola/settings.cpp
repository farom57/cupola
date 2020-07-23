#include "settings.h"
#include "utility.h"
#include "KVStore.h"
#include "kvstore_global_api.h"
enum setting_type { INT, FLOAT, FLOAT3, FLOAT33, END };

struct setting{
  char key[ST_KEY_LEN];
  void* ptr;
  enum setting_type type;
};

char st_keys[ST_NB][ST_KEY_LEN] = {
  "/kv/startup",
  "/kv/compass_bias",  "/kv/compass_amp",  "/kv/compass_rot",  "/kv/compass_heading_bias",
  "/kv/lat",  "/kv/ref_mag", "/kv/A_mag_inv", "/kv/A_acc_inv", "/kv/bias_mag", "/kv/bias_acc"
};
int st_startup;
float st_compass_bias[3];
float st_compass_amp[3];
float st_compass_rot[3][3];
float st_compass_heading_bias;
float st_lat;
float st_ref_mag[3];
float st_A_mag_inv[3][3];
float st_A_acc_inv[3][3];
float st_bias_mag[3];
float st_bias_acc[3];

struct setting settings[]{
  {"/kv/st_startup", &st_startup, INT},
  {"/kv/st_compass_bias", &st_compass_bias, FLOAT3},
  {"/kv/st_compass_amp", &st_compass_amp, FLOAT3},
  {"/kv/st_compass_rot", &st_compass_rot, FLOAT33},
  {"/kv/st_compass_heading_bias", &st_compass_heading_bias, FLOAT},
  {"/kv/st_lat", &st_lat, FLOAT},
  {"/kv/st_ref_mag", & st_ref_mag, FLOAT3},
  {"/kv/st_A_mag_inv", &st_A_mag_inv, FLOAT33},
  {"/kv/st_A_acc_inv", &st_A_acc_inv, FLOAT33},
  {"/kv/st_bias_mag", &st_bias_mag, FLOAT3},
  {"/kv/st_bias_acc", &st_bias_acc, FLOAT3},
  {"", 0, END}
};

// reset setting storage
void resetSt() {
  log_i("Reset KV storage");
  kv_reset("/kv/");
  log_i("Restoring default values");
  defaultSt();
  saveAllSt();

}

// load setings from the flash
void loadSt() {
  defaultSt();
  readIntSt(st_keys[0], &st_startup);
//  readFloatSt(st_keys[1], &st_compass_bias_x);
//  readFloatSt(st_keys[2], &st_compass_bias_y);
//  readFloatSt(st_keys[3], &st_compass_amp_x);
//  readFloatSt(st_keys[4], &st_compass_amp_y);
//  readFloatSt(st_keys[5], &st_compass_amp_z);
  readFloatSt(st_keys[6], &st_compass_rot[0][0]);
  readFloatSt(st_keys[7], &st_compass_rot[0][1]);
  readFloatSt(st_keys[8], &st_compass_rot[0][2]);
  readFloatSt(st_keys[9], &st_compass_rot[1][0]);
  readFloatSt(st_keys[10], &st_compass_rot[1][1]);
  readFloatSt(st_keys[11], &st_compass_rot[1][2]);
  readFloatSt(st_keys[12], &st_compass_rot[2][0]);
  readFloatSt(st_keys[13], &st_compass_rot[2][1]);
  readFloatSt(st_keys[14], &st_compass_rot[2][2]);
  readFloatSt(st_keys[15], &st_compass_heading_bias);
  readFloatSt(st_keys[16], &st_lat);
  readFloatSt(st_keys[17], &st_ref_mag[0]);
  readFloatSt(st_keys[18], &st_ref_mag[1]);
  readFloatSt(st_keys[19], &st_ref_mag[2]);

  st_startup++;
  saveIntSt(st_keys[0], st_startup);
  //log_d("%s=%i", st_keys[0], kv_startup);

}


void saveAllSt() {
  saveIntSt(st_keys[0], st_startup);
//  saveFloatSt(st_keys[1], st_compass_bias_x);
//  saveFloatSt(st_keys[2], st_compass_bias_y);
//  saveFloatSt(st_keys[3], st_compass_amp_x);
//  saveFloatSt(st_keys[4], st_compass_amp_y);
//  saveFloatSt(st_keys[5], st_compass_amp_z);
  saveFloatSt(st_keys[6], st_compass_rot[0][0]);
  saveFloatSt(st_keys[7], st_compass_rot[0][1]);
  saveFloatSt(st_keys[8], st_compass_rot[0][2]);
  saveFloatSt(st_keys[9], st_compass_rot[1][0]);
  saveFloatSt(st_keys[10], st_compass_rot[1][1]);
  saveFloatSt(st_keys[11], st_compass_rot[1][2]);
  saveFloatSt(st_keys[12], st_compass_rot[2][0]);
  saveFloatSt(st_keys[13], st_compass_rot[2][1]);
  saveFloatSt(st_keys[14], st_compass_rot[2][2]);
  saveFloatSt(st_keys[15], st_compass_heading_bias);
  saveFloatSt(st_keys[16], st_lat);
  saveFloatSt(st_keys[17], st_ref_mag[0]);
  saveFloatSt(st_keys[18], st_ref_mag[1]);
  saveFloatSt(st_keys[19], st_ref_mag[2]);
}

// set default settings
void defaultSt() {
  st_startup = 0;
  st_compass_bias[0] = 0;
  st_compass_bias[1] = 0;
  st_compass_amp[0] = 0;
  st_compass_amp[1] = 0;
  st_compass_amp[2] = 0;
  st_compass_rot[0][0] = 1;
  st_compass_rot[0][1] = 0;
  st_compass_rot[0][2] = 0;
  st_compass_rot[1][0] = 0;
  st_compass_rot[1][1] = 1;
  st_compass_rot[1][2] = 0;
  st_compass_rot[2][0] = 0;
  st_compass_rot[2][1] = 0;
  st_compass_rot[2][2] = 1;
  st_compass_heading_bias = 0;
  st_lat = DEFAULT_LAT;
  st_ref_mag[0] = DEFAULT_MAG_X;
  st_ref_mag[1] = DEFAULT_MAG_Y;
  st_ref_mag[2] = DEFAULT_MAG_Z;
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

  errno = 0;
  tmp = atof(buf);
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

void readFloat3x3St(const char* key, float* val[3][3]){
  char buf[200]={0};
  size_t actual_size;
  int res = kv_get(key, buf, 200, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return;
  }
  sscanf(buf, "%.9g %.9g %.9g\n%.9g %.9g %.9g \n%.9g %.9g %.9g", val[0][0],val[0][1],val[0][2],val[1][0],val[1][1],val[1][2],val[2][0],val[2][1],val[2][2]);
}
void readFloat3St(const char* key, float* val[3]){
    char buf[100]={0};
  size_t actual_size;
  int res = kv_get(key, buf, 200, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return;
  }
  sscanf(buf, "%.9g %.9g %.9g", val[0],val[1],val[2]);
}

// save setting
void saveFloatSt(const char* key, float val) {
  char buf[ST_VAL_LEN];
  sprintf(buf, "%.9g", val);
  saveSt(key, buf, strlen(buf));
}

// save setting
void saveFloat3x3St(const char* key, float val[3][3]){
  char buf[200]={0};
  sprintf(buf, "%.9g %.9g %.9g\n%.9g %.9g %.9g \n%.9g %.9g %.9g", val[0][0],val[0][1],val[0][2],val[1][0],val[1][1],val[1][2],val[2][0],val[2][1],val[2][2]);
  saveSt(key, buf, strlen(buf));
}
void saveFloat3St(const char* key, float val[3]){
  char buf[100]={0};
  sprintf(buf, "%.9g %.9g %.9g", val[0],val[1],val[2]);
  saveSt(key, buf, strlen(buf));
}

// save setting
void saveIntSt(const char* key, int val) {
  char buf[ST_VAL_LEN];
  sprintf(buf, "%d", st_startup);
  saveSt(st_keys[0], buf, strlen(buf));
}


void saveCompassCalib() {
//  saveFloatSt(st_keys[1], st_compass_bias_x);
//  saveFloatSt(st_keys[2], st_compass_bias_y);
//  saveFloatSt(st_keys[3], st_compass_amp_x);
//  saveFloatSt(st_keys[4], st_compass_amp_y);
//  saveFloatSt(st_keys[5], st_compass_amp_z);
  saveFloatSt(st_keys[6], st_compass_rot[0][0]);
  saveFloatSt(st_keys[7], st_compass_rot[0][1]);
  saveFloatSt(st_keys[8], st_compass_rot[0][2]);
  saveFloatSt(st_keys[9], st_compass_rot[1][0]);
  saveFloatSt(st_keys[10], st_compass_rot[1][1]);
  saveFloatSt(st_keys[11], st_compass_rot[1][2]);
  saveFloatSt(st_keys[12], st_compass_rot[2][0]);
  saveFloatSt(st_keys[13], st_compass_rot[2][1]);
  saveFloatSt(st_keys[14], st_compass_rot[2][2]);
  saveFloatSt(st_keys[15], st_compass_heading_bias);
  loadSt();
}
