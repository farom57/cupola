#include "settings.h"
#include "utility.h"
#include "KVStore.h"
#include "kvstore_global_api.h"



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
float st_sigma_acc;
float st_sigma_mag;

struct setting settings[ST_NB] {
  {"/kv/st_startup", &st_startup, INT},
  {"/kv/st_compass_bias", st_compass_bias, FLOAT3},
  {"/kv/st_compass_amp", st_compass_amp, FLOAT3},
  {"/kv/st_compass_rot", st_compass_rot, FLOAT9},
  {"/kv/st_compass_heading_bias", &st_compass_heading_bias, FLOAT},
  {"/kv/st_lat", &st_lat, FLOAT},
  {"/kv/st_ref_mag", st_ref_mag, FLOAT3},
  {"/kv/st_A_mag_inv", st_A_mag_inv, FLOAT9},
  {"/kv/st_A_acc_inv", st_A_acc_inv, FLOAT9},
  {"/kv/st_bias_mag", st_bias_mag, FLOAT3},
  {"/kv/st_bias_acc", st_bias_acc, FLOAT3},
  {"/kv/st_sigma_acc", &st_sigma_acc, FLOAT},
  {"/kv/st_sigma_mag", &st_sigma_mag, FLOAT},
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

  for (int i = 0; i < ST_NB; i++) {
    switch (settings[i].type) {
      case INT:

        readIntSt(settings[i].key, (int*)(settings[i].ptr));
        break;
      case FLOAT:

        readFloatSt(settings[i].key, (float*)(settings[i].ptr));
        break;
      case FLOAT3:

        readFloat3St(settings[i].key, (float*)(settings[i].ptr));
        break;
      case FLOAT9:

        readFloat9St(settings[i].key, (float*)(settings[i].ptr));
        break;
    }
  }

  st_startup++;
  saveIntSt(settings[0].key, st_startup);
  //log_d("%s=%i", st_keys[0], kv_startup);


}


void saveAllSt() {

  for (int i = 0; i < ST_NB; i++) {
    switch (settings[i].type) {
      case INT:

        saveIntSt(settings[i].key, *(int*)(settings[i].ptr));
        break;
      case FLOAT:

        saveFloatSt(settings[i].key, *(float*)(settings[i].ptr));
        break;
      case FLOAT3:

        saveFloat3St(settings[i].key, (float*)(settings[i].ptr));
        break;
      case FLOAT9:

        saveFloat9St(settings[i].key, (float*)(settings[i].ptr));
        break;
    }
  }
}

// set default settings
void defaultSt() {
  st_startup = 0;
  st_compass_bias[0] = 12.3;
  st_compass_bias[1] = 45.6;
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
  log_i("Saving setting %s=%s (size=%d)", key, tmp, len);


  //res = kv_set(key, tmp, len + 1, 0);
  res = kv_set(key, tmp, ST_VAL_LEN, 0);
  if (res != 0) {
    log_e("Error during setting saving. Error code: %d", res);
  }

}




// read the setting and store the result in val char array. The number of char read is returned
int readSt(const char* key, char* val) {
  int res;
  size_t actual_size;
  res = kv_get(key, val, ST_VAL_LEN, &actual_size);
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
  res = kv_get(key, buf, ST_VAL_LEN, &actual_size);
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
  res = kv_get(key, buf, ST_VAL_LEN, &actual_size);
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

void readFloat9St(const char* key, float val[9]) {
  char buf[ST_VAL_LEN] = {0};
  size_t actual_size;
  int res = kv_get(key, buf, ST_VAL_LEN, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return;
  }
  const char * separators = " ,!";
  char * strToken = strtok ( buf, separators );
  val[0] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[1] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[2] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[3] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[4] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[5] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[6] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[7] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[8] = atof(strToken);
  strToken = strtok ( NULL, separators );
}
void readFloat3St(const char* key, float val[3]) {
  char buf[ST_VAL_LEN] = {0};
  size_t actual_size;
  int res = kv_get(key, buf, ST_VAL_LEN, &actual_size);
  if (res != 0) {
    log_e("Error while reading %s = %s. Actual_size: %d Error code: %d", key, buf, actual_size, res);
    return;
  }
  const char * separators = " ,!";
  char * strToken = strtok ( buf, separators );
  val[0] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[1] = atof(strToken);
  strToken = strtok ( NULL, separators );
  val[2] = atof(strToken);
  strToken = strtok ( NULL, separators );
}

// save setting
void saveFloatSt(const char* key, float val) {
  char buf[ST_VAL_LEN];
  sprintf(buf, "%.9g", val);
  saveSt(key, buf, strlen(buf));
}

// save setting
void saveFloat9St(const char* key, float val[9]) {
  char buf[ST_VAL_LEN] = {0};
  sprintf(buf, "%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g", val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
  saveSt(key, buf, strlen(buf));
}
void saveFloat3St(const char* key, float val[3]) {
  char buf[ST_VAL_LEN] = {0};


  sprintf(buf, "%.9g %.9g %.9g", val[0], val[1], val[2]);
  // log_d("key=%s",key);
  // log_d(buf);

  saveSt(key, buf, strlen(buf));
}

// save setting
void saveIntSt(const char* key, int val) {
  char buf[ST_VAL_LEN];
  sprintf(buf, "%d", st_startup);
  saveSt(key, buf, strlen(buf));
}
