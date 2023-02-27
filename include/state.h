#pragma once

template <class T>
struct vec3_t {
  T x;
  T y;
  T z;
};

typedef struct imu_data_t {
  bool ready;
  float temp;
  vec3_t<float> acc;
  vec3_t<float> gyro;
} imu_data_t;

typedef struct battery_data_t {
  float voltage;
  float percentage;
} battery_data_t;

typedef struct state_t {
  imu_data_t imu;
  battery_data_t battery;
} state_t;
