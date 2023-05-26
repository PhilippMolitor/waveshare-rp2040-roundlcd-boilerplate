#pragma once

#include <Arduino.h>
#include <pico/mutex.h>

#define SAFE_STATE_UPDATE(mtx, statements) \
  {                                        \
    noInterrupts();                        \
    mutex_enter_blocking(mtx);             \
    statements;                            \
    mutex_exit(mtx);                       \
    interrupts();                          \
  }

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
  uint32_t counter;
  imu_data_t imu;
  battery_data_t battery;
} state_t;
