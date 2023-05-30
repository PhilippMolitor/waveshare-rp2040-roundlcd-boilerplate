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

template <typename T>
struct vec3 {
  T x;
  T y;
  T z;
};

template <typename T>
using vec3_t = vec3<T>;

typedef struct imu_data {
  bool ready;
  float temp;
  vec3_t<float> acc;
  vec3_t<float> gyro;
} imu_data_t;

typedef struct battery_data {
  float voltage;
  float percentage;
} battery_data_t;

typedef struct state {
  uint32_t counter;
  imu_data_t imu;
  battery_data_t battery;
} state_t;
