#pragma once

#include <Arduino.h>
#include <functional>

inline float mapf(float x,
                  float in_min,
                  float in_max,
                  float out_min,
                  float out_max) {
  return constrain(
      (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min,
      out_max);
}

#define BATTERY_LAMBDA_PERCENTAGE_LIPO \
  [](float v) { return mapf(v, 3.5f, 4.2f, 0.0f, 1.0f); }
#define BATTERY_LAMBDA_PERCENTAGE_LIION \
  [](float v) { return mapf(v, 3.0f, 4.2f, 0.0f, 1.0f); }

typedef std::function<float(float voltage)> battery_percentage_lambda_t;

class Battery {
 private:
  pin_size_t m_adcPin;
  unsigned int m_adcValue = 0;

  float m_voltageRef = 3.30f;
  float m_voltageDivider = 0.5;
  battery_percentage_lambda_t m_percentageLambda;

 public:
  Battery();

  void begin(pin_size_t adc_pin);

  void setVoltageRef(float voltage);
  void setVoltageDivider(float divisor);
  void setPercentageCalculator(battery_percentage_lambda_t calculator);

  /// read out the ADC pin of the voltage sensor
  void update();
  /// calculate the power source voltage in relation to the voltage reference,
  /// divided by the voltage divider.
  /// invoke `Battery::read()` beforehand so the value is up to date.
  void voltage(float* value);
  /// calculate the battery capacity percentage based on the percentage
  /// calculator. the default profile estimates a LiPo battery under very low
  /// amp draw.
  /// invoke `Battery::read()` beforehand so the value is up to date.
  void percentage(float* value);
};
