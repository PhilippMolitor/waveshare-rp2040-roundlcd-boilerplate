#pragma once

#include <Arduino.h>
#include <functional>

#define BATTERY_LAMBDA_PERCENTAGE_LIPO [](float v) { return v / 5; }
#define BATTERY_LAMBDA_PERCENTAGE_LIION [](float v) { return v / 5; }

typedef std::function<float(float voltage)> battery_percentage_lambda_t;

class Battery {
 private:
  pin_size_t m_adc_pin;
  unsigned int m_adc_value = 0;

  float m_voltage_ref = 3.30f;
  float m_voltage_divider = 0.5;
  battery_percentage_lambda_t m_percentage_lambda;

  float mapf(float x, float in_min, float in_max, float out_min, float out_max);

 public:
  Battery();

  void begin(pin_size_t adc_pin);

  void set_voltage_ref(float voltage);
  void set_voltage_divider(float divisor);
  void set_percentage_calculator(battery_percentage_lambda_t calculator);

  /// read out the ADC pin of the voltage sensor
  void update();
  /// calculate the power source voltage in relation to the voltage reference,
  /// divided by the voltage divider invoke `Battery::read()` beforehand so the
  /// value is up to date.
  void voltage(float* value);
  void percentage(float* value);
};
