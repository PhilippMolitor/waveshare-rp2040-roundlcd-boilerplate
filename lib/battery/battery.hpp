#pragma once

#include <Arduino.h>
#include <functional>

#define BATTERY_LAMBDA_PERCENTAGE_LIPO [](float v) { return v / 5; }
#define BATTERY_LAMBDA_PERCENTAGE_LIION [](float v) { return v / 5; }

typedef std::function<float(float voltage)> battery_percentage_lambda_t;

class Battery {
 private:
  pin_size_t m_adcPin;
  unsigned int m_adcValue = 0;

  float m_voltageRef = 3.30f;
  float m_voltageDivider = 0.5;
  battery_percentage_lambda_t m_percentageLambda;

  float mapf(float x, float in_min, float in_max, float out_min, float out_max);

 public:
  Battery();

  void begin(pin_size_t adc_pin);

  void setVoltageRef(float voltage);
  void setVoltageDivider(float divisor);
  void setPercentageCalculator(battery_percentage_lambda_t calculator);

  /// read out the ADC pin of the voltage sensor
  void update();
  /// calculate the power source voltage in relation to the voltage reference,
  /// divided by the voltage divider invoke `Battery::read()` beforehand so the
  /// value is up to date.
  void voltage(float* value);
  void percentage(float* value);
};
