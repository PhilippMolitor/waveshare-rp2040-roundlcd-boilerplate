#pragma once

#include <Arduino.h>

class Battery
{
private:
  uint8_t adc_pin;
  int adc_value = 0;

  float m_voltage_ref = 3.30f;
  float m_voltage_min = 3.50f;
  float m_voltage_max = 4.20f;
  float m_voltage_divider = 0.5;

public:
  Battery();

  void begin(uint8_t adc_pin);

  void set_voltage_ref(float voltage);
  void set_voltage_min(float voltage);
  void set_voltage_max(float voltage);
  void set_voltage_divider(float divisor);

  void read();
  float get_voltage();
  float get_percentage();
};
