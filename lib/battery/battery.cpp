#include <Arduino.h>

#include "battery.hpp"

float Battery::mapf(float x,
                    float in_min,
                    float in_max,
                    float out_min,
                    float out_max) {
  return constrain(
      (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min,
      out_max);
}

void Battery::begin(pin_size_t adc_pin) {
  m_adc_pin = adc_pin;
}

void Battery::set_voltage_ref(float voltage) {
  m_voltage_ref = voltage;
}

void Battery::set_voltage_divider(float divisor) {
  if (divisor <= 0.0f)
    return;
  m_voltage_divider = divisor;
}

void Battery::read() {
  analogReadResolution(12);
  m_adc_value = analogRead(m_adc_pin);
}

float Battery::get_voltage() {
  // map adc value to voltage by reference and divide by voltage divider
  return mapf(m_adc_value, 0.0f, 4095.0f, 0.0f, m_voltage_ref) /
         m_voltage_divider;
}
