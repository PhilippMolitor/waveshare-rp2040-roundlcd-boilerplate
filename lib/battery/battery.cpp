#include "battery.hpp"

Battery::Battery()
{
}

void Battery::begin(uint8_t adc_pin)
{
  this->adc_pin = adc_pin;
  pinMode(adc_pin, PinMode::INPUT);
}

void Battery::set_voltage_ref(float voltage)
{
  this->m_voltage_ref = voltage;
}

void Battery::set_voltage_min(float voltage)
{
  if (voltage >= this->m_voltage_max)
    return;
  this->m_voltage_min = voltage;
}

void Battery::set_voltage_max(float voltage)
{
  if (voltage <= this->m_voltage_min)
    return;
  this->m_voltage_max = voltage;
}

void Battery::set_voltage_divider(float divisor)
{
  if (divisor <= 0.0f)
    return;
  this->m_voltage_divider = divisor;
}

void Battery::read()
{
  analogReadResolution(12);
  this->adc_value = analogRead(this->adc_pin);
}

float Battery::get_voltage()
{
  // map adc value to voltage by reference
  float voltage = map(this->adc_value, 0.0f, 4096.0f, 0.0f, this->m_voltage_ref);
  // apply voltage divider correction
  voltage = voltage / this->m_voltage_divider;

  return voltage;
}

float Battery::get_percentage()
{
  float voltage = this->get_voltage();
  float percentage = map(voltage, this->m_voltage_min, this->m_voltage_max, 0.0f, 1.0f);

  return percentage;
}