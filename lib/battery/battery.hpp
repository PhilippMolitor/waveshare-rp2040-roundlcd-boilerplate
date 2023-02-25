#pragma once

class Battery {
 private:
  pin_size_t m_adc_pin;
  unsigned int m_adc_value = 0;

  float m_voltage_ref = 3.30f;
  float m_voltage_divider = 0.5;

  float mapf(float x, float in_min, float in_max, float out_min, float out_max);

 public:
  void begin(pin_size_t adc_pin);

  void set_voltage_ref(float voltage);
  void set_voltage_divider(float divisor);

  /// read out the ADC pin of the voltage sensor
  void read();
  /// calculate the power source voltage in relation to the voltage reference,
  /// divided by the voltage divider invoke `Battery::read()` beforehand so the
  /// value is up to date.
  float get_voltage();
};
