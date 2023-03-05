#include <Arduino.h>

#include "battery.hpp"

Battery::Battery() {
  m_percentageLambda = BATTERY_LAMBDA_PERCENTAGE_LIPO;
}

void Battery::begin(pin_size_t adc_pin) {
  m_adcPin = adc_pin;
}

void Battery::setVoltageRef(float voltage) {
  m_voltageRef = voltage;
}

void Battery::setVoltageDivider(float divisor) {
  if (divisor <= 0.0f)
    return;
  m_voltageDivider = divisor;
}

void Battery::setPercentageCalculator(battery_percentage_lambda_t calculator) {
  m_percentageLambda = calculator;
}

void Battery::update() {
  analogReadResolution(12);
  m_adcValue = analogRead(m_adcPin);
}

void Battery::voltage(float* value) {
  // map adc value to voltage by reference and divide by voltage divider
  *value =
      mapf(m_adcValue, 0.0f, 4095.0f, 0.0f, m_voltageRef) / m_voltageDivider;
}

void Battery::percentage(float* value) {
  float v = 0.0f;
  voltage(&v);

  *value = constrain(m_percentageLambda(v), 0.0f, 1.0f);
}
