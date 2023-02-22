#include <Arduino.h>

#include "pinout.h"
#include "display.hpp"
#include "battery.hpp"

//
// settings
//

// battery voltage divider calibration
// #define CONFIG_VBAT_ADC_DIVIDER (0.5)

//
// constants
//

// TODO

//
// application state
//

// battery
static Battery battery;
// imu
// TODO
// display
static Display display;

void battery_init()
{
  battery.begin(PIN_BAT_ADC);
#ifdef CONFIG_VBAT_ADC_DIVIDER
  battery.set_voltage_divider(CONFIG_VBAT_ADC_DIVIDER);
#endif
}

void battery_tick() {}

void imu_init() {}

void imu_tick() {}

void display_init()
{
  display.begin();
}

void display_tick()
{
  display.startWrite();

  display.fillCircle(120, 120, 30, TFT_GREEN);

  display.endWrite();
}

void setup()
{
  // initialize peripherals
  battery_init();
  imu_init();
  display_init();
}

void loop()
{
  // 24hz maybe?
  display_tick();
}

void loop2()
{
  // should update at its native polling rate
  imu_tick();

  // every second maybe?
  battery_tick();
}
