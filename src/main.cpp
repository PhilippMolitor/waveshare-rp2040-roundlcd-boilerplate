#include <Arduino.h>

#include "pinout.h"
#include "battery.hpp"
#include "lgfx_gc9a01.hpp"

//
// application state
//
static float state_battery_voltage = 0.0f;

// battery
static Battery battery;
// imu
// TODO
// display
static LGFX_GC9A01 display;

void battery_tick()
{
  battery.read();
  state_battery_voltage = battery.get_voltage();
}

void imu_tick()
{
}

void display_tick()
{
  static auto c = false;

  display.startWrite();

  if (!c)
  {
    display.fillCircle(120, 120, 30, TFT_RED);
    display.fillCircle(120, 120, 20, TFT_GREEN);
    display.fillCircle(120, 120, 10, TFT_BLUE);
    c = true;
  }

  // voltage to string
  char voltage_str[6];
  sprintf(voltage_str, "%.2f V", state_battery_voltage);
  display.drawCenterString(voltage_str, 120, 170);

  display.endWrite();
}

void setup()
{
  Serial.begin(115200);

  // initialize peripherals
  battery.begin(PIN_BAT_ADC);
  display.begin(PIN_LCD_CLK,
                PIN_LCD_MOSI,
                -1,
                PIN_LCD_DC,
                PIN_LCD_CS,
                PIN_LCD_RST,
                PIN_LCD_BL);
}

void loop()
{
  // TODO: split these tick tasks on timers so they run via interrupts

  // 24hz maybe?
  display_tick();
  // should update at its native polling rate
  imu_tick();
  // every second maybe?
  battery_tick();

  // TODO: remove this
  delay(500);
}
