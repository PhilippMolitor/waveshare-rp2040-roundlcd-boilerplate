#include <Arduino.h>
#include <Wire.h>

#include "pinout.h"
#include "state.h"

#include "battery.hpp"
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"

//
// application state
//
state_t state = {
    .imu =
        {
            .ready = false,
            .temp = 0.0f,
            .acc = {.x = 0.0f, .y = 0.0f, .z = 0.0f},
            .gyro = {.x = 0.0f, .y = 0.0f, .z = 0.0f},
        },
    .battery =
        {
            .voltage = 0.0f,
        },
};

// battery
static Battery battery;
// imu
static QMI8658C imu;
// display
static LGFX_GC9A01 display;

void battery_tick() {
  battery.update();
  battery.voltage(&state.battery.voltage);
}

void imu_tick() {
  if (!state.imu.ready)
    return;

  imu.read_temperature(&state.imu.temp);
  imu.read_accelerometer(&state.imu.acc.x, &state.imu.acc.y, &state.imu.acc.z);
  imu.read_gyroscope(&state.imu.gyro.x, &state.imu.gyro.y, &state.imu.gyro.z);
}

void display_tick() {
  display.startWrite();

  // voltage
  char voltage_str[10];
  sprintf(voltage_str, "Bat: %.2f V", state.battery.voltage);
  display.drawRightString(voltage_str, 228, 70);

  // IMU temp
  char imu_temp_str[20];
  sprintf(imu_temp_str, "IMU temp: %.2f C", state.imu.temp);
  display.drawString(imu_temp_str, 12, 70);
  // IMU data
  char imu_ax_str[18], imu_ay_str[18], imu_az_str[18];
  char imu_gx_str[21], imu_gy_str[21], imu_gz_str[21];
  sprintf(imu_ax_str, "ax: %.2f g", state.imu.acc.x);
  sprintf(imu_ay_str, "ay: %.2f g", state.imu.acc.y);
  sprintf(imu_az_str, "az: %.2f g", state.imu.acc.z);
  sprintf(imu_gx_str, "gx: %.2f dps", state.imu.gyro.x);
  sprintf(imu_gy_str, "gy: %.2f dps", state.imu.gyro.y);
  sprintf(imu_gz_str, "gz: %.2f dps", state.imu.gyro.z);
  display.drawString(imu_ax_str, 12, 90);
  display.drawString(imu_ay_str, 12, 100);
  display.drawString(imu_az_str, 12, 110);
  display.drawString(imu_gx_str, 12, 120);
  display.drawString(imu_gy_str, 12, 130);
  display.drawString(imu_gz_str, 12, 140);

  display.endWrite();
}

void setup() {
  Serial.begin(115200);

  // initialize battery adc
  battery.begin(PIN_BAT_ADC);
  // initialize IMU
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400'000);
  Wire1.begin();
  state.imu.ready = imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
  if (state.imu.ready) {
    imu.configure_acc(QMI8658C::AccScale::ACC_SCALE_4G,
                      QMI8658C::AccODR::ACC_ODR_250HZ,
                      QMI8658C::AccLPF::ACC_LPF_5_32PCT);
    imu.configure_gyro(QMI8658C::GyroScale::GYRO_SCALE_512DPS,
                       QMI8658C::GyroODR::GYRO_ODR_250HZ,
                       QMI8658C::GyroLPF::GYRO_LPF_5_32PCT);
    attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT_2), imu_tick,
                    PinStatus::RISING);
  }

  // initialize display
  display.begin(PIN_LCD_CLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);
}

void loop() {
  // TODO: split these tick tasks on timers so they run via interrupts

  // 24hz maybe?
  display_tick();
  // every second maybe?
  battery_tick();

  // TODO: remove this
  delay(1000 / 24);
}
