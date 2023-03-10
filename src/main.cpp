#include <Arduino.h>
#include <Wire.h>
#include <pico/stdlib.h>

// project-provided libs
#include <qmi8658c.hpp>

// project-provided includes
#include "state.h"

#include "battery.hpp"
#include "lgfx_gc9a01.hpp"

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
            .percentage = 0.0f,
        },
};

static Battery battery;
static QMI8658C imu;
static LGFX_GC9A01 display;

struct repeating_timer timerDisplay;
struct repeating_timer timerBattery;

void batteryTick() {
  battery.update();
  battery.voltage(&state.battery.voltage);
  battery.percentage(&state.battery.percentage);
}

void imuTick() {
  if (!state.imu.ready)
    return;

  imu.readTemperature(&state.imu.temp);
  imu.readAccelerometer(&state.imu.acc.x, &state.imu.acc.y, &state.imu.acc.z);
  imu.readGyroscope(&state.imu.gyro.x, &state.imu.gyro.y, &state.imu.gyro.z);
}

void displayTick() {
  display.startWrite();

  // voltage
  char bat_v_str[10];
  char bat_ptc_str[10];
  int8_t bat_ptc = round(state.battery.percentage * 100);
  sprintf(bat_v_str, "Bat: %.2f V", state.battery.voltage);
  sprintf(bat_ptc_str, "%d %%", bat_ptc);
  display.drawRightString(bat_v_str, 228, 70);
  display.drawRightString(bat_ptc_str, 228, 80);

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
  {
    // TODO: PIN_BAT_ADC from the PlatformIO target def is 25, should be 29
    battery.begin(29);
  }

  // initialize IMU
  {
    // set up I2C1
    Wire1.setSDA(PIN_IMU_SDA);
    Wire1.setSCL(PIN_IMU_SCL);
    Wire1.setClock(400'000);
    Wire1.begin();
    // set up IMU driver
    state.imu.ready = imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
    if (state.imu.ready) {
      imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G,
                       QMI8658C::AccODR::ACC_ODR_250HZ,
                       QMI8658C::AccLPF::ACC_LPF_5_32PCT);
      imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_512DPS,
                        QMI8658C::GyroODR::GYRO_ODR_250HZ,
                        QMI8658C::GyroLPF::GYRO_LPF_5_32PCT);
      attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT2), imuTick,
                      PinStatus::RISING);
    }
  }

  // initialize display
  {
    display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS,
                  PIN_LCD_RST, PIN_LCD_BL);
  }

  // attach timer callbacks for regular peripheral updates
  {
    // display at 24hz
    add_repeating_timer_ms(
        1000 / 24,
        [](struct repeating_timer* t) {
          displayTick();
          return true;
        },
        NULL, &timerDisplay);

    // battery monitor at 2hz
    add_repeating_timer_ms(
        1000 / 2,
        [](struct repeating_timer* t) {
          batteryTick();
          return true;
        },
        NULL, &timerBattery);
  }
}

void loop() {
  delay(100);
}
