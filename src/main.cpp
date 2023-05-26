#include <Arduino.h>
#include <Wire.h>
#include <pico/mutex.h>
#include <pico/stdlib.h>

// project-provided libs
#include <qmi8658c.hpp>

// project-provided includes
#include "state.h"

// project source
#include "battery.hpp"
#include "lgfx_gc9a01.hpp"

//
// application config
//
#define CONFIG_DISPLAY_UPDATE_RATE_HZ (24u)
#define CONFIG_BATTERY_UPDATE_RATE_HZ (2u)

//
// application state
//
mutex stateMtx;

volatile state_t state = {
    .counter = 0,
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

repeating_timer timerDisplay;
repeating_timer timerBattery;

void batteryTick() {
  battery.update();

  SAFE_STATE_UPDATE(&stateMtx, {
    battery.voltage((float*)&state.battery.voltage);
    battery.percentage((float*)&state.battery.percentage);
  });
}

void imuTick() {
  SAFE_STATE_UPDATE(&stateMtx, {
    if (state.imu.ready) {
      imu.readTemperature((float*)&state.imu.temp);
      imu.readAccelerometer((float*)&state.imu.acc.x, (float*)&state.imu.acc.y,
                            (float*)&state.imu.acc.z);
      imu.readGyroscope((float*)&state.imu.gyro.x, (float*)&state.imu.gyro.y,
                        (float*)&state.imu.gyro.z);
    }
  });
}

void displayTick() {
  char counter_str[16];
  char bat_v_str[10];
  char bat_pct_str[10];
  char imu_temp_str[20];
  char imu_ax_str[18], imu_ay_str[18], imu_az_str[18];
  char imu_gx_str[21], imu_gy_str[21], imu_gz_str[21];

  SAFE_STATE_UPDATE(&stateMtx, {
    sprintf(counter_str, "count: %d", state.counter);
    sprintf(bat_v_str, "Bat: %.2f V", state.battery.voltage);
    auto bat_pct = state.battery.percentage * 100;
    sprintf(bat_pct_str, "%.0f %%", bat_pct);
    sprintf(imu_temp_str, "IMU temp: %.2f C", state.imu.temp);
    sprintf(imu_ax_str, "ax: %.2f g", state.imu.acc.x);
    sprintf(imu_ay_str, "ay: %.2f g", state.imu.acc.y);
    sprintf(imu_az_str, "az: %.2f g", state.imu.acc.z);
    sprintf(imu_gx_str, "gx: %.2f dps", state.imu.gyro.x);
    sprintf(imu_gy_str, "gy: %.2f dps", state.imu.gyro.y);
    sprintf(imu_gz_str, "gz: %.2f dps", state.imu.gyro.z);
  });

  // drawing
  display.startWrite();
  {
    display.drawRightString(counter_str, 228, 140);
    display.drawRightString(bat_v_str, 228, 70);
    display.drawRightString(bat_pct_str, 228, 80);
    display.drawString(imu_temp_str, 12, 70);
    display.drawString(imu_ax_str, 12, 90);
    display.drawString(imu_ay_str, 12, 100);
    display.drawString(imu_az_str, 12, 110);
    display.drawString(imu_gx_str, 12, 120);
    display.drawString(imu_gy_str, 12, 130);
    display.drawString(imu_gz_str, 12, 140);
  }
  display.endWrite();
}

void setup() {
  Serial.begin(115200);

  // set up state access mutex
  mutex_init(&stateMtx);

  // initialize battery adc
  { battery.begin(PIN_BAT_ADC); }

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
    // display
    add_repeating_timer_ms(
        -(1000 / CONFIG_DISPLAY_UPDATE_RATE_HZ),
        [](struct repeating_timer* t) {
          displayTick();
          return true;
        },
        NULL, &timerDisplay);

    // battery monitor
    add_repeating_timer_ms(
        -(1000 / CONFIG_DISPLAY_UPDATE_RATE_HZ),
        [](struct repeating_timer* t) {
          batteryTick();
          return true;
        },
        NULL, &timerBattery);
  }
}

void loop() {
  // run code on processor core 0
  delay(1000);
  SAFE_STATE_UPDATE(&stateMtx, { state.counter += 1; });
}

void loop1() {
  // run code on processor core 1
  delay(2000);
  SAFE_STATE_UPDATE(&stateMtx, { state.counter += 2; });
}
