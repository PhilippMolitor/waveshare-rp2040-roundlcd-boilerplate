#pragma once

// QMI8658C IMU via I2C on I2C1 / Wire1
#define PIN_IMU_SDA 6    // IMU I2C data
#define PIN_IMU_SCL 7    // IMU I2C clock
#define PIN_IMU_INT_1 23 // IMU interrupt 1
#define PIN_IMU_INT_2 24 // IMU interrupt 2

// GC9A01 LCD via 4-Wire SPI
#define PIN_LCD_DC 8    // command/data selection
#define PIN_LCD_CS 9    // chip select
#define PIN_LCD_CLK 10  // clock
#define PIN_LCD_MOSI 11 // MOSI / LDC data in
#define PIN_LCD_RST 12  // reset
#define PIN_LCD_BL 25   // backlight enable

#define PIN_BAT_ADC 29 // battery adc
