#pragma once

#include <Arduino.h>
#include <Wire.h>

// constants

#define QMI8658C_I2C_ADDRESS_DEFAULT \
  (0x6a)  // QMI8658C default I2C device address
#define QMI8658C_I2C_ADDRESS_PULLUP \
  (0x6b)  // QMI8658C I2C device address when SA0 is pulled up

#define QMI8658C_TEMP_LSB_SENSITIVITY (256.0f)

// settings registers (RW)

#define QMI8658C_REG_WHO_AM_I 0x00     // Device identifier
#define QMI8658C_REG_REVISION_ID 0x01  // Device Revision ID
#define QMI8658C_REG_CTRL1 0x02        // Serial Interface and Sensor Enable
#define QMI8658C_REG_CTRL2 \
  0x03  // Accelerometer: Output Data Rate, Full Scale, Self Test
#define QMI8658C_REG_CTRL3 \
  0x04  // Gyroscope: Output Data Rate, Full Scale, Self Test
#define QMI8658C_REG_CTRL4 \
  0x05  // Magnetometer Settings: Output Data Rate, and Device Selection
#define QMI8658C_REG_CTRL5 0x06  // Low pass filter setting.
#define QMI8658C_REG_CTRL6 \
  0x07  // AttitudeEngine™ Settings: Output Data Rate, Motion on Demand
#define QMI8658C_REG_CTRL7 0x08  // Enable Sensors
#define QMI8658C_REG_CTRL8 0x09  // Reserved: Not Used
#define QMI8658C_REG_CTRL9 0x0a  // Host Commands

// status registers (R)

#define QMI8658C_REG_STATUS0 0x46
#define QMI8658C_REG_STATUS1 0x47

// calibration registers (RW)

#define QMI8658C_REG_CAL1_L 0x0b
#define QMI8658C_REG_CAL1_H 0x0c
#define QMI8658C_REG_CAL2_L 0x0d
#define QMI8658C_REG_CAL2_H 0x0e
#define QMI8658C_REG_CAL3_L 0x0f
#define QMI8658C_REG_CAL3_H 0x10
#define QMI8658C_REG_CAL4_L 0x11
#define QMI8658C_REG_CAL4_H 0x12

// temperature sensor data registers (R)

#define QMI8658C_REG_TEMP_L 0x33
#define QMI8658C_REG_TEMP_H 0x34
#define QMI8658C_BUFSIZE_REG_TEMP \
  (QMI8658C_REG_TEMP_H - QMI8658C_REG_TEMP_L + 1)

// accelerometer data registers (R)

#define QMI8658C_REG_AX_L 0x35
#define QMI8658C_REG_AX_H 0x36
#define QMI8658C_REG_AY_L 0x37
#define QMI8658C_REG_AY_H 0x38
#define QMI8658C_REG_AZ_L 0x39
#define QMI8658C_REG_AZ_H 0x3a
#define QMI8658C_BUFSIZE_REG_ACC_XYZ (QMI8658C_REG_AZ_H - QMI8658C_REG_AX_L + 1)

// gyroscope data registers (R)

#define QMI8658C_REG_GX_L 0x3b
#define QMI8658C_REG_GX_H 0x3c
#define QMI8658C_REG_GY_L 0x3d
#define QMI8658C_REG_GY_H 0x3e
#define QMI8658C_REG_GZ_L 0x3f
#define QMI8658C_REG_GZ_H 0x40
#define QMI8658C_BUFSIZE_REG_GYRO_XYZ \
  (QMI8658C_REG_GZ_H - QMI8658C_REG_GX_L + 1)

/**
 * QMI8658C 6-Axis IMU I2C device driver.
 *
 * Open ToDos:
 * - calibration. documentation page 37-39 describe the process.
 * - figure out how the accelerometer low power modes work.
 *   it seems that when waiting for the INT2 interrupt, it only happens once in
 *   this mode. also, there were some weird quirks resulting in the temp sensor
 *   reading about 80 degrees.
 *   also, maybe just don't allow them.
 * - have a look into the AttitudeEngine stuff and maybe make it work
 *
 * For reference, see the following URLs:
 *
 * - Basic (and very use-case specific) implementation that inspired this
 *   library:
 *   https://github.com/SamibyDubuisson/Smart-Robot-Car-V4.0-UPDATED/blob/main/QMI8658C.cpp
 * - Datasheet:
 *   https://datasheet.lcsc.com/lcsc/2108161930_QST-QMI8658C_C2842151.pdf
 *
 * @author Philipp Molitor <philipp@molitor-consulting.com>
 */
class QMI8658C {
 private:
  TwoWire* m_wire;
  uint8_t m_i2cAddress;

  uint8_t m_deviceId;
  uint8_t m_deviceRevision;
  float m_accelerometerLsbSensitivity;
  float m_gyroscopeLsbSensitivity;

  bool detectDevice();
  bool i2cReadU16(uint8_t register_l, uint8_t register_h, uint16_t* buffer);
  uint8_t i2cReadRegisterBlock(uint8_t start_register,
                               uint8_t length,
                               uint8_t* buffer);
  uint16_t floatToFixed(float value, uint8_t int_bits, uint8_t fraction_bits);

 public:
  /// Gyroscope scale in dps (degrees per second)
  enum class AccScale : uint8_t {
    /// Accelerometer Full-scale = ±2 g
    ACC_SCALE_2G = 0b000,
    /// Accelerometer Full-scale = ±4 g
    ACC_SCALE_4G = 0b001,
    /// Accelerometer Full-scale = ±8 g
    ACC_SCALE_8G = 0b010,
    /// Accelerometer Full-scale = ±16 g
    ACC_SCALE_16G = 0b011,
  };

  /// Accelerometer output data rate in Hz
  enum class AccODR : uint8_t {
    /// 8000Hz, normal mode, 100% duty cycle
    ACC_ODR_8000HZ = 0b0000,
    /// 4000Hz, normal mode, 100% duty cycle
    ACC_ODR_4000HZ = 0b0001,
    /// 2000Hz, normal mode, 100% duty cycle
    ACC_ODR_2000HZ = 0b0010,
    /// 1000Hz, normal mode, 100% duty cycle
    ACC_ODR_1000HZ = 0b0011,
    /// 500Hz, normal mode, 100% duty cycle
    ACC_ODR_500HZ = 0b0100,
    /// 250Hz, normal mode, 100% duty cycle
    ACC_ODR_250HZ = 0b0101,
    /// 125Hz, normal mode, 100% duty cycle
    ACC_ODR_125HZ = 0b0110,
    /// 62.5Hz, normal mode, 100% duty cycle
    ACC_ODR_62_5HZ = 0b0111,
    /// 31.25Hz, normal mode, 100% duty cycle
    ACC_ODR_31_25HZ = 0b1000,

    /*
     * low power modes
     * disabled for now, as they behave weirdly in a normal IMU setup
     * with gyro+acc enabled.
     */
    /*
    /// 128Hz, low-power mode, 100% duty cycle
    ACC_ODR_LP_128HZ = 0b1100,
    /// 21Hz, low-power mode, 58% duty cycle
    ACC_ODR_LP_21HZ = 0b1101,
    /// 11Hz, low-power mode, 31% duty cycle
    ACC_ODR_LP_11HZ = 0b1110,
    /// 3Hz, low-power mode, 8.5% duty cycle
    ACC_ODR_LP_3HZ = 0b1111,
    */
  };

  /// Accelerometer low pass filter
  enum class AccLPF : uint8_t {
    /// Bandwidth: 2.62% of ODR
    ACC_LPF_2_62PCT = 0b00,
    /// Bandwidth: 2.62% of ODR
    ACC_LPF_3_59PCT = 0b01,
    /// Bandwidth: 2.62% of ODR
    ACC_LPF_5_32PCT = 0b10,
    /// Bandwidth: 2.62% of ODR
    ACC_LPF_14PCT = 0b11,
    /// Disables the accelerometer low-pass filter
    ACC_LPF_DISABLED = 0b1111'1111,
  };

  /// Accelerometer scale in g
  enum class GyroScale : uint8_t {
    /// Gyroscope Full-scale = ±16 dps
    GYRO_SCALE_16DPS = 0b000,
    /// Gyroscope Full-scale = ±32 dps
    GYRO_SCALE_32DPS = 0b001,
    /// Gyroscope Full-scale = ±64 dps
    GYRO_SCALE_64DPS = 0b010,
    /// Gyroscope Full-scale = ±128 dps
    GYRO_SCALE_128DPS = 0b011,
    /// Gyroscope Full-scale = ±256 dps
    GYRO_SCALE_256DPS = 0b100,
    /// Gyroscope Full-scale = ±512 dps
    GYRO_SCALE_512DPS = 0b101,
    /// Gyroscope Full-scale = ±1024 dps
    GYRO_SCALE_1024DPS = 0b110,
    /// Gyroscope Full-scale = ±2048 dps
    GYRO_SCALE_2048DPS = 0b111,
  };

  /// Gyroscope output data rate in Hz
  enum class GyroODR : uint8_t {
    /// 8000Hz, normal mode, 100% duty cycle
    GYRO_ODR_8000HZ = 0b0000,
    /// 4000Hz, normal mode, 100% duty cycle
    GYRO_ODR_4000HZ = 0b0001,
    /// 2000Hz, normal mode, 100% duty cycle
    GYRO_ODR_2000HZ = 0b0010,
    /// 1000Hz, normal mode, 100% duty cycle
    GYRO_ODR_1000HZ = 0b0011,
    /// 500Hz, normal mode, 100% duty cycle
    GYRO_ODR_500HZ = 0b0100,
    /// 250Hz, normal mode, 100% duty cycle
    GYRO_ODR_250HZ = 0b0101,
    /// 125Hz, normal mode, 100% duty cycle
    GYRO_ODR_125HZ = 0b0110,
    /// 62.5Hz, normal mode, 100% duty cycle
    GYRO_ODR_62_5HZ = 0b0111,
    /// 31.25Hz, normal mode, 100% duty cycle
    GYRO_ODR_31_25HZ = 0b1000,
  };

  /// Gyroscope low pass filter
  enum class GyroLPF : uint8_t {
    /// Bandwidth: 2.62% of ODR
    GYRO_LPF_2_62PCT = 0b00,
    /// Bandwidth: 2.62% of ODR
    GYRO_LPF_3_59PCT = 0b01,
    /// Bandwidth: 2.62% of ODR
    GYRO_LPF_5_32PCT = 0b10,
    /// Bandwidth: 2.62% of ODR
    GYRO_LPF_14PCT = 0b11,
    /// Disables the gyroscope low-pass filter
    GYRO_LPF_DISABLED = 0b1111'1111,
  };

  bool begin();
  bool begin(TwoWire* wire);
  bool begin(uint8_t device_address);
  bool begin(TwoWire* wire, uint8_t device_address);
  bool deviceInfo(uint8_t* device_id, uint8_t* device_revision);

  // void calibrate();
  // void calibrate(int samples);
  void enable(bool enable_gyro, bool enable_acc);
  void configureHsClock(bool enable);
  void configureAcc();
  void configureAcc(AccScale scale);
  void configureAcc(AccScale scale, AccODR odr);
  void configureAcc(AccScale scale, AccODR odr, AccLPF lpf);
  void configureGyro();
  void configureGyro(GyroScale scale);
  void configureGyro(GyroScale scale, GyroODR odr);
  void configureGyro(GyroScale scale, GyroODR odr, GyroLPF lpf);

  /// @brief reads out the internal temperature sensor
  /// @param degrees
  /// @return `true` if the read operation was successful
  bool readTemperature(float* degrees);
  /// @brief reads the current accelerometer values
  /// @param ax x-axis value in g
  /// @param ay y-axis value in g
  /// @param az z-axis value in g
  /// @return `true` on success, `false` if the read operation failed
  bool readAccelerometer(float* ax, float* ay, float* az);
  /// @brief reads out the current gyroscope values
  /// @param gx x-axis rotation in dps (degrees per second)
  /// @param gy y-axis rotation in dps (degrees per second)
  /// @param gz z-axis rotation in dps (degrees per second)
  /// @return `true` on success, `false` if the read operation failed
  bool readGyroscope(float* gx, float* gy, float* gz);
};
