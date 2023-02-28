#include <Arduino.h>
#include <I2Cdev.h>

#include "qmi8658c.hpp"

bool QMI8658C::detectDevice() {
  m_deviceId = 0;
  m_deviceRevision = 0;

  auto whoami_result =
      I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_WHO_AM_I, &m_deviceId,
                       I2Cdev::readTimeout, m_wire);

  if (whoami_result <= 0 || m_deviceId <= 0)
    return false;

  // read revision id as well
  I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_REVISION_ID, &m_deviceRevision,
                   I2Cdev::readTimeout, m_wire);

  return true;
}

bool QMI8658C::i2cReadU16(uint8_t register_l,
                          uint8_t register_h,
                          uint16_t* buffer) {
  uint8_t buf_l = 0, buf_h = 0;

  auto l = I2Cdev::readByte(m_i2cAddress, register_l, &buf_l,
                            I2Cdev::readTimeout, m_wire);
  auto h = I2Cdev::readByte(m_i2cAddress, register_h, &buf_h,
                            I2Cdev::readTimeout, m_wire);

  if (h && l) {
    *buffer = static_cast<uint16_t>(buf_h << 8) | buf_l;
    return true;
  }

  return false;
}

uint8_t QMI8658C::i2cReadRegisterBlock(uint8_t start_register,
                                       uint8_t length,
                                       uint8_t* buffer) {
  uint8_t read = 0;

  for (uint8_t i = 0; i < length; i++)
    if (I2Cdev::readByte(m_i2cAddress, start_register + i, &buffer[i],
                         I2Cdev::readTimeout, m_wire))
      read++;

  return read;
}

uint16_t QMI8658C::floatToFixed(float value,
                                uint8_t int_bits,
                                uint8_t fraction_bits) {
  int16_t integer_part = static_cast<int16_t>(value);
  int16_t fraction_part =
      static_cast<int16_t>((value - integer_part) * (1 << fraction_bits));
  return (integer_part << fraction_bits) + fraction_part;
}

bool QMI8658C::begin() {
  return begin(&Wire, QMI8658C_I2C_ADDRESS_DEFAULT);
}

bool QMI8658C::begin(TwoWire* wire) {
  m_wire = wire;
  return begin(QMI8658C_I2C_ADDRESS_DEFAULT);
}

bool QMI8658C::begin(uint8_t device_address) {
  m_i2cAddress = device_address;
  return begin(&Wire);
}

bool QMI8658C::begin(TwoWire* wire, uint8_t device_address) {
  m_wire = wire;
  m_i2cAddress = device_address;

  // wait for device to reply with device id
  auto success = false;
  uint8_t whoami_tries = 0;

  while (!success) {
    success = detectDevice();
    delay(10);

    // device not responding
    if (whoami_tries >= 3)
      return false;

    whoami_tries++;
  }

  // default settings
  // enable gyroscope and accelerometer
  enable(true, true);
  configureAcc();
  configureGyro();

  return true;
}

bool QMI8658C::deviceInfo(uint8_t* device_id, uint8_t* device_revision) {
  if (m_deviceId <= 0 || m_deviceRevision <= 0)
    return false;

  *device_id = m_deviceId;
  *device_revision = m_deviceRevision;

  return true;
}

void QMI8658C::enable(bool enable_gyro, bool enable_acc) {
  uint8_t value = 0;
  I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_CTRL7, &value,
                   I2Cdev::readTimeout, m_wire);
  value = ((value & 0b1111'1100) |
           ((enable_gyro ? 1 : 0) << 1 | (enable_acc ? 1 : 0)));

  I2Cdev::writeByte(m_i2cAddress, QMI8658C_REG_CTRL7, value, m_wire);
}

void QMI8658C::configureHsClock(bool enable) {
  uint8_t value = 0;
  I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_CTRL7, &value,
                   I2Cdev::readTimeout, m_wire);
  value = ((value & 0b1011'1111) | (enable ? 1 : 0) << 6);

  I2Cdev::writeByte(m_i2cAddress, QMI8658C_REG_CTRL7, value, m_wire);
}

void QMI8658C::configureAcc() {
  configureAcc(AccScale::ACC_SCALE_2G);
}

void QMI8658C::configureAcc(AccScale scale) {
  configureAcc(scale, AccODR::ACC_ODR_8000HZ);
}

void QMI8658C::configureAcc(AccScale scale, AccODR odr) {
  configureAcc(scale, odr, AccLPF::ACC_LPF_DISABLED);
}

void QMI8658C::configureAcc(AccScale scale, AccODR odr, AccLPF lpf) {
  // set the lsb sensitivity based on the scale
  switch (scale) {
    case AccScale::ACC_SCALE_2G:
      m_accelerometerLsbSensitivity = 16384.0f;
      break;
    case AccScale::ACC_SCALE_4G:
      m_accelerometerLsbSensitivity = 8192.0f;
      break;
    case AccScale::ACC_SCALE_8G:
      m_accelerometerLsbSensitivity = 4096.0f;
      break;
    case AccScale::ACC_SCALE_16G:
      m_accelerometerLsbSensitivity = 2048.0f;
      break;
  };

  // set scale and ODR
  I2Cdev::writeByte(
      m_i2cAddress, QMI8658C_REG_CTRL2,
      (static_cast<uint8_t>(scale) << 4) + static_cast<uint8_t>(odr), m_wire);

  // set lpf
  uint8_t lpf_reg = 0;
  uint8_t mask = 0b1111'1000;
  I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_CTRL5, &lpf_reg,
                   I2Cdev::readTimeout, m_wire);

  if (lpf == AccLPF::ACC_LPF_DISABLED)

    lpf_reg = (lpf_reg & mask) | (0b000);
  else
    lpf_reg = (lpf_reg & mask) | (static_cast<uint8_t>(lpf) << 1) | (0b1);

  I2Cdev::writeByte(m_i2cAddress, QMI8658C_REG_CTRL5, lpf_reg, m_wire);
}

void QMI8658C::configureGyro() {
  configureGyro(GyroScale::GYRO_SCALE_512DPS);
}

void QMI8658C::configureGyro(GyroScale scale) {
  configureGyro(scale, GyroODR::GYRO_ODR_8000HZ);
}

void QMI8658C::configureGyro(GyroScale scale, GyroODR odr) {
  configureGyro(scale, odr, GyroLPF::GYRO_LPF_DISABLED);
}

void QMI8658C::configureGyro(GyroScale scale, GyroODR odr, GyroLPF lpf) {
  // set the lsb sensitivity based on the scale
  switch (scale) {
    case GyroScale::GYRO_SCALE_16DPS:
      m_gyroscopeLsbSensitivity = 2048.0f;
      break;
    case GyroScale::GYRO_SCALE_32DPS:
      m_gyroscopeLsbSensitivity = 1024.0f;
      break;
    case GyroScale::GYRO_SCALE_64DPS:
      m_gyroscopeLsbSensitivity = 512.0f;
      break;
    case GyroScale::GYRO_SCALE_128DPS:
      m_gyroscopeLsbSensitivity = 256.0f;
      break;
    case GyroScale::GYRO_SCALE_256DPS:
      m_gyroscopeLsbSensitivity = 128.0f;
      break;
    case GyroScale::GYRO_SCALE_512DPS:
      m_gyroscopeLsbSensitivity = 64.0f;
      break;
    case GyroScale::GYRO_SCALE_1024DPS:
      m_gyroscopeLsbSensitivity = 32.0f;
      break;
    case GyroScale::GYRO_SCALE_2048DPS:
      m_gyroscopeLsbSensitivity = 16.0f;
      break;
  };

  // set scale and ODR
  I2Cdev::writeByte(
      m_i2cAddress, QMI8658C_REG_CTRL3,
      (static_cast<uint8_t>(scale) << 4) + static_cast<uint8_t>(odr), m_wire);

  // set lpf
  uint8_t lpf_reg = 0;
  uint8_t mask = 0b1000'1111;
  I2Cdev::readByte(m_i2cAddress, QMI8658C_REG_CTRL5, &lpf_reg,
                   I2Cdev::readTimeout, m_wire);

  if (lpf == GyroLPF::GYRO_LPF_DISABLED)

    lpf_reg = (lpf_reg & mask) | (0b000 << 4);
  else
    lpf_reg = (lpf_reg & mask) | (static_cast<uint8_t>(lpf) << 5) | (0b1 << 4);

  I2Cdev::writeByte(m_i2cAddress, QMI8658C_REG_CTRL5, lpf_reg, m_wire);
}

bool QMI8658C::readTemperature(float* degrees) {
  uint16_t temp = 0;

  if (!i2cReadU16(QMI8658C_REG_TEMP_L, QMI8658C_REG_TEMP_H, &temp))
    return false;

  // convert to float, and divide by the LSB/°C sensivity
  *degrees = static_cast<float>(static_cast<uint16_t>(temp)) / 256.0f;

  return true;
}

bool QMI8658C::readAccelerometer(float* ax, float* ay, float* az) {
  uint8_t result[QMI8658C_BUFSIZE_REG_ACC_XYZ] = {};

  if (i2cReadRegisterBlock(QMI8658C_REG_AX_L, QMI8658C_BUFSIZE_REG_ACC_XYZ,
                           result) != QMI8658C_BUFSIZE_REG_ACC_XYZ)
    return false;

  // combine h/l byte to uint16_t and divide by LSB sensitivy to get the actual
  // value
  *ax = static_cast<int16_t>(static_cast<uint16_t>(result[1] << 8) |
                             (result[0])) /
        m_accelerometerLsbSensitivity;
  *ay = static_cast<int16_t>(static_cast<uint16_t>(result[3] << 8) |
                             (result[2])) /
        m_accelerometerLsbSensitivity;
  *az = static_cast<int16_t>(static_cast<uint16_t>(result[5] << 8) |
                             (result[4])) /
        m_accelerometerLsbSensitivity;

  return true;
}

bool QMI8658C::readGyroscope(float* gx, float* gy, float* gz) {
  uint8_t result[QMI8658C_BUFSIZE_REG_GYRO_XYZ] = {};

  if (i2cReadRegisterBlock(QMI8658C_REG_GX_L, QMI8658C_BUFSIZE_REG_GYRO_XYZ,
                           result) != QMI8658C_BUFSIZE_REG_GYRO_XYZ)
    return false;

  // combine h/l byte to uint16_t and divide by LSB sensitivy to get the actual
  // value
  *gx = static_cast<int16_t>(static_cast<uint16_t>(result[1] << 8) |
                             (result[0])) /
        m_gyroscopeLsbSensitivity;
  *gy = static_cast<int16_t>(static_cast<uint16_t>(result[3] << 8) |
                             (result[2])) /
        m_gyroscopeLsbSensitivity;
  *gz = static_cast<int16_t>(static_cast<uint16_t>(result[5] << 8) |
                             (result[4])) /
        m_gyroscopeLsbSensitivity;

  return true;
}
