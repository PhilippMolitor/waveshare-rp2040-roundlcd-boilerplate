/*
 * @Description: QMI8658C
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-27 10:47:31
 * @LastEditors: Changhua
 */
#include <Wire.h>
#include <I2Cdev.hpp>

#include "qmi8658c.hpp"

/*----------------------------------------------------------------------------------------------
  QMI8658C UI Sensor Configuration Settings and Output Data
*/
///< Configuration Registers>
#define address 0X6B  // device address
#define WHO_AM_I 0X00 // Device identifier
#define CTRL1 0x02    // Serial Interface and Sensor Enable
#define CTRL2 0x03    // Accelerometer Settings
#define CTRL3 0x04    // Gyroscope Settings
#define CTRL4 0X05    // Magnetometer Settings
#define CTRL5 0X06    // Sensor Data Processing Settings
#define CTRL7 0x08    // Enable Sensors and Configure Data Reads
#define CTRL8 0X09    // Reserved – Special Settings

///< Sensor Data Output Registers>
#define AccX_L 0x35
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A

#define GyrX_L 0x3B
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40

/*
  QMI8658C UI Sensor Configuration Settings and Output Data
----------------------------------------------------------------------------------------------*/
int16_t QMI8658C_readBytes(void)
{
  uint8_t buffer[14];
  I2Cdev::readBytes(address, GyrZ_L, 2, buffer);
  return ((buffer[1] << 8) | buffer[0]);
}

/*
  获取陀螺仪静态原始数据
  作用于偏航角计算时调零参量
*/
bool QMI8658C::QMI8658C_calibration(void)
{
  unsigned short times = 1000; // 采样次数
  for (int i = 0; i < times; i++)
  {
    gz = QMI8658C_readBytes();
    gzo += gz;
  }
  gzo /= times; // 计算陀螺仪偏移

  return false;
}

bool QMI8658C::QMI8658C_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout;
  do
  {
    I2Cdev::readBytes(address, WHO_AM_I, 1, &chip_id);
    Serial.print("WHO_AM_I: 0X");
    Serial.println(chip_id, HEX);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
    delay(10);
  } while (chip_id == 0); // 确保从机设备在线（强行等待 获取 ID ）

  I2Cdev::writeByte(address, CTRL1, 0x40); // Serial Interface and Sensor Enable<串行接口（SPI或I 2 C）地址自动递增>
  I2Cdev::writeByte(address, CTRL7, 0x03); // Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>

  I2Cdev::writeByte(address, CTRL2, 0x04); // Accelerometer Settings<±2g  500Hz>
  I2Cdev::writeByte(address, CTRL3, 0x64); // Gyroscope Settings< ±2048dps 500Hz>
  I2Cdev::writeByte(address, CTRL5, 0x11); // Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>

  delay(1000);

  // unsigned short times = 1000; //采样次数
  // for (int i = 0; i < times; i++)
  // {
  //   gz = QMI8658C_readBytes();
  //   gzo += gz;
  // }
  // gzo /= times; //计算陀螺仪偏移

  QMI8658C_calibration();
  return false;
}

bool QMI8658C::QMI8658C_dveGetEulerAngles(float *gyro, float *yaw)
{
  unsigned long now = millis();   // 当前时间(ms)
  dt = (now - lastTime) / 1000.0; // 微分时间(s)
  lastTime = now;                 // 上一次采样时间(ms)
  gz = QMI8658C_readBytes();      // 读取六轴原始数值
  float gyroz = -(gz - gzo) / 32.00 * dt;
  if (fabs(gyroz) <= 0.05)
  {
    gyroz = 0.00;
  }
  agz += gyroz; // z轴角速度积分

  *gyro = gyroz;

  *yaw = agz;
  return false;
}

bool QMI8658C::QMI8658C_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();   // 当前时间(ms)
  dt = (now - lastTime) / 1000.0; // 微分时间(s)
  lastTime = now;                 // 上一次采样时间(ms)
  gz = QMI8658C_readBytes();
  float gyroz = -(gz - gzo) / 32.00 * dt; // z轴角速度< 131.0 为传感器比例系数常量，详细信息请查阅MPU-6050_DataSheet>
  if (fabs(gyroz) <= 0.05)
  {
    gyroz = 0.00;
  }
  agz += gyroz; // z轴角速度积分
  *Yaw = agz;
  return false;
}

void HDSC_IIC_Test(void)
{
  float gyro, is_yaw;
  uint8_t IIC_buff[10];
  uint8_t a = 0;
  Wire.requestFrom(0x51, 6); // request 6 bytes from slave device #2
  while (Wire.available())   // slave may send less than requested
  {
    IIC_buff[a++] = Wire.read(); // receive a byte as character
  }

  if ((IIC_buff[0] == 0XA1) && (IIC_buff[5] == 0XB1))
  {

    gyro = ((IIC_buff[1] << 8) | (IIC_buff[2])) / 1000.00;
    is_yaw = ((IIC_buff[3] << 8) | (IIC_buff[4])) / 100.00;

    Serial.print(gyro);
    Serial.print("\t");
    Serial.print(is_yaw);
  }
  else
  {
    /* code */
    Serial.println("Contact Changhua :STM8S003F3_MPU6050 data error"); // print the character
    return;
  }
  // for (int i = 0; i < a; i++)
  // {
  //   Serial.print(IIC_buff[i], HEX);
  //   Serial.print("\t");
  // }
  Serial.println("");
}

void QMI8658C::QMI8658C_Check(void)
{
  Wire.begin();
  uint8_t address_Test = 0;
  int error = 1;
  delay(1000);
  do
  {
    Wire.beginTransmission(address_Test);
    error = Wire.endTransmission();

    Serial.print("address_Test: 0X");
    Serial.println(address_Test, HEX);
    address_Test++;
    delay(100);
  } while (error); // 扫描从机设备
  Serial.println("address_Test: OK");

  // Wire.beginTransmission(0x51);
  // Wire.write(110);
  // Wire.endTransmission();

  // for (;;)
  // {

  //   HDSC_IIC_Test();
  //   //delay(1);
  // }
}
