/*!
 * @file DFRobot_QMC5883.cpp
 * @brief Compatible with QMC5883 and QMC5883
 * @n 3-Axis Digital Compass IC
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2017
 * @copyright	GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "DFRobot_QMC5883.h"

bool DFRobot_QMC5883::begin()
{
  bool ret = false;
  if (ICType == IC_NONE)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      Wire.begin();
      Wire.beginTransmission(HMC5883L_ADDRESS);
      if (Wire.endTransmission() == 0)
      {
        ICType = IC_HMC5883L;
        this->ICAddr = HMC5883L_ADDRESS;
        break;
      }
    }
  }
  if (ICType == IC_NONE)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      Wire.begin();
      Wire.beginTransmission(QMC5883_ADDRESS);
      if (Wire.endTransmission() == 0)
      {
        ICType = IC_QMC5883;
        this->ICAddr = QMC5883_ADDRESS;
        break;
      }
    }
  }
  if (ICType == IC_NONE)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      Wire.begin();
      Wire.beginTransmission(VCM5883L_ADDRESS);
      if (Wire.endTransmission() == 0)
      {
        ICType = IC_VCM5883L;
        this->ICAddr = VCM5883L_ADDRESS;
        break;
      }
    }
  }
  switch (ICType)
  {
  case IC_HMC5883L:
    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48) || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
    {
      return false;
    }
    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);
    ret = true;

    break;
  case IC_QMC5883:
    // Perform soft reset
    writeRegister8(QMC5883_REG_CONFIG_2, 0b10000000);
    // Define Set/Reset period
    writeRegister8(QMC5883_REG_IDENT_B, 0X01);

    //writeRegister8(QMC5883_REG_IDENT_C, 0X40);
    //writeRegister8(QMC5883_REG_IDENT_D, 0X01);
    // Write initail settings
    writeRegister8(QMC5883_REG_CONFIG_1, 0b00011101);
    if (fastRegister8(QMC5883_REG_CONFIG_1) != 0b00011101)
    {
      return false;
    }
    setRange(QMC5883_RANGE_8GA);
    setMeasurementMode(QMC5883_CONTINOUS);
    setDataRate(QMC5883_DATARATE_50HZ);
    setSamples(QMC5883_OVERSAMPLERATIO_512);
    ret = true;
    break;
  case IC_VCM5883L:
    writeRegister8(VCM5883L_CTR_REG1, 0X00);
    writeRegister8(VCM5883L_CTR_REG2, 0X4D);
    ret = true;
    break;
  case IC_NONE:
  default:
    ret = false;
    break;
  }
  return ret;
}

bool DFRobot_QMC5883::isDataReady(void)
{
  if (ICType == IC_QMC5883)
  {
    int8_t status = readRegister8(QMC5883_REG_STATUS);
    // Data Ready Register (DRDY), it is set when all three axis data is ready,
    // and loaded to the output data registers in the continuous measurement mode.
    // It is reset to “0” by reading any data register
    bool dataReady = (status & 0b00000001) == 0b1;
    // Overflow flag (OVL) is set to “1” if any data of three axis magnetic sensor channels is out of range
    bool dataOverflow = (status & 0b00000010) == 0b10;

    return dataReady & !dataOverflow;
  }
  return true;
}

int16_t DFRobot_QMC5883::readTempRaw(void)
{
  if (ICType == IC_QMC5883)
  {
    temp = readRegister16(QMC5883_REG_OUT_TEMP_1);
  }
  return temp;
}

Vector DFRobot_QMC5883::readRaw(void)
{
  if (ICType == IC_HMC5883L)
  {
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M);
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M);
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);
  }
  else if (ICType == IC_QMC5883)
  {
    v.XAxis = readRegister16(QMC5883_REG_OUT_X_L);
    v.YAxis = readRegister16(QMC5883_REG_OUT_Y_L);
    v.ZAxis = readRegister16(QMC5883_REG_OUT_Z_L);
  }
  else if (ICType == IC_VCM5883L)
  {
    v.XAxis = -readRegister16(VCM5883L_REG_OUT_X_L);
    v.YAxis = -readRegister16(VCM5883L_REG_OUT_Y_L);
    v.ZAxis = -readRegister16(VCM5883L_REG_OUT_Z_L);
  }
  //v.AngleXY = (atan2((double)v.YAxis, (double)v.XAxis) * (180 / PI) + 180);
  //v.AngleXZ = (atan2((double)v.ZAxis, (double)v.XAxis) * (180 / PI) + 180);
  //v.AngleYZ = (atan2((double)v.ZAxis, (double)v.YAxis) * (180 / PI) + 180);
  return v;
}

VectorScaled DFRobot_QMC5883::readScaled(void)
{
  if (isDataReady())
  {
    Vector raw = readRaw();
    s.XAxis = m_Scale * raw.XAxis;
    s.YAxis = m_Scale * raw.YAxis;
    s.ZAxis = m_Scale * raw.ZAxis;
    calibrate();
    s.XAxis -= offset.XAxis;
    s.YAxis -= offset.YAxis;
    s.ZAxis -= offset.ZAxis;
  }
  return s;
}

Calibration DFRobot_QMC5883::readCalibration(void)
{
  newCalibration = false;
  Calibration cal;
  cal.min = min;
  cal.max = max;
  cal.offset = offset;
  return cal;
}

void DFRobot_QMC5883::setCalibration(Calibration calibration)
{
  newCalibration = false;
  min = calibration.min;
  max = calibration.max;
  offset = calibration.offset;
}

void DFRobot_QMC5883::setDeclinationAngle(float declinationAngle)
{
  this->ICdeclinationAngle = declinationAngle;
}

float DFRobot_QMC5883::readHeadingDegrees()
{
  VectorScaled scaled = readScaled();

  double heading = atan2(scaled.YAxis, scaled.XAxis);

  heading += this->ICdeclinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0.0)
  {
    heading += (2.0 * PI);
  }
  // Check for wrap due to addition of declination.
  else if (heading > (2.0 * PI))
  {
    heading -= (2.0 * PI);
  }
  // Convert radians to degrees for readability.
  heading = heading * (180.0 / PI);

  float ret = (float)heading;
  return ret;
}

void DFRobot_QMC5883::calibrate()
{
  if (fabs(s.XAxis) > 600 || fabs(s.YAxis) > 600 || fabs(s.ZAxis) > 600)
  {
    return;
  }

  bool calibrationChanged = false;

  if (!minmaxset)
  {
    minmaxset = true;
    min.XAxis = s.XAxis;
    max.XAxis = s.XAxis;
    min.YAxis = s.YAxis;
    max.YAxis = s.YAxis;
    min.ZAxis = s.ZAxis;
    max.ZAxis = s.ZAxis;
    calibrationChanged = true;
  }
  else
  {
    if (s.XAxis < min.XAxis)
    {
      min.XAxis = s.XAxis;
      calibrationChanged = true;
    }
    else if (s.XAxis > max.XAxis)
    {
      max.XAxis = s.XAxis;
      calibrationChanged = true;
    }
    if (s.YAxis < min.YAxis)
    {
      min.YAxis = s.YAxis;
      calibrationChanged = true;
    }
    else if (s.YAxis > max.YAxis)
    {
      max.YAxis = s.YAxis;
      calibrationChanged = true;
    }
    if (s.ZAxis < min.ZAxis)
    {
      min.ZAxis = s.ZAxis;
      calibrationChanged = true;
    }
    else if (s.ZAxis > max.ZAxis)
    {
      max.ZAxis = s.ZAxis;
      calibrationChanged = true;
    }
  }

  if (calibrationChanged)
  {
    newCalibration = true;
    offset.XAxis = (max.XAxis + min.XAxis) / 2;
    offset.YAxis = (max.YAxis + min.YAxis) / 2;
    offset.ZAxis = (max.ZAxis + min.ZAxis) / 2;
  }
}

void DFRobot_QMC5883::setRange(QMC5883_range_t range)
{
  uint8_t value;
  if (ICType == IC_HMC5883L)
  {
    switch (range)
    {
    case HMC5883L_RANGE_0_88GA:
      Gauss_LSB_XY = 1370.0;
      break;

    case HMC5883L_RANGE_1_3GA:
      Gauss_LSB_XY = 1090.0;
      break;

    case HMC5883L_RANGE_1_9GA:
      Gauss_LSB_XY = 820.0;
      break;

    case HMC5883L_RANGE_2_5GA:
      Gauss_LSB_XY = 660.0;
      break;

    case HMC5883L_RANGE_4GA:
      Gauss_LSB_XY = 440.0;
      break;

    case HMC5883L_RANGE_4_7GA:
      Gauss_LSB_XY = 390.0;
      break;

    case HMC5883L_RANGE_5_6GA:
      Gauss_LSB_XY = 330.0;
      break;

    case HMC5883L_RANGE_8_1GA:
      Gauss_LSB_XY = 230.0;
      break;
    default:
      break;
    }

    value = readRegister8(HMC5883L_REG_CONFIG_B);
    value &= 0b00011111;
    value |= (range << 5);
    writeRegister8(HMC5883L_REG_CONFIG_B, value);
  }
  else if (ICType == IC_QMC5883)
  {
    switch (range)
    {
    case QMC5883_RANGE_2GA:
      // 12'000 LSB/G
      Gauss_LSB_XY = 12000;
      break;
    case QMC5883_RANGE_8GA:
      //  3'000 LSB/G
      Gauss_LSB_XY = 3000;
      break;
    default:
      break;
    }
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b11001111;
    value |= (range << 4);
    writeRegister8(QMC5883_REG_CONFIG_1, value);
  }
  else if (ICType == IC_VCM5883L)
  {
    //default 8G
  }
  m_Scale = 1000 / Gauss_LSB_XY;
}

QMC5883_range_t DFRobot_QMC5883::getRange(void)
{
  uint8_t value = 0;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_CONFIG_B);
    value &= 0b11100000;
    value >>= 5;
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00110000;
    value >>= 4;
    break;
  case IC_VCM5883L:
    value = QMC5883_RANGE_8GA;
    break;
  default:
    break;
  }
  return (QMC5883_range_t)value;
}

void DFRobot_QMC5883::setMeasurementMode(QMC5883_mode_t mode)
{
  uint8_t value;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;
    writeRegister8(HMC5883L_REG_MODE, value);
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b11111100;
    value |= mode;
    writeRegister8(QMC5883_REG_CONFIG_1, value);
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG2);
    value &= 0b11111110;
    value |= mode;
    writeRegister8(VCM5883L_CTR_REG2, value);
    break;
  default:
    break;
  }
}

QMC5883_mode_t DFRobot_QMC5883::getMeasurementMode(void)
{
  uint8_t value = 0;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00000011;
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG2);
    value &= 0b00000001;
    break;
  default:
    break;
  }
  return (QMC5883_mode_t)value;
}

void DFRobot_QMC5883::setDataRate(QMC5883_dataRate_t dataRate)
{
  uint8_t value;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);
    writeRegister8(HMC5883L_REG_CONFIG_A, value);
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b11110011;
    value |= (dataRate << 2);
    writeRegister8(QMC5883_REG_CONFIG_1, value);
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG2);
    value &= 0b11110011;
    value |= (dataRate << 2);
    writeRegister8(VCM5883L_CTR_REG2, value);
    break;
  default:
    break;
  }
}

QMC5883_dataRate_t DFRobot_QMC5883::getDataRate(void)
{
  uint8_t value = 0;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00001100;
    value >>= 2;
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG2);
    value &= 0b00001100;
    value >>= 2;
    break;
  default:
    break;
  }
  return (QMC5883_dataRate_t)value;
}

void DFRobot_QMC5883::setSamples(QMC5883_samples_t samples)
{
  uint8_t value;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);
    writeRegister8(HMC5883L_REG_CONFIG_A, value);
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00111111;
    value |= (samples << 6);
    writeRegister8(QMC5883_REG_CONFIG_1, value);
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG1);
    value &= 0b00111111;
    value |= (samples << 6);
    writeRegister8(VCM5883L_CTR_REG1, value);
    break;
  default:
    break;
  }
}

QMC5883_samples_t DFRobot_QMC5883::getSamples(void)
{
  uint8_t value = 0;
  switch (ICType)
  {
  case IC_HMC5883L:
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;
    break;
  case IC_QMC5883:
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00111111;
    value >>= 6;
    break;
  case IC_VCM5883L:
    value = readRegister8(VCM5883L_CTR_REG1);
    value &= 0b00111111;
    value >>= 6;
    break;
  default:
    break;
  }
  return (QMC5883_samples_t)value;
}

// Write byte to register
void DFRobot_QMC5883::writeRegister8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(this->ICAddr);
#if ARDUINO >= 100
  Wire.write(reg);
  Wire.write(value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}

// Read byte to register
uint8_t DFRobot_QMC5883::fastRegister8(uint8_t reg)
{
  uint8_t value = 0;
  Wire.beginTransmission(this->ICAddr);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)this->ICAddr, (uint8_t)1);
#if ARDUINO >= 100
  value = Wire.read();
#else
  value = Wire.receive();
#endif
  Wire.endTransmission();
  return value;
}

// Read byte from register
uint8_t DFRobot_QMC5883::readRegister8(uint8_t reg)
{
  uint8_t value = 0;
  Wire.beginTransmission(this->ICAddr);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)this->ICAddr, (uint8_t)1);
  while (!Wire.available())
  {
  };
#if ARDUINO >= 100
  value = Wire.read();
#else
  value = Wire.receive();
#endif
  Wire.endTransmission();
  return value;
}

// Read word from register
int16_t DFRobot_QMC5883::readRegister16(uint8_t reg)
{
  int16_t value = 0;
  uint8_t vha, vla;
  Wire.beginTransmission(this->ICAddr);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)this->ICAddr, (uint8_t)2);

  while (!Wire.available())
  {
  };
  if (ICType == IC_HMC5883L)
  {
#if ARDUINO >= 100
    vha = Wire.read();
    vla = Wire.read();
#else
    vha = Wire.receive();
    vla = Wire.receive();
#endif
  }
  else
  {
#if ARDUINO >= 100
    vla = Wire.read();
    vha = Wire.read();
#else
    vla = Wire.receive();
    vha = Wire.receive();
#endif
  }
  value = vha << 8 | vla;
  return value;
}

int DFRobot_QMC5883::getICType(void)
{
  return ICType;
}
