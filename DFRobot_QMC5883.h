/*!
 * @file DFRobot_QMC5883.h
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

#ifndef DFROBOT_QMC5883_H
#define DFROBOT_QMC5883_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HMC5883L_ADDRESS             (0x1E)
#define QMC5883_ADDRESS              (0x0D)
#define VCM5883L_ADDRESS             (0x0C)


#define IC_NONE     0
#define IC_HMC5883L 1
#define IC_QMC5883  2
#define IC_VCM5883L 3
#define IC_ERROR    4


#define HMC5883L_REG_CONFIG_A        (0x00)
#define HMC5883L_REG_CONFIG_B        (0x01)
#define HMC5883L_REG_MODE            (0x02)
#define HMC5883L_REG_OUT_X_M         (0x03)
#define HMC5883L_REG_OUT_X_L         (0x04)
#define HMC5883L_REG_OUT_Z_M         (0x05)
#define HMC5883L_REG_OUT_Z_L         (0x06)
#define HMC5883L_REG_OUT_Y_M         (0x07)
#define HMC5883L_REG_OUT_Y_L         (0x08)
#define HMC5883L_REG_STATUS          (0x09)
#define HMC5883L_REG_IDENT_A         (0x0A)
#define HMC5883L_REG_IDENT_B         (0x0B)
#define HMC5883L_REG_IDENT_C         (0x0C)


#define QMC5883_REG_OUT_X_L          (0x00)
#define QMC5883_REG_OUT_X_M          (0x01)
#define QMC5883_REG_OUT_Y_L          (0x02)
#define QMC5883_REG_OUT_Y_M          (0x03)
#define QMC5883_REG_OUT_Z_L          (0x04)
#define QMC5883_REG_OUT_Z_M          (0x05)
#define QMC5883_REG_STATUS           (0x06)
#define QMC5883_REG_OUT_TEMP_1       (0x07)
#define QMC5883_REG_OUT_TEMP_2       (0x08)
#define QMC5883_REG_CONFIG_1         (0x09)
#define QMC5883_REG_CONFIG_2         (0x0A)
#define QMC5883_REG_IDENT_B          (0x0B)
#define QMC5883_REG_IDENT_C          (0x20)
#define QMC5883_REG_IDENT_D          (0x21)


#define VCM5883L_REG_OUT_X_L         (0x00)
#define VCM5883L_REG_OUT_X_H         (0x01)
#define VCM5883L_REG_OUT_Y_L         (0x02)
#define VCM5883L_REG_OUT_Y_H         (0x03)
#define VCM5883L_REG_OUT_Z_L         (0x04)
#define VCM5883L_REG_OUT_Z_H         (0x05)
#define VCM5883L_CTR_REG1            (0x0B)
#define VCM5883L_CTR_REG2            (0x0A)


typedef enum
{
  HMC5883L_SAMPLES_8          = 0b11,
  HMC5883L_SAMPLES_4          = 0b10,
  HMC5883L_SAMPLES_2          = 0b01,
  HMC5883L_SAMPLES_1          = 0b00,

  QMC5883_OVERSAMPLERATIO_64  = 0b11,
  QMC5883_OVERSAMPLERATIO_128 = 0b10,
  QMC5883_OVERSAMPLERATIO_256 = 0b01,
  QMC5883_OVERSAMPLERATIO_512 = 0b00, // default, most accurate, higest power consumption
} QMC5883_samples_t;

typedef enum
{
  HMC5883L_DATARATE_0_75_HZ = 0b000,
  HMC5883L_DATARATE_1_5HZ   = 0b001,
  HMC5883L_DATARATE_3HZ     = 0b010,
  HMC5883L_DATARATE_7_5HZ   = 0b011,
  HMC5883L_DATARATE_15HZ    = 0b100,
  HMC5883L_DATARATE_30HZ    = 0b101,
  HMC5883L_DATARATE_75HZ    = 0b110,

  QMC5883_DATARATE_10HZ     = 0b000,
  QMC5883_DATARATE_50HZ     = 0b001,
  QMC5883_DATARATE_100HZ    = 0b010,
  QMC5883_DATARATE_200HZ    = 0b011,

  VCM5883L_DATARATE_200HZ   = 0b000,
  VCM5883L_DATARATE_100HZ   = 0b001,
  VCM5883L_DATARATE_50HZ    = 0b010,
  VCM5883L_DATARATE_10HZ    = 0b011,
} QMC5883_dataRate_t;

typedef enum
{
  HMC5883L_RANGE_0_88GA = 0b000,
  HMC5883L_RANGE_1_3GA  = 0b001,
  HMC5883L_RANGE_1_9GA  = 0b010,
  HMC5883L_RANGE_2_5GA  = 0b011,
  HMC5883L_RANGE_4GA    = 0b100,
  HMC5883L_RANGE_4_7GA  = 0b101,
  HMC5883L_RANGE_5_6GA  = 0b110,
  HMC5883L_RANGE_8_1GA  = 0b111,

  QMC5883_RANGE_2GA     = 0b00,
  QMC5883_RANGE_8GA     = 0b01,

  VCM5883L_RANGE_8GA    = 0b01,
} QMC5883_range_t;

typedef enum
{
  HMC5883L_IDLE      = 0b10,
  HMC5883_SINGLE     = 0b01,
  HMC5883L_CONTINOUS = 0b00,

  QMC5883_STANDBY    = 0b00,
  QMC5883_CONTINOUS  = 0b01,

  VCM5883L_SINGLE    = 0b00,
  VCM5883L_CONTINOUS = 0b01,
} QMC5883_mode_t;


#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
  int16_t XAxis;
  int16_t YAxis;
  int16_t ZAxis;
  //float   AngleXY;
  //float   AngleXZ;
  //float   AngleYZ;
};
#endif

struct VectorScaled
{
  float XAxis;
  float YAxis;
  float ZAxis;
};

struct Calibration {
  VectorScaled min;
  VectorScaled max;
  VectorScaled offset;
};

class DFRobot_QMC5883
{
public:
  bool begin(void);

  bool isDataReady(void);

  int16_t readTempRaw(void);

  Vector readRaw(void);
  VectorScaled readScaled(void);

  Calibration readCalibration(void);
  void setCalibration(Calibration calibration);
  bool hasNewCalibration()
  {
    return newCalibration;
  }

  void setDeclinationAngle(float declinationAngle);
  float readHeadingDegrees();
  
  void setRange(QMC5883_range_t range);
  QMC5883_range_t getRange(void);

  void setMeasurementMode(QMC5883_mode_t mode);
  QMC5883_mode_t getMeasurementMode(void);

  void setDataRate(QMC5883_dataRate_t dataRate);
  QMC5883_dataRate_t getDataRate(void);

  void setSamples(QMC5883_samples_t samples);
  QMC5883_samples_t getSamples(void);

  double ICdeclinationAngle;
  int ICType = IC_NONE;
  uint8_t ICAddr = HMC5883L_ADDRESS;
  int getICType(void);
  bool isHMC()
  {
    return ICType == IC_HMC5883L;
  }
  bool isQMC()
  {
    return ICType == IC_QMC5883;
  }
  bool isVCM()
  {
    return ICType == IC_VCM5883L;
  }

private:
  double m_Scale = 1;
  float Gauss_LSB_XY = 1090.0;
  Vector v;
  VectorScaled s;

  int16_t temp;

  bool newCalibration = false;
  bool minmaxset = false;
  VectorScaled min;
  VectorScaled max;
  VectorScaled offset;
  
  void calibrate(void);

  void writeRegister8(uint8_t reg, uint8_t value);
  uint8_t readRegister8(uint8_t reg);
  uint8_t fastRegister8(uint8_t reg);
  int16_t readRegister16(uint8_t reg);
};

#endif