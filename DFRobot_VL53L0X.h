/*!
 * @file DFRobot_VL53L0X.h
 * @brief DFRobot's Laser rangefinder library. This example provides the VL53L0X laser rangefinder API function
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [LiXin](xin.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-8-21
 * @url https://github.com/DFRobot/DFRobot_VL53L0X
 */
 
#ifndef __DFRobot_VL53L0X_H
#define __DFRobot_VL53L0X_H

#include <Arduino.h>
#include <Wire.h>

#define VL53L0X_DEF_I2C_ADDR 0x29

class DFRobot_VL53L0X{
public:
  struct sVL53L0X_DetailedData_t{
    unsigned char i2cDevAddr;
    uint8_t mode;
    uint8_t precision;     /**< precision */
    unsigned char originalData[16];
    uint16_t ambientCount; /**< Environment quantity */
    uint16_t signalCount;  /**< A semaphore */
    uint16_t distance; 
    uint8_t status;
  };

  typedef enum {eDISABLE = 0, eENABLE = !eDISABLE} eFunctionalState;
  typedef enum {eHigh = 0, eLow = !eHigh} ePrecisionState;
  typedef enum {eSingle = 0, eContinuous = !eSingle} eModeState;
  /**
   * @fn DFRobot_VL53L0X
   * @brief DFRobot_VL53L0X abstract class constructor.
   */
  DFRobot_VL53L0X();
  ~DFRobot_VL53L0X();
  /**
   * @fn begin
   * @brief Init sensor and set I2C sub-device address.
   * @param addr 7 bits I2C address: 1~127
   * @return NONE
   */
  void begin(uint8_t addr = VL53L0X_DEF_I2C_ADDR,bool flag = false);	
  /**
   * @fn setMode
   * @brief Set operational mode to VL53L0X sensor.
   * @param mode Work mode settings
   * @n     eSingle      Single mode
   * @n     eContinuous  Back-to-back mode
   * @param precision Set measurement precision
   * @n     eHigh  High precision(0.25mm)
   * @n     eLow   Low precision(1mm)
   * @return NONE
   */
  void setMode(eModeState mode, ePrecisionState precision);
  /**
   * @fn start
   * @brief Start measuring distance.
   */
  void start();
  /**
   * @fn stop
   * @brief Stop measurement.
   */
  void stop();
  /**
   * @fn getDistance
   * @brief This function returns the distance measured by the sensor in mm.
   * @return The detailed distance
   */
  float getDistance();
  /**
   * @fn getAmbientCount
   * @brief Get ambient count.
   * @return Ambient count.
   */
  uint16_t getAmbientCount();
  /**
   * @fn getSignalCount
   * @brief Get signal count.
   * @return Signal count.
   */
  uint16_t getSignalCount();
  /**
   * @fn getStatus
   * @brief Get Status flag.
   * @return Status flag.
   */
  uint8_t getStatus();	
  struct sVL53L0X_DetailedData_t _detailedData;
private:
	uint16_t _distance;
	void writeByteData(unsigned char Reg, unsigned char byte);	
	uint8_t readByteData(unsigned char Reg);
	void setDeviceAddress(uint8_t newAddr);
	void writeData(unsigned char Reg ,unsigned char *buf, unsigned char Num);
	void readData(unsigned char Reg, unsigned char Num);
	void highPrecisionEnable(eFunctionalState NewState);
	void dataInit();
	void readVL53L0X();
};

#endif


