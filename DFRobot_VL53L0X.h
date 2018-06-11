/*!
 * @file DFRobot_VL53L0X.h
 * @brief DFRobot's Laser rangefinder library
 * @n This example provides the VL53L0X laser rangefinder API function

 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 
 * @author [LiXin]
 * @version  V1.0
 * @date  2017-8-21
 * @https://github.com/DFRobot/DFRobot_VL53L0X
 */
 
#ifndef __DFRobot_VL53L0X_H
#define __DFRobot_VL53L0X_H

#include <Arduino.h>
#include <Wire.h>

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID      		    0x00c0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      		0x00c2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   		0x0050
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 		0x0070
#define VL53L0X_REG_SYSRANGE_START                 		    0x0000
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS        		    0x0013
#define VL53L0X_REG_RESULT_RANGE_STATUS            		    0x0014
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS        		0x008a
#define VL53L0X_I2C_ADDR									0x0029
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG			            0x0009
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x0089
#define VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT                0x0000
#define VL53L0X_REG_SYSRANGE_MODE_START_STOP                0x0001
#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK                0x0002
#define VL53L0X_REG_SYSRANGE_MODE_TIMED                     0x0004

#define VL53L0X_DEVICEMODE_SINGLE_RANGING	               ((uint8_t)  0)
#define VL53L0X_DEVICEMODE_CONTINUOUS_RANGING	           ((uint8_t)  1)
#define VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING        ((uint8_t)  3)
#define VL53L0X_DEFAULT_MAX_LOOP  200

#define ESD_2V8
#define I2C_DevAddr 0x29

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {High = 0, Low = !High} PrecisionState;
typedef enum {Single = 0, Continuous = !Single} ModeState;
typedef struct {
	unsigned char I2cDevAddr;
	uint8_t mode;
	uint8_t precision; //precision
	unsigned char originalData[16];
	uint16_t ambientCount;//Environment quantity
	uint16_t signalCount;//A semaphore
	uint16_t distance; 
	uint8_t status;
}VL53L0X_DetailedData_t;
extern VL53L0X_DetailedData_t DetailedData;


class DFRobotVL53L0X
{
	public:
		DFRobotVL53L0X();
		~DFRobotVL53L0X();
		void begin(uint8_t i2c_addr);	
		void setMode(ModeState mode, PrecisionState precision);
		void start();
		void stop();
		float getDistance();
		uint16_t getAmbientCount();
		uint16_t getSignalCount();
		uint8_t getStatus();	
	private:
		uint16_t _distance;
		void writeByteData(unsigned char Reg, unsigned char byte);	
		uint8_t readByteData(unsigned char Reg);
		void writeData(unsigned char Reg ,unsigned char *buf, unsigned char Num);
		void readData(unsigned char Reg, unsigned char Num);
		void setDeviceAddress(uint8_t newAddr);
		void highPrecisionEnable(FunctionalState NewState);
		void DataInit();
		void readVL53L0X();
};

#endif


