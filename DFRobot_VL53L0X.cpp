/*!
 * @file DFRobot_VL53L0X.cpp
 * @brief DFRobot's Laser rangefinder library. This example provides the VL53L0X laser rangefinder API function
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [LiXin](xin.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-8-21
 * @url https://github.com/DFRobot/DFRobot_VL53L0X
 */

#include "DFRobot_VL53L0X.h"

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID      		    0x00c0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      		0x00c2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   		0x0050
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 		0x0070
#define VL53L0X_REG_SYSRANGE_START                 		    0x0000
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS        		    0x0013
#define VL53L0X_REG_RESULT_RANGE_STATUS            		    0x0014
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS        		0x008a
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
struct sVL53L0X_DetailedData_t _detailedData;


DFRobot_VL53L0X::DFRobot_VL53L0X()
{}

DFRobot_VL53L0X::~DFRobot_VL53L0X()
{}


void DFRobot_VL53L0X::begin(uint8_t i2c_addr){
  uint8_t val1;
  delay(1500);
  _detailedData.i2cDevAddr = VL53L0X_DEF_I2C_ADDR; 
  dataInit(); 
  setDeviceAddress(i2c_addr);
  val1 = readByteData(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
  Serial.println("");
  Serial.print("Revision ID: "); Serial.println(val1,HEX);

  val1 = readByteData(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  Serial.print("Device ID: "); Serial.println(val1,HEX);	
  Serial.println("");
	
}

void DFRobot_VL53L0X::dataInit(){
	uint8_t data;
#ifdef ESD_2V8
	data = readByteData(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
	data = (data & 0xFE) | 0x01;
	writeByteData(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, data);
#endif
	writeByteData(0x88, 0x00);
	writeByteData(0x80, 0x01);
	writeByteData(0xFF, 0x01);
	writeByteData(0x00, 0x00);
	readByteData(0x91);
	writeByteData(0x91, 0x3c);
	writeByteData(0x00, 0x01);
	writeByteData(0xFF, 0x00);
	writeByteData(0x80, 0x00);
}


void DFRobot_VL53L0X::writeData(unsigned char Reg ,unsigned char *buf, 
	unsigned char Num){
   for(unsigned char i=0;i<Num;i++)
   {
	   writeByteData(Reg++, buf[i]);
   }
}

void DFRobot_VL53L0X::writeByteData(unsigned char Reg, unsigned char byte){
	Wire.beginTransmission(_detailedData.i2cDevAddr); // transmit to device #8
	Wire.write(Reg);              // sends one byte
	Wire.write((uint8_t)byte);
	Wire.endTransmission();     // stop transmitting
}

void DFRobot_VL53L0X::readData(unsigned char Reg, unsigned char Num){

	Wire.beginTransmission(_detailedData.i2cDevAddr); // transmit to device #8
	Wire.write((uint8_t)Reg);              // sends one byte
	Wire.endTransmission();    // stop transmitting
    Wire.requestFrom((uint8_t)_detailedData.i2cDevAddr, (uint8_t)Num);

	for(int i=0;i<Num;i++)
	{
		_detailedData.originalData[i] = Wire.read();
		delay(1);
	}
}


uint8_t DFRobot_VL53L0X::readByteData(unsigned char Reg){
	uint8_t data;
	Wire.beginTransmission(_detailedData.i2cDevAddr); // transmit to device #8
	Wire.write((uint8_t)Reg);              // sends one byte
	Wire.endTransmission();    // stop transmitting
    Wire.requestFrom((uint8_t)_detailedData.i2cDevAddr, (uint8_t)1);
	data = Wire.read();
	return data;
}

void DFRobot_VL53L0X::start(){
	uint8_t DeviceMode;
	uint8_t Byte;
	uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
	uint32_t LoopNb;
	
	DeviceMode = _detailedData.mode;
	
	writeByteData(0x80, 0x01);
	writeByteData(0xFF, 0x01);
	writeByteData(0x00, 0x00);
	writeByteData(0x91, 0x3c);
	writeByteData(0x00, 0x01);
	writeByteData(0xFF, 0x00);
	writeByteData(0x80, 0x00);
	
	switch(DeviceMode){
		case VL53L0X_DEVICEMODE_SINGLE_RANGING:
			writeByteData(VL53L0X_REG_SYSRANGE_START, 0x01);
			Byte = StartStopByte; 
			/* Wait until start bit has been cleared */
			LoopNb = 0;
			do {
				if (LoopNb > 0) Byte = readByteData(VL53L0X_REG_SYSRANGE_START);
				LoopNb = LoopNb + 1;
			} while (((Byte & StartStopByte) == StartStopByte) && 
						(LoopNb < VL53L0X_DEFAULT_MAX_LOOP));
			break;
		case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
			/* Back-to-back mode */
			/* Check if need to apply interrupt settings */
			//VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);中断检查?
			writeByteData(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
			break;
		default:
			/* Selected mode not supported */
			Serial.println("---Selected mode not supported---");	
	}	
}

void DFRobot_VL53L0X::readVL53L0X(){
	readData(VL53L0X_REG_RESULT_RANGE_STATUS, 12);
	_detailedData.ambientCount = ((_detailedData.originalData[6] & 0xFF) << 8) | 
									(_detailedData.originalData[7] & 0xFF);
	_detailedData.signalCount = ((_detailedData.originalData[8] & 0xFF) << 8) | 
									(_detailedData.originalData[9] & 0xFF);
	_detailedData.distance = ((_detailedData.originalData[10] & 0xFF) << 8) | 
								(_detailedData.originalData[11] & 0xFF);
	_detailedData.status = ((_detailedData.originalData[0] & 0x78) >> 3);
}

void DFRobot_VL53L0X::setDeviceAddress(uint8_t newAddr){
	newAddr &= 0x7F;
	writeByteData(VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, newAddr);
	_detailedData.i2cDevAddr = newAddr;
}

		
void DFRobot_VL53L0X::highPrecisionEnable(eFunctionalState NewState){
	writeByteData(VL53L0X_REG_SYSTEM_RANGE_CONFIG, 
		NewState);
}

void DFRobot_VL53L0X::setMode(eModeState mode, ePrecisionState precision){
	_detailedData.mode = mode;
	if(precision == eHigh){
		highPrecisionEnable(eENABLE);
		_detailedData.precision = precision;
	}
	else{
		highPrecisionEnable(eDISABLE);
		_detailedData.precision = precision;
	}
}



void DFRobot_VL53L0X::stop(){
	writeByteData(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);
	
	writeByteData(0xFF, 0x01);
	writeByteData(0x00, 0x00);
	writeByteData(0x91, 0x00);
	writeByteData(0x00, 0x01);
	writeByteData(0xFF, 0x00);
}

float DFRobot_VL53L0X::getDistance(){
	readVL53L0X();
	if(_detailedData.distance == 20)
		_detailedData.distance = _distance;
	else
		_distance = _detailedData.distance;
	if(_detailedData.precision == eHigh)
		return _detailedData.distance/4.0;
	else
		return _detailedData.distance;
}
		
uint16_t DFRobot_VL53L0X::getAmbientCount(){
	readVL53L0X();
	return _detailedData.ambientCount;
}
		
uint16_t DFRobot_VL53L0X::getSignalCount(){
	readVL53L0X();
	return _detailedData.signalCount;
}
		
uint8_t DFRobot_VL53L0X::getStatus(){
	readVL53L0X();
	return _detailedData.status;
}

