/*!
 * @file DFRobot_VL53L0X.cpp
 * @brief DFRobot's Laser rangefinder library
 * @n This example provides the VL53L0X laser rangefinder API function

 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 
 * @author [LiXin]
 * @version  V1.0
 * @date  2017-8-21
 * @https://github.com/DFRobot/DFRobot_VL53L0X
 */

#include "DFRobot_VL53L0X.h"
VL53L0X_DetailedData_t DetailedData;


DFRobotVL53L0X::DFRobotVL53L0X()
{}

DFRobotVL53L0X::~DFRobotVL53L0X()
{}


void DFRobotVL53L0X::begin(uint8_t i2c_addr=0x29){
  uint8_t val1;
  delay(1500);
  DetailedData.I2cDevAddr = I2C_DevAddr; 
  DataInit(); 
  setDeviceAddress(i2c_addr);
  val1 = readByteData(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
  Serial.println("");
  Serial.print("Revision ID: "); Serial.println(val1,HEX);

  val1 = readByteData(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  Serial.print("Device ID: "); Serial.println(val1,HEX);	
  Serial.println("");
	
}

void DFRobotVL53L0X::DataInit(){
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


void DFRobotVL53L0X::writeData(unsigned char Reg ,unsigned char *buf, 
	unsigned char Num){
   for(unsigned char i=0;i<Num;i++)
   {
	   writeByteData(Reg++, buf[i]);
   }
}

void DFRobotVL53L0X::writeByteData(unsigned char Reg, unsigned char byte){
	Wire.beginTransmission(DetailedData.I2cDevAddr); // transmit to device #8
	Wire.write(Reg);              // sends one byte
	Wire.write((uint8_t)byte);
	Wire.endTransmission();     // stop transmitting
}

void DFRobotVL53L0X::readData(unsigned char Reg, unsigned char Num){

	Wire.beginTransmission(DetailedData.I2cDevAddr); // transmit to device #8
	Wire.write((uint8_t)Reg);              // sends one byte
	Wire.endTransmission();    // stop transmitting
    Wire.requestFrom((uint8_t)DetailedData.I2cDevAddr, (uint8_t)Num);

	for(int i=0;i<Num;i++)
	{
		DetailedData.originalData[i] = Wire.read();
		delay(1);
	}
}


uint8_t DFRobotVL53L0X::readByteData(unsigned char Reg){
	uint8_t data;
	Wire.beginTransmission(DetailedData.I2cDevAddr); // transmit to device #8
	Wire.write((uint8_t)Reg);              // sends one byte
	Wire.endTransmission();    // stop transmitting
    Wire.requestFrom((uint8_t)DetailedData.I2cDevAddr, (uint8_t)1);
	data = Wire.read();
	return data;
}

void DFRobotVL53L0X::start(){
	uint8_t DeviceMode;
	uint8_t Byte;
	uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
	uint32_t LoopNb;
	
	DeviceMode = DetailedData.mode;
	
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

void DFRobotVL53L0X::readVL53L0X(){
	readData(VL53L0X_REG_RESULT_RANGE_STATUS, 12);
	DetailedData.ambientCount = ((DetailedData.originalData[6] & 0xFF) << 8) | 
									(DetailedData.originalData[7] & 0xFF);
	DetailedData.signalCount = ((DetailedData.originalData[8] & 0xFF) << 8) | 
									(DetailedData.originalData[9] & 0xFF);
	DetailedData.distance = ((DetailedData.originalData[10] & 0xFF) << 8) | 
								(DetailedData.originalData[11] & 0xFF);
	DetailedData.status = ((DetailedData.originalData[0] & 0x78) >> 3);
}

void DFRobotVL53L0X::setDeviceAddress(uint8_t newAddr){
	newAddr &= 0x7F;
	writeByteData(VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, newAddr);
	DetailedData.I2cDevAddr = newAddr;
}

		
void DFRobotVL53L0X::highPrecisionEnable(FunctionalState NewState){
	writeByteData(VL53L0X_REG_SYSTEM_RANGE_CONFIG, 
		NewState);
}

void DFRobotVL53L0X::setMode(ModeState mode, PrecisionState precision){
	DetailedData.mode = mode;
	if(precision == High){
		highPrecisionEnable(ENABLE);
		DetailedData.precision = precision;
	}
	else{
		highPrecisionEnable(DISABLE);
		DetailedData.precision = precision;
	}
}



void DFRobotVL53L0X::stop(){
	writeByteData(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);
	
	writeByteData(0xFF, 0x01);
	writeByteData(0x00, 0x00);
	writeByteData(0x91, 0x00);
	writeByteData(0x00, 0x01);
	writeByteData(0xFF, 0x00);
}

float DFRobotVL53L0X::getDistance(){
	readVL53L0X();
	if(DetailedData.distance == 20)
		DetailedData.distance = _distance;
	else
		_distance = DetailedData.distance;
	if(DetailedData.precision == High)
		return DetailedData.distance/4.0;
	else
		return DetailedData.distance;
}
		
uint16_t DFRobotVL53L0X::getAmbientCount(){
	readVL53L0X();
	return DetailedData.ambientCount;
}
		
uint16_t DFRobotVL53L0X::getSignalCount(){
	readVL53L0X();
	return DetailedData.signalCount;
}
		
uint8_t DFRobotVL53L0X::getStatus(){
	readVL53L0X();
	return DetailedData.status;
}

