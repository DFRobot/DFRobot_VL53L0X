# VL53L0 Distance Ranging Sensor
World smallest Time-of-Flight ranging and gesture detection sensor
![SVG1](https://raw.githubusercontent.com/DFRobot/binaryfiles/master/SEN0245/SEN0245svg1.png)
---------------------------------------------------------

# DFRobot_VL53L0X Library for Arduino

This library provides the VL53L0X laser rangefinder API function

### Ready to start 
 
	void begin(uint8_t i2c_addr);
	i2c_addr:Set I2C sub-device address

### Set operational mode to VL53L0X
   
	void setMode(uint8_t mode, uint8_t precision);
	mode：Work mode settings
	Single：Single mode  
	Continuous：Back-to-back mode
	precision：Set measurement precision
	High：High precision(0.25mm)
	Low: Low precision(1mm)
	
### Start measuring distance
   
	void start();

### Stop measurement
	
	void stop();
	
### get distance data
	
	uint16_t getDistance();
		
### get ambient count
		
	uint16_t getAmbientCount();
		
### get signal count

	uint16_t getSignalCount();
		
### get Status flag
		
	uint8_t getStatus();
	

 * @file DFRobot_VL53L0X.ino
 * @brief DFRobot's Laser rangefinder library
 * @n This example provides the VL53L0X laser rangefinder API function

 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 
 * @author [LiXin]
 * @version  V1.0
 * @date  2017-8-21
 * @https://github.com/DFRobot/DFRobot_VL53L0X