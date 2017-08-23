/*!
 * @file DFRobot_VL53L0X.ino
 * @brief DFRobot's Laser rangefinder library
 * @n The example shows the usage of VL53L0X in a simple way.

 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 
 * @author [LiXin]
 * @version  V1.0
 * @date  2017-8-21
 * @https://github.com/DFRobot/DFRobot_VL53L0X
 timer*/
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

/*****************Keywords instruction*****************/
//Continuous--->Continuous measurement model
//Single------->Single measurement mode
//High--------->Accuracy of 0.25 mm
//Low---------->Accuracy of 1 mm
/*****************Function instruction*****************/
//setMode(ModeState mode, PrecisionState precision)
  //*This function is used to set the VL53L0X mode
  //*mode: Set measurement mode       Continuous or Single
  //*precision: Set the precision     High or Low
//void start()
  //*This function is used to enabled VL53L0X
//float getDistance()
  //*This function is used to get the distance
//uint16_t getAmbientCount()
  //*This function is used to get the ambient count
//uint16_t getSignalCount()
  //*This function is used to get the signal count
//uint8_t getStatus();
  //*This function is used to get the status
//void stop()
  //*This function is used to stop measuring

DFRobotVL53L0X sensor;

float calcSpeed()
{
	uint32_t timer1,timer2;
	float inspeed1,inspeed2;
	timer1 = millis();
	inspeed1 = sensor.getDistance();
	delay(50);
	timer2 = millis();
	inspeed2 = sensor.getDistance();
	return (inspeed2-inspeed1)/(timer2-timer1);
}

void setup() {
  //initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set operational mode of laser rangefinder
  sensor.setMode(Continuous,High);
  //Laser rangefinder begins to work
  sensor.start();
}

void loop() 
{
	Serial.println("----- START TEST ----");
	Serial.print("ambient Count: ");Serial.println(sensor.getAmbientCount());
	Serial.print("Signal Count: ");Serial.println(sensor.getSignalCount());
	Serial.print("Distance: ");Serial.println(sensor.getDistance());
	Serial.print("Status: ");Serial.println(sensor.getStatus());
	Serial.print("Speed: ");Serial.println(calcSpeed());
	Serial.println("----- END TEST ----");
	Serial.println("");
	delay(1000);
}