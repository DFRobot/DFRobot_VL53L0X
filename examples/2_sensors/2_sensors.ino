
#include "DFRobot_VL53L0X.h"

DFRobot_VL53L0X sensor1;
DFRobot_VL53L0X sensor2;

// start with one sensor unpowered, or disconected from I2C bus

void setup() {
  delay(10);
  Serial.begin(9600);
  Wire.begin();
  
  // change I2C address of plugged in sensor
  sensor1.begin(0x40, true);
  sensor1.setMode(sensor1.eContinuous,sensor1.eHigh);
  sensor1.start();

  // tell user to plug in new sensor and wait till acknolegement
  Serial.println("Plug in second sensor now!");
  Serial.println("Then send anything through serial.");
  while(Serial.available() == 0) { }

  //connect to second sensor  
  sensor2.begin(0x29);
  sensor2.setMode(sensor2.eContinuous,sensor2.eHigh);
  sensor2.start();

  // tell user the addresses of the two sensors
  delay(10);
  Serial.print("address 1: 0x");
  Serial.println(sensor1._detailedData.i2cDevAddr,HEX);
  Serial.print("address 2: 0x");
  Serial.println(sensor2._detailedData.i2cDevAddr,HEX);
  delay(10);
}

void loop()
{
  Serial.print("Distance 1: ");
  Serial.print(sensor1.getDistance());
  Serial.print("  Distance 2: ");
  Serial.println(sensor2.getDistance()-20);
    
  delay(1000);
}
