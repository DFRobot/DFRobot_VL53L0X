# VL53L0 Distance Ranging Sensor
World smallest Time-of-Flight ranging and gesture detection sensor
![SVG1](https://raw.githubusercontent.com/DFRobot/binaryfiles/master/SEN0245/SEN0245svg1.png)
---------------------------------------------------------

# DFRobot_VL53L0X Library for Arduino

This library provides the VL53L0X laser rangefinder API function

### Ready to start 

```C++
    //i2c_addr:Set I2C sub-device address
    void begin(uint8_t i2c_addr);
```

### Set operational mode to VL53L0X

```C++
    /*!
     *  @brief Set operational mode to VL53L0X
     *
     *  @param mode Work mode settings
     *      Single : Single mode
     *      Continuous : Back-to-back mode
     *  @param precision：Set measurement precision
     *      High：High precision(0.25mm)
     *      Low: Low precision(1mm)
     */
    void setMode(uint8_t mode, uint8_t precision);
```

### Start measuring distance

```C++
    void start();
```

### Stop measurement

```C++
    void stop();
```

### get distance data

```C++
    uint16_t getDistance();
```

### get ambient count

```C++
    uint16_t getAmbientCount();
```
### get signal count

```C++
    uint16_t getSignalCount();
```

### get Status flag

```C++
    uint8_t getStatus();
```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
FireBeetle-BLE4.1 |      √       |             |            | 

## Credits

Written by lixin(1035868977@qq.com), 2017. (Welcome to our [website](https://www.dfrobot.com/))