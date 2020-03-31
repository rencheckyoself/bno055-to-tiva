# BNO055 on the Tiva

## Description
This repository contains the files to use the BNO055 driver provided by Bosch on the Tiva platform. Currently it is only tested with the Tiva C Series Launchpads, specifically the TM4C123GXL.

### File Descriptions:
- `init_imu.h`: I2C and IMU initialization functions as well as and some wrapper functions to interact with the IMU.
- `quaternion.h`: Simple Quaternion conversion functions.
- `examples/calibrated_position.c`: Example file for how to use the various provided functions. This file will first loop until the device is fully calibrated then read the sensor for the absolute position.

## How to use:
Place the files in the proper location to integrate with your current project.

1. Download/clone the [Bosch BNO055 driver](https://github.com/BoschSensortec/BNO055_driver) and place it in your project location.
2. Download/clone this repository and place it in your project location. Ensure your project is connected to Tiva's driverlib files to properly compile.
3. Copy the following code into your main file to initialize the I2C peripheral and the BNO055 sensor.

```
ConfigureI2C();
init_imu();
```

That's it! I would suggest digging into the Bosch driver and the sesnor data sheet to figure out what functions they offer and how to use their functions properly. I haven't found a hosted API but the driver if fully commented with doxygen so you could just generate your own if you don't want to read through the giant source code files.

## Quaternion Conversion
Also included in the repo is a small quaternion conversion file to interact with the absolute position data returned by the sensor. In short, a quaternion is a method of uniquely representing an orientation in 3D space, for more details checkout the wikipedia page. The available functions are currently targeted at only converting from a quaternion to euler angles.

## Useful Resources:
- [BNO055 Data Sheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [BNO055 Driver Quickstart Guide](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf)
- [Adafruit BNO055 Breakout Board](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
