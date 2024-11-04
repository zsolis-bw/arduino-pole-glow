# PoleEffects

## Overview
The PoleEffects project is a multi-device PlatformIO-based setup for synchronized LED effects designed to run on multiple ESP32 devices. It supports a variety of LED strip effects triggered by GPS and accelerometer data and is used for activities such as night skiing.

## Features
- Syncs LED effects across multiple ESP32 devices using ESP-NOW.
- Supports built-in NeoPixel status LED for mode indication.
- Uses GPS data to control LED effects like scrolling rainbow speed.
- Incorporates accelerometer data for direction-based effects.

## Dev Kit Hardware
- [Adafruit PowerBoost 1000 Charger](https://www.adafruit.com/product/2465)
  - Can charge the LiPo battery and boost the voltage to the 5v needed for the LED strips
- [Adafruit QT Py ESP32](https://www.adafruit.com/product/2465)
  - Any ESP32 will work, but these are compact and support STEMMA QT for adding sensors easily
- [Adafruit Mini GPS](https://www.adafruit.com/product/4415)
  - Compact GPS sensor
- [Adafruit Accel and Gyro Sensor](https://www.adafruit.com/product/3886)
  - The more basic of the accelerometer sensors with only 6 degrees of freedom
- [Adafruit LiPo 2000mAh Battery](https://www.adafruit.com/product/2011)
  - Any 2000mAh 3.7v battery will be supported on this kit
- [Adafruit STEMMA QT / Qwiic JST SH 4-Pin Cable](https://www.adafruit.com/product/4399)
  - One for each sensor in the system

## Software Requirements
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO extension for VS Code](https://platformio.org/install/ide?install=vscode)
- PlatformIO dependencies specified in `platformio.ini`:
  - ESP32 platform
  - Adafruit_NeoPixel library
  - Adafruit_GFX library
  - Adafruit_GPS library
  - Adafruit_MPU6050 library
  - Adafruit_Sensor library

## How to Use
1. Open the `PoleEffects` project in VS Code with PlatformIO.
2. Connect your ESP32 boards to your computer using USB cables.
3. Build and upload the sketch to each board using the `Upload` button or the `pio run -t upload` command.
4. Power the boards with a 3.7V LiPo battery or USB connection.
5. The boards will communicate via ESP-NOW and synchronize LED effects based on the mode selected with the on-board button.

## Configuration
- The number of LEDs can be adjusted by changing `#define NUM_LEDS` in `main.cpp`.
- MAC addresses of connected boards should be configured in the `main.cpp` for ESP-NOW communication.
- Ensure GPS and accelerometer are connected to the appropriate I2C ports.

## Troubleshooting
- Verify that the MAC addresses of all devices are correctly set.
- Check if the LED strips are connected and powered properly.
- Ensure that sensor connections are correct if GPS- or direction-based effects are not working.
- Confirm that all required libraries are installed as specified in `platformio.ini`.

## Expected Behaviors
- The built-in LED will indicate the mode:
  - Red: "Off" mode
  - Green: Scrolling rainbow
  - Blue: Directional strobe
  - Yellow: Comet tail
  - Cyan: Twinkle burst
  - Magenta: Breathing effect
- LED strip effects will synchronize across devices when connected via ESP-NOW.