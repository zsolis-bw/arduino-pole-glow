# GetMacAddress

## Overview
This project contains a PlatformIO sketch designed to print the MAC address of an ESP32-based device. It is helpful for determining the MAC address of devices for applications that require peer-to-peer communication, such as ESP-NOW.

## Hardware Requirements
- ESP32 board (compatible with PlatformIO)
- USB cable to connect the ESP32 to your computer

## Software Requirements
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO extension for VS Code](https://platformio.org/install/ide?install=vscode)

## How to Use
1. Open the `GetMacAddress` project in VS Code with PlatformIO.
2. Connect your ESP32 board to your computer using a USB cable.
3. Ensure that PlatformIO recognizes your ESP32 board by checking **PlatformIO Home > Devices**.
4. Build and upload the sketch using the `Upload` button in PlatformIO or by running the `pio run -t upload` command.
5. Open the Serial Monitor by clicking on the `PlatformIO: Serial Monitor` button or using the `pio device monitor` command.
6. The MAC address of the connected ESP32 will be printed to the Serial Monitor.

## Expected Output
> MAC Address: XX:XX:XX:XX:XX

## Troubleshooting
- Ensure that your board is specified correctly in the `platformio.ini` file.
- Verify the correct port is set in PlatformIO or run `pio device list` to check connected devices.