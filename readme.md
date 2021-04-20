DIY Zwift Steering with ESP32
=============================

> Warning  
> ------
>
> __Work in progress__

Build a Zwift steering device using an ESP32 (development board).

## Software / Firmware
Target hardware is: MH-ET Live (ESP32 Development board)

## Hardware
3D printed. Modified STL files from Keith files.

Hardware list:
- MH-ET Live  
  https://docs.platformio.org/en/latest/boards/espressif32/mhetesp32devkit.html  
  [PinOut](hardware/img/1499504017768-1pinmap.jpg)

## Features

* BLE connection to Zwift
* Smoothing of measured data
* Automatic detection of measured interval  
  Not constructing the device, it is impossible to mount the potentiometer always in exact the same position. This feature will detect the highest & lowest measured values.
* Automatic correction of the center   
  When the device is not completely centered on the floor, you get a steering error. This feature will automatically correct and makes sure you steer straight.

# Quick manual

1. Turn on steering device
2. Detect the steering range
   1. Steer maximum to the right for 10 seconds
   1. Steer maximum to the left for 10 seconds
   1. The device is calibrated now
3. Start Zwift, click on the Steer button and connect.
4. When using Zwift, if you notice there is an offset for steering straight. 
   * Keep your steer like you want to steer straight (0° angle).
   * After a few minutes this will become your zero angle.
   

References
----------

* https://github.com/kwakeham/zwift-steerer/
* https://sourceforge.net/projects/zwift-steering-esp32/
