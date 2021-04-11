#include <Arduino.h>

// Settings to calibrate the steer

// Define ANGLE_CALIBRATE to print the measured angle
// to Serial
#define ANGLE_CALIBRATE
#define AUTO_CALIBRATE

// Next 2 values are used for calibration 
// Set to the raw value measured when steering to max left
uint16_t potValue_min = 1800; // Value of potmeter, when minimum angle = -40°

// Set to the raw value measured when steering to max right
uint16_t potValue_max = 2400; // Value of potmeter, when maximum angle =  40°

const uint16_t potValue_error = 10; // If beneath, something is wrong with the measurements

// Set to the value you want sent to swift, when steering to max right. 
const float zwift_angle_sensitivity = 30.0; // Default = 40.0°;
const float zwift_angle_direction   = -1.0; // 1.0 = normal -1.0 = reverse steering

#ifdef ANGLE_CALIBRATE
    // When calibrating always write debug info to serial.
    #define DEBUG_TO_SERIAL
#endif