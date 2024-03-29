#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__
#include <Arduino.h>

//#define USE_SOFT_PWM 0

// I/O Pins for Vacuum Solenoids
#define NO_OF_SOLENOIDS 10
//const uint8_t solenoidPins[NO_OF_SOLENOIDS] = {2,3,4,12,13,7,8,9,10,11};
const uint8_t solenoidPins[NO_OF_SOLENOIDS] = {13, 3, 12, 11, 8, 7, 4, A6, A3, A2};


// I/O Pins for Pressure //
#define PRESSURE_INCREASE_PIN  5  // Solenoid that lets air into system.
#define PRESSURE_DECREASE_PIN  6  // Solenoid that lets air out of system.


#define GRBL_SET_PIN A1 // Pin to control grblFlag.                                 -FP1 on Board
#define GRBL_DAT_PIN A0 // Pin to get PWM wave from grbl to control set point.      -FP2 on Board

#endif

