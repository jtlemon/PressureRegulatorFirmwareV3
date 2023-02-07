#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__
#include <Arduino.h>

#define USE_SOFT_PWM 1

// I/O Pins for Vacuum Solenoids
#define NO_OF_SOLENOIDS 10
const uint8_t solenoidPins[NO_OF_SOLENOIDS] = {2,3,4,12,13,7,8,9,10,11};

// I/O Pins for Pressure //
#define PSI_TRANSDUCER_PIN     A2  // Transducer to measure PSI in system.
#define PRESSURE_INCREASE_PIN  5  // Solenoid that lets air into system.
#define PRESSURE_DECREASE_PIN  6  // Solenoid that lets air out of system.


#define GRBL_DAT_PIN A7 // Pin to get PWM wave from grbl to control set point.
#define GRBL_SET_PIN A6 // Pin to control grblFlag.


#define FOOT_PEDAL_LEFT  A1
#define FOOT_PEDAL_RIGHT A0

#endif
