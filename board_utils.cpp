#include "board_utils.h"
#include "user_config.h"
#include <Arduino.h>

#ifdef USE_SOFT_PWM
#include <SoftPWM.h>
#endif

// local variables

// let's define the solenoid objects and their states
solenoid_t solenoids[NO_OF_SOLENOIDS];


void init_board(void)
{
  // initialize the solenoid pins
  for (int i = 0; i < NO_OF_SOLENOIDS; i++)
  {
    solenoids[i].pin = solenoidPins[i];
    solenoids[i].state = LOW;
    pinMode(solenoids[i].pin, OUTPUT);
    digitalWrite(solenoids[i].pin, solenoids[i].state);
  }
  #ifdef USE_SOFT_PWM
  SoftPWMBegin();
  #endif
  // initialize the pressure pins
  pinMode(PRESSURE_INCREASE_PIN, OUTPUT);
  pinMode(PRESSURE_DECREASE_PIN, OUTPUT);
  set_solenoid_pressure(0, 0);
  // Ensure solenoids off
  turn_off_solenoids();

  // Establish I/O
  pinMode(PSI_TRANSDUCER_PIN,    INPUT);
  pinMode(GRBL_DAT_PIN,       INPUT);
  pinMode(GRBL_SET_PIN,       INPUT_PULLUP);
  
  pinMode(FOOT_PEDAL_LEFT,    INPUT_PULLUP);
  pinMode(FOOT_PEDAL_RIGHT,   INPUT_PULLUP);

}



void set_solenoid_pressure(uint8_t increase_val, uint8_t decrease_val)
{
    #ifdef USE_SOFT_PWM
    SoftPWMSet(PRESSURE_INCREASE_PIN, increase_val);
    SoftPWMSet(PRESSURE_DECREASE_PIN, decrease_val);
    #else
    analogWrite(PRESSURE_INCREASE_PIN, increase_val);
    analogWrite(PRESSURE_DECREASE_PIN, decrease_val);
    #endif
 
}
void turn_off_solenoids(void)
{
  for (int i = 0; i < NO_OF_SOLENOIDS; i++)
  {
    solenoids[i].state = LOW;
    digitalWrite(solenoids[i].pin, solenoids[i].state);
  }
}

void update_solonoids_state(bool* solenoidState)
{
  for (int i = 0; i < NO_OF_SOLENOIDS; i++)
  {
    solenoids[i].state = solenoidState[i];
    digitalWrite(solenoids[i].pin, solenoids[i].state);
  }
}

void display_solenoid_state(void)
{
  Serial.print("Solenoid States:     [");
  for (int i = 0; i < NO_OF_SOLENOIDS-1; i++)
  {
    Serial.println(solenoids[i].state);
     Serial.print(",");
  }
  Serial.println(solenoids[NO_OF_SOLENOIDS-1].state);
  Serial.println("]");
}