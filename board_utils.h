#ifndef __BOARD_UTILS_H__
#define __BOARD_UTILS_H__
#include <Arduino.h>
typedef struct {
  uint8_t pin;
  uint8_t state;
} solenoid_t;

void init_board(void);
void set_solenoid_pressure(uint8_t increase_val, uint8_t decrease_val);
void turn_off_solenoids(void);
void update_solonoids_state(bool* solenoidState);
void display_solenoid_state(void);
#endif
