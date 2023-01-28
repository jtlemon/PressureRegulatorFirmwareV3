#ifndef __MACHINE_INTERFACE_H__
#define __MACHINE_INTERFACE_H__
#include<Arduino.h>
#include <Arduino_AVRSTL.h>
#define BAUD_RATE 19200

# define numOfSolenoids 10

typedef struct {
  bool onOff;
  uint16_t set_point;
  bool solenoidState[numOfSolenoids];
  double kp_value;
  double ki_value;
  double kd_value;
  int accuracy_value;
  int sample_time_value_ms;
} machine_setting_t;



void init_communication(void);
bool is_new_configurations_available(void);
bool is_new_fast_command_detected(void);
machine_setting_t get_received_data(void);
char get_last_fast_command(void);
#endif