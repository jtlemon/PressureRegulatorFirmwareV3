#ifndef __MACHINE_INTERFACE_H__
#define __MACHINE_INTERFACE_H__
#include<Arduino.h>
#define BAUD_RATE 115200

# define numOfSolenoids 10

typedef struct {
  bool onOff;
  double set_point;
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
void get_received_data(machine_setting_t&);
char get_last_fast_command(void);
#endif