#ifndef __MACHINE_INTERFACE_H__
#define __MACHINE_INTERFACE_H__
#include<Arduino.h>
#include <Arduino_AVRSTL.h>
#define BAUD_RATE 19200



void init_communication(void);
bool is_new_configurations_available(void);
bool is_new_fast_command_detected(void);
std::vector<String> get_received_data(void);
char get_last_fast_command(void);
#endif