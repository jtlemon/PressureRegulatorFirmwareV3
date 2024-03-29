#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "machine_interface.h"

bool new_configurations_available = false;
HardwareSerial &serial_dev = Serial;

volatile bool is_configuration_decded = false;
String last_received_string = "";
String received_chars = "";
volatile uint8_t comma_counter = 0;
machine_setting_t decoded_data;

volatile bool new_fast_command_available = false;
volatile char last_fast_command = '0';

void init_communication()
{
    serial_dev.begin(BAUD_RATE);
    delay(300);
}
bool is_new_configurations_available()
{
    return new_configurations_available;
}
bool is_new_fast_command_detected(void)
{
    return new_fast_command_available;
}

char get_last_fast_command(void)
{
    new_fast_command_available = false;
    return last_fast_command;
}

void get_received_data(machine_setting_t& decoded_data)
{
    new_configurations_available = false;
    if (!is_configuration_decded)
    {
        uint8_t strlen = last_received_string.length();
        double splitted_string[17];
        String current_str = "";
        uint8_t count = 0;
        for (int i = 0; i < strlen; i++)
        {
            if (last_received_string.charAt(i) == ',')
            {
                splitted_string[count] = current_str.toDouble();
                count++;
                current_str = "";
            }
            else
            {
                current_str += last_received_string.charAt(i);
            }
        }
        splitted_string[count] = current_str.toDouble();
        count++;
        //Serial.println(count);
        switch (count)
        {
        case 1:
            decoded_data.onOff = splitted_string[0] > 0;
            break;
        case 2:
            decoded_data.onOff = splitted_string[0] > 0;
            decoded_data.set_point = (double)splitted_string[1];
          //Serial.println( decoded_data.set_point);
            break;
        case numOfSolenoids:
            for (int i = 0; i < numOfSolenoids; i++)
            {
                decoded_data.solenoidState[i] = splitted_string[i] > 0;
            }
            break;
        case numOfSolenoids + 2:
            decoded_data.onOff = splitted_string[0] > 0;
            decoded_data.set_point = (double)splitted_string[1];
            for (int i = 0; i < numOfSolenoids; i++)
            {
                decoded_data.solenoidState[i] = splitted_string[2 + i] > 0;
            }
            break;
        case 17:
            decoded_data.onOff = splitted_string[0] > 0;
            decoded_data.set_point = (double)splitted_string[1];
            for (int i = 0; i < numOfSolenoids; i++)
            {
                decoded_data.solenoidState[i] = splitted_string[2 + i] > 0;
            }
            decoded_data.kp_value = splitted_string[numOfSolenoids + 2];
            decoded_data.ki_value = splitted_string[numOfSolenoids + 3];
            decoded_data.kd_value = splitted_string[numOfSolenoids + 4];
            decoded_data.accuracy_value = (int)splitted_string[numOfSolenoids + 5];
            decoded_data.sample_time_value_ms = (int)splitted_string[numOfSolenoids + 6];
            break;

        default:
            break;
        }

        is_configuration_decded = true;
    }
    return decoded_data;
}

void serialEvent()
{
    if (serial_dev.available())
    {
        char inChar = (char)serial_dev.read();
        if (inChar == '$')
        {
            // start of packet
            received_chars = "";
            comma_counter = 0;
        }
        else if (inChar == '!' || inChar == '?')
        {
            last_fast_command = inChar;
            new_fast_command_available = true;
            received_chars = "";
            comma_counter = 0;
        }
        else if (inChar == '!')
        {
            // end of packet
            received_chars = "";
            comma_counter = 0;
        }
        else if (inChar == '*')
        {
            if (received_chars.length() > 0 && ((comma_counter == 0) || (comma_counter == 1) || (comma_counter == 9) || (comma_counter == 11) || (comma_counter == 16) || (comma_counter == 17)))
            {
                // this a valid packet
                is_configuration_decded = false;
                last_received_string = received_chars;
                new_configurations_available = true;
            }
            else
            {
                received_chars = "";
                comma_counter = 0;
            }
        }
        else
        {
            received_chars += String(inChar);
            if (inChar == ',')
                comma_counter += 1;
        }
    }
}


void copy_solenoid_state_only(machine_setting_t* src, machine_setting_t* dst)
{
    for (int i = 0; i < numOfSolenoids; i++)
    {
        dst->solenoidState[i] = src->solenoidState[i];
    }

}