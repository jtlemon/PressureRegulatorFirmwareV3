#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino_AVRSTL.h>
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


std::vector<String> splitString(String originalString, String delimiter);

void init_communication()
{
    serial_dev.begin(BAUD_RATE);
    delay(300);
}
bool is_new_configurations_available()
{
    return new_configurations_available;
}
bool is_new_fast_command_detected(void){
    return new_fast_command_available;
}

char get_last_fast_command(void){
    new_fast_command_available = false;
    return last_fast_command;
}

machine_setting_t get_received_data()
{
    new_configurations_available = false;
    if (!is_configuration_decded)
    {
        std::vector<String> splitted_string = splitString(last_received_string, ",");
        if (splitted_string.size() == 17)
        {
            decoded_data.onOff = splitted_string[0].toInt();
            decoded_data.set_point = splitted_string[1].toInt();
            for (int i = 0; i < numOfSolenoids; i++)
            {
                decoded_data.solenoidState[i] = splitted_string[2 + i].toInt();
            }
            decoded_data.kp_value = splitted_string[numOfSolenoids+2].toDouble();
            decoded_data.ki_value = splitted_string[numOfSolenoids+3].toDouble();
            decoded_data.kd_value = splitted_string[numOfSolenoids+4].toDouble();
            decoded_data.accuracy_value = splitted_string[numOfSolenoids+5].toInt();
            decoded_data.sample_time_value_ms = splitted_string[numOfSolenoids+6].toInt();
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
            
        }else if(inChar == '!' || inChar == '?'){
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

std::vector<String> splitString(String originalString, String delimiter)
{
    std::vector<String> supStrings;
    String s = originalString;
    int pos = 0;
    while ((pos = s.indexOf(delimiter)) != -1)
    {
        String token = s.substring(0, pos);
        if (token.length() > 0)
            supStrings.push_back(token);
        s = s.substring(pos + delimiter.length());
    }

    if (s.length() > 0)
        supStrings.push_back(s);
    return supStrings;
}
