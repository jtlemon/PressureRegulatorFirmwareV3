# THE MODI COMPANY
```
Author:     Trevor S. McFarland
Partner:    Mohamed Mejia
Date:       January 22, 2022
Company:    The MODI Company
GitHub ID:  tmcfarland17
Repo:       https://github.com/tmcfarland17/ArduinoProjects.git
Commit #:   
Solution:   PressureRegulatorFirmwareV2
Copyright:  The MODI Company. 
```
# Project Overview

Contains firmware written for the I/O remote board. Written for Arduino Nano. Used on Sanding Machine by The MODI Company.
INPUTS:
- Two foot pedals     - Each signifying the active state of the two sides of the machine. 
- Pressure Transducer - Monitors pressure in the air line leading to the pistons whereon the sanders are mounted.
- Serial Input        - Data from main controller input. Current input format - 
		      - $double,bool,bool,bool,bool,bool,bool,bool,bool,bool,bool,bool,double,double,double,int,int*
		      - $setPoint, machineState, sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8, sol9, sol10, kp, ki, kd, centerMargin, sampleTime*
OUTPUTS:
- 5V  Solenoids (x2)  - One allows more air into pressure line monitored by transducer, the other lets air out. Calculation performed by PID to 
			control which solenoid currently active. 
- 24V Solenoids (x10) - Solenoids controlling vacuum pods on sanding table. State values given by serial input. 
- Serial Output       - Outputs which foot pedal is currently active. 

# Updates

January 22, 2022: 
- Switch from general machine state being controlled by foot pedals to foot pedal states being seperated differentiating between the left and right sides of the mahcine. 
- Serial output function added, to send data back to the main controller board. 

March 19, 2022:
- Complete rehaul of PID to use two PIDs, one for each valve.
- Ouput valves also now utilizing variable solenoid valves controlled via PWMs. 
