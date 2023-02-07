//////////////////////////////////////////////////////////////////////////////////////////
//                            Pressure Regulator Firmware v1                            //
//                                        MODI                                          //
//                            Last Update: January 25, 2023                             // 
//                                 By: Trevor McFarland                                 //
//////////////////////////////////////////////////////////////////////////////////////////

/* 
 *  Input(s):
 *   0-30PSI Pressure Transducer
 *   Serial Data
 *   Momentary Foot Pedals (x2)
 *  Output(s):
 *   5V Solenoid Valve (x2)
 *   24V Solenoid Valve (x10)
 *   
 *  Desired Result:
 *   One solenoid allows pushes more pressure into line (up to 30 PSI), 
 *   other solenoid allows pressure out of line. Based on transducer 
 *   readings and PID techniques, balance PSI in line to be 5 PSI. 
 *   
 *   Based on Arduino PID Library by Brett Beauregard
 *   
 *   Also to drive 10 solenoids based on serial data input. 
*/

#include <PID_v1.h>
#include "TimerOne.h"
#include "machine_interface.h"
#include "user_config.h"
#include "board_utils.h"






// PID Variables //
double psiInput;         // Analog reads from PSI transducer.
double psiInputInvert;   // An inverse of the psiInput based on the zero psi reading.
double psiIncrease;      // Output by PID for input  air PWM (0-255)
double pidSetPoint   = 0;   // Relative PSI, calculated amount
bool   grblFlag   = 0;   // Bool stating whether set point is being sent over serial (0) or via pin input from grbl (1).


// kp, ki, kd are to control the PID with desired aggressiveness\
// See https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID for a math heavy explanation, alternatively 
// See https://www.mathworks.com/discovery/pid-control.html for a more intuition based video explanation. 
double kp         = 50;   // How aggressivle the PID responsds to current amount of error.
double ki         = 3.5;   // How aggressivle the PID responsds to error over time. 
double kd         = 0;  // How aggressivle the PID responsds to rate of change of error.
int sampleTime    = 20; // How often a new output is calculated in milliseconds.

// the target is to maxmize the error 
PID pidI(&psiInput, &psiIncrease, &pidSetPoint, kp, ki, kd, P_ON_E, DIRECT);



// Variable to signify if control valves are normally open (true) or normally closed (false) //
bool NO = false;


// Variables For PSI Tuning //
int zeroPSI        = 0;     // Transducer input under no pressure (in steps). 
int maxPSI         = 1023;  // Maximum PSI the system can deliver (must be under maxSystemPSI).                       ***
int maxSystemPSI   = 30;    // Max PSI limit either the system can handle, or the item receiving the pressure from the system can handle (in PSI).                      ***
int transducerStep = 15;    // Transducer Max PSI Reading.
int multFactor     = ((1023-zeroPSI)/transducerStep);                            // Calculate multiplication factor from transducerStep.
int maxSystemStep  = ((maxSystemPSI*((1023-zeroPSI)/transducerStep)) + zeroPSI); // Convert MaxSystemPSI to analog step input amount.
int calibrateTime  = 10000; // How much time is needed for system to zero in on a given pressure (in milliseconds).
int accuracy       = 50;    // This is for using two state solenoid valves. The PID outputs a PWM between 0 and 255 to control the different 
                            // solenoid valves. For a bool state system, this value acts as the the tripping point value to compare to the 
                            // PID output. The higher this number, the higher the PID output needs to be to open either of the solenoids. 
                            // Thus, a low value would result in more aggressive behavior, while a higher number would result in a less 
                            // aggressive behavior. 




// Foot Pedal Variables and I/O //
bool inputMachineState     = false;
volatile bool machineState = false;     // System on or off.

const uint8_t debounceTime = 5;         // Interval for debounce time.
bool currentInputState;                 // Used for debouncer.
volatile int tickCounter;               // Used for debouncer.
const unsigned int updateEvery = 500;   // Interval (ms) to update machine information.
const int interruptTime = 6;            // In milliseconds

typedef struct {
  int  pinNumber;
  int  debouncePressCounter;
  int  debounceLiftCounter;
  int  debounceLiftedCounter;
  bool pressed;
  bool footCurrentState;
}inputControl;

inputControl footPedals[2] = {
  {FOOT_PEDAL_LEFT,  0, 0, 0, false, false},  // Foot pedal on left.
  {FOOT_PEDAL_RIGHT, 0, 0, 0, false, false},  // Foot pedal on right.
};


// Vector of current machine values - used in being able to update only portions of current settings. 
machine_setting_t current_settings = {
  false, pidSetPoint, {false, false, false, false, false, false, false, false, false, false}, kp, ki, kd, accuracy, sampleTime
};


bool testingState = false;
int  printInterval = 60;

volatile uint8_t pid_tick_counter = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////     SET UP     /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

long lastTime = 0;

void setup() {
  // Begin Serial Communication
  init_communication();
  init_board();
  // Set up PID
  pidI.SetMode(AUTOMATIC);
  pidI.SetSampleTime(sampleTime);
  

  // Establish Timer and Create Interrupt Every 6ms //
  Timer1.initialize(interruptTime*1000);
  Timer1.attachInterrupt(timerIsr);
    // Calibrate
  //calibrateZero();
  //calibrateMax();
  footPedals[0].footCurrentState = false;
  footPedals[1].footCurrentState = false;

}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////      LOOP      /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  // See where set point data should be coming from.
  if (digitalRead(GRBL_SET_PIN)!= HIGH) // Pullup
  {
    grblFlag = true;
    current_settings.onOff = true; //machine is ON BASED on hardware input
    inputMachineState = true;
  }
  else
  {
    // Until timing is figured out, when the spindal output is disengaged, disengage all vacuum solenoinds. 
    if (grblFlag == true)
    {
      turn_off_solenoids();
      current_settings.onOff = false;
      inputMachineState = false;
    }
    grblFlag = false;
  }
  
  if(is_new_fast_command_detected())
  {
    char command = get_last_fast_command();
    if(command == '?')printStatus();
    else if (command == '!'){
      testingState = !testingState;
      printInterval = testingState ? 800 : 60;
    }
  }
  if(is_new_configurations_available())
  {
    get_received_data(current_settings);
    // we have the new settings, now we need to update the PID
    pidI.SetTunings(current_settings.kp_value, current_settings.ki_value, current_settings.kd_value);
    pidI.SetSampleTime(current_settings.sample_time_value_ms);
    if(current_settings.onOff == false)turn_off_solenoids();
    else update_solonoids_state(current_settings.solenoidState);
    updateSetPoint(current_settings.set_point); 

  } 
  else 
  {
    updateSetPoint(current_settings.set_point);
  }
  long now = millis();
  if(now - lastTime > 100)
  {
    lastTime = now;
    Serial.print(pidSetPoint);
    Serial.print(" ");
    Serial.println(psiInput);
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// HELPER METHODS /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////








/*
 * Method run each timer driven interrupt. Method to check inputs from foot pedals and
 * update the associated side machine states.
 * Parameters:
 *    None.
 * Return:
 *    Nothing.
 */
void timerIsr() {
  //Serial.println("Entering Interrupt");
  pid_tick_counter++;
  for(uint8_t i=0; i<2; i++) {
    
    // Get input for each foot pedal.
    currentInputState = digitalRead(footPedals[i].pinNumber); 
    // Invert due to pullup.
    currentInputState = !currentInputState;

    if (currentInputState == HIGH) 
    {
      // Make it so there is just a burst of pressed data, so that 
      // the button does not continuously say that it is on. 
      footPedals[i].debounceLiftedCounter = 0;
      footPedals[i].debounceLiftCounter++;
      if(footPedals[i].debounceLiftCounter >= debounceTime*4) {
        footPedals[i].footCurrentState = false;
        footPedals[i].pressed = true;
        // Reset debounce counter.
        footPedals[i].debounceLiftCounter = 0;
      }
    }
    else
    {
      footPedals[i].pressed = false;
      /*
        footPedals[i].debounceLiftCounter = 0;
        footPedals[i].debounceLiftedCounter++;
        if(footPedals[i].debounceLiftedCounter >= debounceTime) {
          footPedals[i].pressed == false;
          // Reset debounce counter.
          footPedals[i].debounceLiftedCounter = 0;
        } */
    }
    
    // If differrent than the last recroded input...
    //if((currentInputState != footPedals[i].footCurrentState)) {
    if((currentInputState != footPedals[i].footCurrentState) && (footPedals[i].pressed == false)) {
      // Increase debounce counter.
      footPedals[i].debouncePressCounter++;
      // If enough time has passed where the pedal has not changed state.
      if(footPedals[i].debouncePressCounter >= debounceTime) {
        footPedals[i].footCurrentState = currentInputState;
        // Reset debounce counter.
        footPedals[i].debouncePressCounter = 0;
      }
    } else {
      // Otherwise, ensure debounce counter is at 0;
      footPedals[i].debouncePressCounter = 0;
    } 
  }
  if(pid_tick_counter >= current_settings.sample_time_value_ms) {
    pid_tick_counter = 0;
    // Compute PID and Alter Valves //
    psiInput = analogRead(PSI_TRANSDUCER_PIN);
    pidI.Compute();
    set_solenoid_pressure(psiIncrease, 255 - psiIncrease);
  }
  
  //@todo: This is a hack to get the serial output to work.
  return ;
  // Update information AND at readable intervals print relevant information.
  tickCounter++;
  if(tickCounter >= printInterval/interruptTime) {
   
    if (testingState) {
      testingPrints();
    }
    else
    {
      serialOutput();
    }
    
    tickCounter = 0;
  }
  
  //Serial.println("Leaving Interrupt"); // For Testing purposes.
}






// Method just for updating the set point.
void updateSetPoint(float serialSetPoint) 
{
  int setPointTemp;
  if (grblFlag == true) 
  {
    // Get set point from the pin controlled by gerbal. 
    // Should meausure a voltage on scale between 0 and 1023.
    pidSetPoint = analogRead(GRBL_DAT_PIN);
  }
  else 
  {
    // Look at Set Point from serial data. 
    //@todo i should map and constrain this value.
    pidSetPoint = serialSetPoint;
  }
}








/*
 * Method to Calibrate variable "zeroPSI".
 * Parameters: None
 * Return: Nothing
 */
void calibrateZero() {
  Serial.println();
  Serial.println("Calibrating Zero");
  
  // Open line up to release all pressure.
  if (NO == true) {
    set_solenoid_pressure(255, 0);
  } else {
    set_solenoid_pressure(0, 255);
  }

  // Wait for pressure to be released.
  delay(calibrateTime);

  // Set the zeroPSI variable to equal whatever the transducer reads when no pressure in the system. 
  zeroPSI = analogRead(PSI_TRANSDUCER_PIN);

  Serial.println();
  Serial.println("Done Calibrating Zero");
}

/*
 * Method to calibrate variable "maxPSI".
 * Parameters: None
 * Return: Nothing
 */
void calibrateMax() {
  
  Serial.println();
  Serial.println("Calibrating Max");

  // Read pressure
  psiInput = analogRead(PSI_TRANSDUCER_PIN);
  
  // Fill line up with air, stop at either max air system can handle, or max reading available from transducer.
  while((psiInput < maxSystemStep) && (psiInput < 1023)) {

    // Read pressure
    psiInput = analogRead(PSI_TRANSDUCER_PIN);
    
    if (NO == true) {
      set_solenoid_pressure(0, 255);
    } else {
      set_solenoid_pressure(255, 0);
    }
  }

  // Record pressure when at max. 
  maxPSI = analogRead(PSI_TRANSDUCER_PIN);

  // Start letting air out of system.
  if (NO == true) {
    set_solenoid_pressure(255, 0);

  } else {
    set_solenoid_pressure(0, 255);
  }
}

/*
 * Method to print potentially valuable information in the testing process.
 * Parameters: 
 *    Time of current loop. 
 *    Time since last interupt.
 * Return: 
 *    Nothing. <
 */
void testingPrints() {
  //if (currentTime >= timerTime+1000) {
    Serial.println();
    Serial.print("MACHINE STATE:       ");
    Serial.println(machineState);
    Serial.print("Input Machine State: ");
    Serial.println(inputMachineState);
    Serial.print("GRBL CONTROL:        ");
    Serial.println(grblFlag);
    Serial.println();
    
    Serial.print("Left Foot Pedal State:    ");
    Serial.println(footPedals[0].footCurrentState);
    Serial.print("Right Foot Pedal State:   ");
    Serial.println(footPedals[1].footCurrentState);
    
    display_solenoid_state();
    
    Serial.print("Zero PSI:            ");
    Serial.println(zeroPSI);
    Serial.print("Max PSI (steps):     ");
    Serial.println(maxPSI);
    Serial.print("kp:                  ");
    Serial.println(current_settings.kp_value);
    Serial.print("ki:                  ");
    Serial.println(current_settings.ki_value);
    Serial.print("kd:                  ");
    Serial.println(current_settings.kd_value);
    Serial.print("Sample Time:         ");
    Serial.println(current_settings.sample_time_value_ms);   
    Serial.print("Set Point:           ");
    Serial.println(pidSetPoint);   
    Serial.print("Transducer Input:    ");
    Serial.println(psiInput);
    Serial.print("PSI Increase Step:   ");
    Serial.println(psiIncrease);
    Serial.println();
    //timerTime = currentTime;
  //}
}

/*
 * Method to output needed information back to the main controller board.
 * Format: $Left_Foot_Pedal_State,Right_Foot_Pedal_State*
 * Parameters:
 *    None.
 * Return:
 *    Nothing.
 */
void serialOutput() 
{
  Serial.print("$");
  Serial.print(footPedals[0].footCurrentState);
  Serial.print(",");
  Serial.print(footPedals[1].footCurrentState);
  Serial.println("*");
}

void printStatus(void){
  Serial.print(current_settings.onOff);
  Serial.print(",");
  Serial.print(current_settings.set_point);
  for(int i=0; i<10; i++){
    Serial.print(",");
    Serial.print(current_settings.solenoidState[i]);
  }
  Serial.print(current_settings.kp_value);
  Serial.print(",");
  Serial.print(current_settings.ki_value);
  Serial.print(",");
  Serial.print(current_settings.kd_value);
  Serial.print(",");
  Serial.print(current_settings.accuracy_value);
  Serial.print(",");
  Serial.println(current_settings.sample_time_value_ms);
}

// I don't know if Arduino has a method like this for arrays
bool contains(int arr[], int testValue) 
{
  for (int i=0; i<sizeof(arr); i++)
  {
    if (arr[i] == testValue) 
    {
      return true;
    }
  }
  return false;
}
