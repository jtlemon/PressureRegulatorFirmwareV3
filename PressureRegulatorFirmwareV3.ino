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

#include <PID_v1_bc.h>
#include "TimerOne.h"
#include "machine_interface.h"
#include "user_config.h"
#include "board_utils.h"
#include <Wire.h>

#define OUTPUT_MAX 14745.0 // 90% of 2^14
#define OUTPUT_MIN 1638.0 // 10%
#define PRESSURE_MAX 30.0
#define PRESSURE_MIN 0.0



// PID Variables //
double psiInput;         // Analog reads from PSI transducer.
double psiInputInvert;   // An inverse of the psiInput based on the zero psi reading.
double psiIncrease;      // Output by PID for input  air PWM (0-255)
double pidSetPoint   = 900;   // Relative PSI, calculated amount
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




// Update Timing
const unsigned int SEND_BOARD_STATUS_EVERY = 500;   // Interval (ms) to update machine information.
#define TICK_TIME_US  100  // In microseconds

// Vector of current machine values - used in being able to update only portions of current settings. 
machine_setting_t current_settings = {
  false, pidSetPoint, {false, false, false, false, false, false, false, false, false, false}, kp, ki, kd, accuracy, sampleTime
};

// some variables for debugging
bool testingState = false;
int  printInterval = 60;
bool updateStatusFlag = false;
// since the target is to measure the pwm signal on GRBL_DAT_PIN so i will so this from the interupt and I have to track last pin state
bool lastGrblDataPinStata = LOW;
// i want to make a variable for high and low time of the pulse
volatile uint32_t grblPulseWidthHighTicks  = 0; // in ticks 
volatile uint32_t grblPulseWidth = 0; // this in micro seconds

volatile uint32_t noPulseChangeTickCounter  = 0; // in ticks
volatile bool countRaisingEdgeFlag = false;

#define GRBL_DEAD_TICKS (((uint32_t) 1000 * 1000) / TICK_TIME_US) // 1000 ms to wait for a pulse to be considered dead

volatile bool lastGrblDataPinState = LOW;


// enum variable to monitor the edge 
typedef enum {
  GRBL_PULSE_STATE_LOW,
  GRBL_PULSE_STATE_HIGH,

} grbl_pulse_state_t;

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////     SET UP     /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

long lastTime = 0;
long lastTimePressure = 0;
long lastTimePrint = 0;
void setup() {
  // Begin Serial Communication
  init_communication();
  init_board();
  Wire.begin();
  // Set up PID
  pidI.SetMode(AUTOMATIC);
  pidI.SetSampleTime(sampleTime);
   // Set up pins for solenoids //
  // initiate the pin state for the GRBL_DAT_PIN
  lastGrblDataPinStata = digitalRead(GRBL_DAT_PIN);

  // Establish Timer and Create Interrupt Every 6ms //
  Timer1.initialize(TICK_TIME_US);
  Timer1.attachInterrupt(timerIsr);

 

}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////      LOOP      /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  // See where set point data should be coming from.
  if (digitalRead(GRBL_SET_PIN) == HIGH) // Pullup
  {
    current_settings.onOff = true; //machine is ON BASED on hardware input
    grblFlag = true;
  }
  else
  {
    // Until timing is figured out, when the spindal output is disengaged, disengage all vacuum solenoinds. 
    if (grblFlag == true)
    {
      current_settings.onOff = false;
      turn_off_solenoids();
    }
    grblFlag = false;
  }
  
  if(is_new_fast_command_detected())
  {
    char command = get_last_fast_command();
    if(command == '?') serialOutput();
    else if (command == '!'){
      Serial.println("start testing mode");
      testingState = !testingState;
      printInterval = testingState ? 800 : 60;
    }
  }
  if(is_new_configurations_available())
  {
    // create a copy of the current settings
    machine_setting_t current_settings_copy = current_settings;
    // get the new settings
    get_received_data(current_settings_copy);
    
    if(grblFlag){
          // we will update only the solenoidState
          copy_solenoid_state_only(&current_settings_copy, &current_settings);
    }
    else current_settings = current_settings_copy;
    if (current_settings.onOff) update_solonoids_state(current_settings_copy.solenoidState);
    else turn_off_solenoids();
     
    // update set point
    updateSetPoint(current_settings.set_point);
  } // end if new configurations available
  else 
  {
    updateSetPoint(current_settings.set_point);
  }

  // Update PID
  long now = millis();
  if(now - lastTimePressure > current_settings.sample_time_value_ms)
  {
      read_pressure();
      pidI.Compute();
      if (current_settings.onOff) set_solenoid_pressure(psiIncrease, 255 - psiIncrease);
      else set_solenoid_pressure(1, 0); // Turn pressure on so that the sanders ensured to retract.
      lastTimePressure = millis();
  }

  // Update information AND at readable intervals print relevant information.
  if(now - lastTimePrint > SEND_BOARD_STATUS_EVERY)
  {
    // Update information
    
    lastTimePrint = millis();
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// HELPER METHODS /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////



/*
 * Method run each timer driven interrupt. Method to
 * update the associated side machine states.
 * Parameters:
 *    None.
 * Return:
 *    Nothing.
 */
void timerIsr() {
  bool grblDataPinState = digitalRead(GRBL_DAT_PIN);
  if(grblDataPinState == HIGH && countRaisingEdgeFlag) grblPulseWidthHighTicks++;

  if (grblDataPinState != lastGrblDataPinState) {
    // We have a change in state
    if (grblDataPinState == HIGH) {
      countRaisingEdgeFlag = true;
    }
    else {
      grblPulseWidth = grblPulseWidthHighTicks * TICK_TIME_US;
      grblPulseWidthHighTicks = 0;
    }
    noPulseChangeTickCounter = 0;
    lastGrblDataPinState = grblDataPinState;
  }
  else {
    // no change in state
    noPulseChangeTickCounter++;
  }


  if(noPulseChangeTickCounter > GRBL_DEAD_TICKS)
  {
    // we have a dead pulse
    grblPulseWidthHighTicks = 0;
    grblPulseWidth = 0;
    noPulseChangeTickCounter = GRBL_DEAD_TICKS + 1; // just to makesure no overflow
  }  
}






// Method just for updating the set point.
void updateSetPoint(float serialSetPoint) 
{
  int setPointTemp;
  if (grblFlag == true) 
  {
    // Get set point from the pin controlled by gerbal. 
    // Should meausure a voltage on scale between 0 and 1023.
    //@todo uncomment this line
    pidSetPoint = analogRead(GRBL_DAT_PIN);
  }
  else 
  {
    // Look at Set Point from serial data. 
    //@todo i should map and constrain this value.
    pidSetPoint = serialSetPoint;
    //Serial.println("updated");
  }
}








/*
 * Method to Calibrate variable "zeroPSI".
 * Parameters: None
 * Return: Nothing



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
    Serial.println(current_settings.onOff);
    Serial.print("GRBL CONTROL:        ");
    Serial.println(grblFlag);
    Serial.println();
    
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
 * Format: 
 * Parameters:
 *    None.
 * Return:
 *    Nothing.
 */
void serialOutput() 
{
    /*
    send packet that contain the folowing information:
    - on/off
    - set point
    - current pressure
    - solenoid state
    */
   Serial.print(current_settings.onOff);
   Serial.print(",");
    Serial.print(current_settings.set_point);
    Serial.print(",");
    Serial.print(psiInput);
    for(int i=0; i<numOfSolenoids; i++){
      Serial.print(",");
      Serial.print(current_settings.solenoidState[i]);
    }
    Serial.print(",");
    Serial.println(grblPulseWidth);

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

void read_pressure(void){
  uint8_t rec_bytes =  Wire.requestFrom(0x08, 2);    // request 2 bytes from peripheral device #8
  if(rec_bytes == 2)
  {
      uint8_t hi= Wire.read();
      uint8_t lo= Wire.read();
      uint8_t status = hi>>6;
      hi &= 0x3f;
      uint16_t val = hi;
      val <<=8;
      val |= lo;
      if(status == 0 )
      {
         float pressure = (val - PRESSURE_MIN)*(PRESSURE_MAX - PRESSURE_MIN)/(OUTPUT_MAX - OUTPUT_MIN) + PRESSURE_MIN; 
         psiInput = map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0, 1023);
      }   
  }
}
