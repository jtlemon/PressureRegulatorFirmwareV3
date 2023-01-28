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
#include <Arduino_AVRSTL.h>


#include <PID_v1.h>
#include "TimerOne.h"
#include "machine_interface.h"
#include <SoftPWM.h>


String received_serial_chars = "";



// I/O Pins for Pressure //
#define PSITransducer     A2  // Transducer to measure PSI in system.
#define pressureIncrease  A3  // Solenoid that lets air into system.
#define pressureDecrease  A4  // Solenoid that lets air out of system.


// PID Variables //
double psiInput;         // Analog reads from PSI transducer.
double psiInputInvert;   // An inverse of the psiInput based on the zero psi reading.
double psiIncrease;      // Output by PID for input  air PWM (0-255)
double psiDecrease;      // Output by PID for output air PWM (0-255)
double setPoint   = 0;   // Relative PSI, calculated amount
bool   grblFlag   = 0;   // Bool stating whether set point is being sent over serial (0) or via pin input from grbl (1).
int    grblDatPin = A7;  // Pin to get PWM wave from grbl to control set point.
int    grblSetPin = A5;  // Pin to control grblFlag.
// kp, ki, kd are to control the PID with desired aggressiveness\
// See https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID for a math heavy explanation, alternatively 
// See https://www.mathworks.com/discovery/pid-control.html for a more intuition based video explanation. 
double kp         = 3;   // How aggressivle the PID responsds to current amount of error.
double ki         = 5;   // How aggressivle the PID responsds to error over time. 
double kd         = .5;  // How aggressivle the PID responsds to rate of change of error.
int sampleTime    = 200; // How often a new output is calculated in milliseconds.
int outPutLimit   = 255; // PWM limit for PID output.

PID pidI(&psiInput, &psiIncrease, &setPoint, kp, ki, kd, P_ON_M, DIRECT);
PID pidD(&setPoint, &psiDecrease, &psiInput, kp, ki, kd, P_ON_M, DIRECT);
//PID pidD(&psiInputInverted, &psiDecrease, &setPoint, kp, ki, kd, DIRECT);


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

// I/O Pins for Vacuum Solenoids //

int solState[numOfSolenoids];  // Array to store state of solenoids.
#define solenoidOne   2
#define solenoidTwo   3
#define solenoidThree 4
#define solenoidFour  5
#define solenoidFive  6
#define solenoidSix   7
#define solenoidSeven 8
#define solenoidEight 9
#define solenoidNine  10
#define solenoidTen   11


// Serial Data Variables // 
const byte numChar = 255;       // Number of bytes expected to handle at a time.
char incomingChar[numChar];     // Array to store incoming data.
char tempIncChar[numChar];      // Array to copy incoming data into. 
// int  timeOut       = 60000;     // Time (milliseconds) to wait for serial data before automatically setting machine state to off. 
unsigned long timeReference;
// Array for data regarding solenoids given above.
// The PSI read in will be set to be the set point for the PID.


// Foot Pedal Variables and I/O //
bool inputMachineState     = false;
volatile bool machineState = false;     // System on or off.
const int footPedalRight   = A0;  
const int footPedalLeft    = A1;
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
  {footPedalLeft,  0, 0, 0, false, false},  // Foot pedal on left.
  {footPedalRight, 0, 0, 0, false, false},  // Foot pedal on right.
};

// Vector of current machine values - used in being able to update only portions of current settings. 


machine_setting_t current_settings = {
  false, 0, {false, false, false, false, false, false, false, false, false, false}, kp, ki, kd, accuracy, sampleTime
};

/*
// Timing //
unsigned long timerTime    = 0;
unsigned long currentTime  = 0;
unsigned long previousTime = 0;
int timerDelay = 750;
*/

#define led 13
bool ledState = false;
bool testingState = false;
int  printInterval = 60;

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////     SET UP     /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Begin Serial Communication
  init_communication();

  // Set up PID
  pidI.SetMode(AUTOMATIC);
  pidI.SetSampleTime(sampleTime);
  pidD.SetMode(AUTOMATIC);
  pidD.SetSampleTime(sampleTime);

  // Establish I/O
  pinMode(grblDatPin,       INPUT);
  pinMode(grblSetPin,       INPUT_PULLUP);
  pinMode(PSITransducer,    INPUT);
  pinMode(footPedalLeft,    INPUT_PULLUP);
  pinMode(footPedalRight,   INPUT_PULLUP);
  pinMode(solenoidOne,      OUTPUT);
  pinMode(solenoidTwo,      OUTPUT);
  pinMode(solenoidThree,    OUTPUT);
  pinMode(solenoidFour,     OUTPUT);
  pinMode(solenoidFive,     OUTPUT);
  pinMode(solenoidSix,      OUTPUT);
  pinMode(solenoidSeven,    OUTPUT);
  pinMode(solenoidEight,    OUTPUT);
  pinMode(solenoidNine,     OUTPUT);
  pinMode(solenoidTen,      OUTPUT);

  pinMode(pressureIncrease, OUTPUT);
  pinMode(pressureDecrease, OUTPUT);

  SoftPWMBegin();
  SoftPWMSet(pressureIncrease, 0);
  SoftPWMSet(pressureDecrease, 0);


  
  // Ensure solenoids off - Machine State == false
  updateSol();

  // Establish Timer and Create Interrupt Every 6ms //
  Timer1.initialize(interruptTime*1000);
  Timer1.attachInterrupt(timerIsr);
    // Calibrate
  //calibrateZero();
  //calibrateMax();
  footPedals[0].footCurrentState = false;
  footPedals[1].footCurrentState = false;

  // Time out timer
  timeReference = millis();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////      LOOP      /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // Time out timer check // this was breaking the code and not letting the machine state activate for some reason, inconsistent between devices. 
  /*
  if (millis() - timeReference > timeOut) 
  {
    settingsVector[0] = "0";
    valueUpdate(settingsVector);
  }
  */
  
  // See where set point data should be coming from.
  if (digitalRead(grblSetPin)!= HIGH) // Pullup
  {
    grblFlag = true;
    current_settings.onOff = true;
    valueUpdate();
  }
  else
  {
    // Until timing is figured out, when the spindal output is disengaged, disengage all vacuum solenoinds. 
    if (grblFlag == true)
    {
      for (int i=0; i<numOfSolenoids; i++)
      {
        current_settings.solenoidState[i] = false;
      }
      valueUpdate();
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
    timeReference = millis();
    //noInterrupts();
    //ledState = !ledState;
    //digitalWrite(led, ledState);
    get_received_data(current_settings);
    valueUpdate();
      
      // For Testing purposes - prints recieved packet information.
      /*
      if (testingState)
      {
        Serial.println("packet received ............");
        for(uint8_t i=0; i<temp_vector.size();  i++)
        {
          Serial.print("the received ele at index ");
          Serial.print(i);
          Serial.print(":");
          Serial.println(atof(temp_vector[i].c_str()));
        }
      }
      */
      
    //interrupts();
  } 
  else 
  {
    updateSetPoint(current_settings.set_point);
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
  
  // Recieve Incoming Data //
  //Timer1.stop();
  //Timer1.resume(); 
  
  // Compute PID and Alter Valves //
  psiInput = analogRead(PSITransducer);
//  psiInputInverted = 
  pidI.Compute();
  pidD.Compute();
  //Serial.print("Output Right after compute");
  //Serial.println(psiOutput);

  // Update Machine //
  machineStateUpdate();
  
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

/*
 * Method to iterate through input serial data and make value updates as assigned here. 
 * <machineState, setPoint, sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8, sol9, sol10, kp, ki, kd, sampleTime>
 * Strings Representing: $bool,float,bool,bool,bool,bool,bool,bool,bool,bool,bool,bool,float,float,float,int,int*
 * Parameters: 
 *        Vector to parse.
 * Return: 
 *        Nothing, though update system values. 
 */
void valueUpdate() {
  Serial.println("let's update the values");
  // Machine State
  inputMachineState = current_settings.onOff;
  
  updateSetPoint(current_settings.set_point);

  // Solenoids
  for(uint8_t i=0; i<numOfSolenoids;  i++)
  {
    solState[i-2] = current_settings.solenoidState[i];
  }

  // kp/ki/kd
  kp = current_settings.kp_value;
  ki = current_settings.ki_value;
  kd = current_settings.kd_value;

  // Center Margin
  accuracy = current_settings.accuracy_value;
  // Sample Time
  sampleTime = current_settings.sample_time_value_ms;
  // Time Out Time
  // timeOut = atoi(temp_vector[17].c_str());
  Serial.println("finished updating values");
}


// Method just for updating the set point.
void updateSetPoint(float serialSetPoint) 
{
  int setPointTemp;
  if (grblFlag == true) 
  {
    // Get set point from the pin controlled by gerbal. 
    // Should meausure a voltage on scale between 0 and 1023.
    setPoint = analogRead(grblDatPin);
  }
  else 
  {
    // Look at Set Point from serial data. 
    setPointTemp = serialSetPoint;
    
    // Check given PSI is within range before setting (if not in range, PSI does not change).
    if (setPointTemp <= maxPSI) 
    {
       // Calculate setPoint in Steps
       setPoint = (setPointTemp*multFactor)+zeroPSI;
    }  
  }
}

// Method to update the machine. The internal code can force the 
// machine into the off state, otherwise the machine state is that 
// of the last given serial input. Then updates all of the solenoids.
void machineStateUpdate() {
  machineState = inputMachineState;
  valveUpdate();
  updateSol();
}

/*
 * Method to update state of pressure control valves.
 * Parameters:
 *    None. 
 * Return:
 *    Nothing.
 */
void valveUpdate() {
  // Machine in Off State //
  if (machineState == false) {
    if (NO == true) {
      // LOCKS GATES IN OFF STATE...
      digitalWrite(pressureDecrease, HIGH);
      digitalWrite(pressureIncrease, HIGH);
    } else {
      digitalWrite(pressureDecrease, LOW);
      digitalWrite(pressureIncrease, LOW);
    }
  } else {

    SoftPWMSet(pressureIncrease, psiIncrease);
    SoftPWMSet(pressureDecrease, psiDecrease);
  }
}

/*
 * Method to establish current state of 24V solenoids.
 * Parameters:
 *    None.
 * Return:
 *    Nothing. 
 */
void updateSol() {
  if(machineState == false) {
    digitalWrite(solenoidOne,   LOW);
    digitalWrite(solenoidTwo,   LOW);
    digitalWrite(solenoidThree, LOW);
    digitalWrite(solenoidFour,  LOW);
    digitalWrite(solenoidFive,  LOW);
    digitalWrite(solenoidSix,   LOW);
    digitalWrite(solenoidSeven, LOW);
    digitalWrite(solenoidEight, LOW);
    digitalWrite(solenoidNine,  LOW);
    digitalWrite(solenoidTen,   LOW);
  } else {
    // Solenoid One
    if (solState[0] == 1) {
      digitalWrite(solenoidOne, HIGH);
    } else {
      digitalWrite(solenoidOne, LOW);
    }
    
    // Solenoid Two
    if (solState[1] == 1) {
      digitalWrite(solenoidTwo, HIGH);
    } else {
      digitalWrite(solenoidTwo, LOW);
    }
    
    // Solenoid Three
    if (solState[2] == 1) {
      digitalWrite(solenoidThree, HIGH);
    } else {
      digitalWrite(solenoidThree, LOW);
    }
    
    // Solenoid Four
    if (solState[3] == 1) {
      digitalWrite(solenoidFour, HIGH);
    } else {
      digitalWrite(solenoidFour, LOW);
    }

    // Solenoid Five
    if (solState[4] == 1) {
      digitalWrite(solenoidFive, HIGH);
    } else {
      digitalWrite(solenoidFive, LOW);
    }
    
    // Solenoid Six
    if (solState[5] == 1) {
      digitalWrite(solenoidSix, HIGH);
    } else {
      digitalWrite(solenoidSix, LOW);
    }

    // Solenoid Seven
    if (solState[6] == 1) {
      digitalWrite(solenoidSeven, HIGH);
    } else {
      digitalWrite(solenoidSeven, LOW);
    }
    
    // Solenoid Eight
    if (solState[7] == 1) {
      digitalWrite(solenoidEight, HIGH);
    } else {
      digitalWrite(solenoidEight, LOW);
    }

    // Solenoid Nine
    if (solState[8] == 1) {
      digitalWrite(solenoidNine, HIGH);
    } else {
      digitalWrite(solenoidNine, LOW);
    }
    
    // Solenoid Ten
    if (solState[9] == 1) {
      digitalWrite(solenoidTen, HIGH);
    } else {
      digitalWrite(solenoidTen, LOW);
    }
  }
  return;
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
    digitalWrite(pressureDecrease, LOW);
    digitalWrite(pressureIncrease, HIGH);
  } else {
    digitalWrite(pressureDecrease, HIGH);
    digitalWrite(pressureIncrease, LOW);
  }

  // Wait for pressure to be released.
  delay(calibrateTime);

  // Set the zeroPSI variable to equal whatever the transducer reads when no pressure in the system. 
  zeroPSI = analogRead(PSITransducer);

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
  psiInput = analogRead(PSITransducer);
  
  // Fill line up with air, stop at either max air system can handle, or max reading available from transducer.
  while((psiInput < maxSystemStep) && (psiInput < 1023)) {

    // Read pressure
    psiInput = analogRead(PSITransducer);
    
    if (NO == true) {
      digitalWrite(pressureDecrease, HIGH);
      digitalWrite(pressureIncrease, LOW);
    } else {
      digitalWrite(pressureDecrease, LOW);
      digitalWrite(pressureIncrease, HIGH);
    }
  }

  // Record pressure when at max. 
  maxPSI = analogRead(PSITransducer);

  // Start letting air out of system.
  if (NO == true) {
    digitalWrite(pressureDecrease, LOW);
    digitalWrite(pressureIncrease, HIGH);
  } else {
    digitalWrite(pressureDecrease, HIGH);
    digitalWrite(pressureIncrease, LOW);
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
    
    Serial.print("Solenoid States:     [");
    for(int j=0; j<(numOfSolenoids-1); j++) {
      Serial.print(solState[j]);
      Serial.print(",");
    }
    Serial.print(solState[numOfSolenoids-1]);
    Serial.println("]");
    
    Serial.print("Zero PSI:            ");
    Serial.println(zeroPSI);
    Serial.print("Max PSI (steps):     ");
    Serial.println(maxPSI);
    Serial.print("kp:                  ");
    Serial.println(kp);
    Serial.print("ki:                  ");
    Serial.println(ki);
    Serial.print("kd:                  ");
    Serial.println(kd);
    Serial.print("Sample Time:         ");
    Serial.println(sampleTime);   
    Serial.print("Set Point:           ");
    Serial.println(setPoint);   
    Serial.print("Transducer Input:    ");
    Serial.println(psiInput);
    Serial.print("PSI Increase Step:   ");
    Serial.println(psiIncrease);
    Serial.print("PSI Decrease Step:   ");
    Serial.println(psiDecrease);
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
  return;
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


/*
 * Method to print an array of char data types. 
 * Parameters: 
 *    Array of Char data values to be printed.
 * Return:
 *    Nothing.
 */
void printCharArray(char arr[]) {
  Serial.println();
  for(int i=0; i<(sizeof(arr)-1); i++) {
    Serial.print(arr[i]);
    Serial.print(", ");
  }
  Serial.print(arr[sizeof(arr)-1]);
  Serial.println();
  return;
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
