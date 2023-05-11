#include <PID_v1.h>
#include "TimerOne.h"

#define pressureIncrease  5  // Solenoid that lets air into system.
#define pressureDecrease  6
#define PSITransducer     A2
double hysteresis_offset = 10;
double kp         = 20;   // How aggressivle the PID responsds to current amount of error.
double ki         = 0;   // How aggressivle the PID responsds to error over time. 
double kd         = 0;  // How aggressivle the PID responsds to rate of change of error.
int sampleTime    = 5; // How often a new output is calculated in milliseconds.
int outPutLimit   = 255; // PWM limit for PID output.

doublee psiInput;
double pidSetPoint   = 200;   // Relative PSI, calculated amount
double psiIncrease;      // Output by PID for input  air PWM (0-255)
double psiDecrease;
double up = 0;
double down = 0;
double step = 20;
char current_dir = 's';
PID pidI(&psiInput, &psiIncrease, &setPoint, kp, ki, kd, P_ON_M, DIRECT);
PID pidD(&pidSetPoint, &psiDecrease, &psiInput, kp, ki, kd, P_ON_M, DIRECT);

void change(char dir){
  current_dir = dir;
  if(dir == 'u'){
    up += step;
    down -=step;
  }else{
     up -= step;
    down +=step;
  }
  if(up > 255){
    up = 255;
  }
  else if(up<0) up = 0;
  if(down > 255){
    down = 255;
  }
  else if(down<0) down = 0;
}
void setup()
{
  Serial.begin(115200);
  TCCR0B = TCCR0B & B11111010 ;
  pidI.SetMode(AUTOMATIC);
  pidI.SetSampleTime(sampleTime);
  pidD.SetMode(AUTOMATIC);
  pidD.SetSampleTime(sampleTime);
  pinMode(pressureIncrease, OUTPUT);
  pinMode(pressureDecrease, OUTPUT);
  pinMode(PSITransducer, INPUT);
  Timer1.initialize(sampleTime*1000);
  Timer1.attachInterrupt(timerIsr);
  
  //timeReference = millis();
}

void loop()
{
    noInterrupts();
    Serial.print(psiInput);
    Serial.print(" ");
    Serial.print(pidSetPoint);
    Serial.print(" ");
    Serial.print(up);
    Serial.print(" ");
    Serial.println(down);
    interrupts();
    delay(50);
}

void timerIsr() {
  psiInput = analogRead(PSITransducer); // psi 
//  psiInputInverted = 
  ///pidI.Compute();
  //pidD.Compute();
  // 1 0 max 
  if(psiDecrease > 255) psiDecrease = 255;
  else if(psiDecrease < 0) psiDecrease = 0;
  if(psiIncrease > 255) psiIncrease = 255;
  else if(psiIncrease < 0) psiIncrease = 0;
  char dir = current_dir;
  if(psiInput > pidSetPoint + hysteresis_offset)
  {
     dir = 'd';
     step = 10;
     down=255;
  }
  else if (psiInput < pidSetPoint - hysteresis_offset)
  {
      dir = 'u';
      step = 10;
      up=255;
  }
  else{
     step = abs(pidSetPoint-psiInput) * 0.7;
  }
  change(dir);
  analogWrite(pressureIncrease, up);
   analogWrite(pressureDecrease, down);
  
}