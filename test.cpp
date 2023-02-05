#include <PID_v1.h>
#include "TimerOne.h"

#define pressureIncrease  6  // Solenoid that lets air into system.
#define pressureDecrease  5
#define PSITransducer     A2

double kp         = 20;   // How aggressivle the PID responsds to current amount of error.

int sampleTime    = 25; // How often a new output is calculated in milliseconds.
int outPutLimit   = 255; // PWM limit for PID output.

double psiInput;
double setPoint   = 300;   // Relative PSI, calculated amount
double psiIncrease;      // Output by PID for input  air PWM (0-255)
double psiDecrease;

PID pidI(&psiInput, &psiIncrease, &setPoint, kp, ki, kd, P_ON_M, DIRECT);
PID pidD(&setPoint, &psiDecrease, &psiInput, kp, ki, kd, P_ON_M, DIRECT);

void setup()
{
  Serial.begin(9600);
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
    Serial.print(psiIncrease);
    Serial.print(" ");
    Serial.print(psiDecrease);
    Serial.print(" ");
    Serial.println(setPoint);
    interrupts();
    delay(50);
}

void timerIsr() {
  psiInput = analogRead(PSITransducer); // psi 
//  psiInputInverted = 
  pidI.Compute();
  pidD.Compute();
  // 1 0 max 
  if(psiDecrease > 255) psiDecrease = 255;
  else if(psiDecrease < 0) psiDecrease = 0;
  if(psiIncrease > 255) psiIncrease = 255;
  else if(psiIncrease < 0) psiIncrease = 0;
  analogWrite(pressureIncrease, psiIncrease);
  analogWrite(pressureDecrease, psiDecrease);
}