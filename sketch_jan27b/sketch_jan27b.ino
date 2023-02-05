
#define pressureIncrease  5  // Solenoid that lets air into system.
#define pressureDecrease  6
#define PSITransducer     A2

void setup()
{
  Serial.begin(9600);
  pinMode(pressureIncrease, OUTPUT);
  pinMode(pressureDecrease, OUTPUT);

  
}

void loop()
{
  int val = analogRead(PSITransducer);
  analogWrite(pressureIncrease, 205);
  //digitalWrite(pressureIncrease, LOW);
  analogWrite(pressureDecrease, 200);
  Serial.println(val);
  delay(10);
}
