// #include <PID_v1.h>
// #include <math.h>
// #include <SharpIR.h>
#include <MazebusterIR.h>
#define pinIR1 A0
MazebusterIR IR1(pinIR1, 1, 25, 93);

void setup()
{
  Serial.begin (9600);
  pinMode(pinIR1, INPUT);
}

void loop(){
  double raw = analogRead(pinIR1);
  double x = IR1.distance();
  Serial.println(x);
  // Serial.print(" ");
  // Serial.println(x);
}
