// #include <PID_v1.h>
// #include <math.h>
// #include <SharpIR.h>
//#include <DistanceGP2Y0A02YK.h>
//#include <DistanceGP2Y0A21YK.h>
#include <MazebusterIR.h>
//#include <SharpIR.h>
#define pinIR1 A0
//SharpIR sensorLR(pinIR1, 25, 93, 20150);
//DistanceGP2Y0A02YK sensorLR;
MazebusterIR sensorLR(pinIR1, 11, 25, 93);

void setup()
{
  Serial.begin (9600);
  pinMode(pinIR1, INPUT);
//  sensorLR.begin(pinIR1);
}

void loop(){
  double sum = 0;
//  double raw = analogRead(pinIR1);
//  for(int i=0; i<20; i++){
//    sum += sensorLR.getDistancemCentimeter2();
//    sum += sensorLR.distance();
//  }
//  double raw = 
  //double x = IR1.distance();
//  Serial.println(sensorLR.distance());
//  Serial.print(" ");
  Serial.println(analogRead(pinIR1));
  // Serial.println(x);
}
