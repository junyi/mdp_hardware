// #include <PID_v1.h>
// #include <math.h>
// #include <SharpIR.h>
//#include <DistanceGP2Y0A02YK.h>
#include <DistanceGP2Y0A21YK.h>
//#include <MazebusterIR.h>
//#include <SharpIR.h>
#define pinIR1 A4
//SharpIR sensorLR(pinIR1, 25, 93, 20150);
//DistanceGP2Y0A02YK sensorLR;
//MazebusterIR sensorLR(pinIR1, 11, 25, 93);
float smoothedVal = 0;
DistanceGP2Y0A21YK sensor(4);

void setup()
{
  Serial.begin (9600);
  pinMode(pinIR1, INPUT);
  sensor.begin(pinIR1);
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
  float data = analogRead(pinIR1);

//  double dist =3.06852324338388E-11*pow(data,5) - 5.84564605110099E-08*pow(data,4) + 0.0000426208835385612*pow(data,3) - 0.0145690712120406*pow(data,2) + 2.14262545695821*data - 40.871349341041;
//    double smoothedVal = smooth(dist, 0.5, smoothedVal);
//  double dist = -8.33656097582605E-10*pow(data, 4) + 1.70099164379982E-07*pow(data, 3) + 7.15664379621434E-04*pow(data, 2) - 5.35245811418573E-01*data + 1.40279263671649E+02;
//  smoothedVal = smooth(dist, 0.85, smoothedVal);
//  float dist = pow(data/4120.72436828283,1/-0.93133245362108);

  Serial.println(sensor.getDistanceCm());
//    Serial.println(data);
}

float smooth(float data, float filterVal, float smoothedVal) {
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}
