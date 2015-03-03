#include <MazebusterIR.h>

#define SENSOR_IR_FRONT_LEFT A0
#define SENSOR_IR_FRONT_RIGHT A1
#define SENSOR_IR_LEFT A2
#define SENSOR_IR_RIGHT A3

MazebusterIR irFrontLeft(SENSOR_IR_FRONT_LEFT, 11, 25, 93);
MazebusterIR irFrontRight(SENSOR_IR_FRONT_RIGHT, 12, 25, 93);
MazebusterIR irLeft(SENSOR_IR_LEFT, 3, 25, 93);
MazebusterIR irRight(SENSOR_IR_RIGHT, 1, 25, 93);

double sensorReadings[4] = {};

void setup(){
  Serial.begin(9600);

  pinMode(SENSOR_IR_FRONT_LEFT, INPUT);
  //pinMode(SENSOR_IR_FRONT_RIGHT, INPUT);
  //pinMode(SENSOR_IR_LEFT, INPUT);
  //pinMode(SENSOR_IR_RIGHT, INPUT);
}

void loop(){
  readAllSensors();
  
  for(int i=0; i<1; i++){
    Serial.print(sensorReadings[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void readAllSensors(){
  sensorReadings[0] = irFrontLeft.distance();
  //sensorReadings[1] = irFrontRight.distance();
  //sensorReadings[2] = irLeft.distance();
  //sensorReadings[3] = irRight.distance();
}
