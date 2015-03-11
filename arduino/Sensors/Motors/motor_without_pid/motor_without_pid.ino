#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <DistanceGP2Y0A21YK.h>
#include <DistanceGP2Y0A02YK.h>
#include <PID_AutoTune_v0.h>

#define MOTOR_L_ENCODER_A 3
#define MOTOR_L_ENCODER_B 5

#define MOTOR_R_ENCODER_A 11
#define MOTOR_R_ENCODER_B 13

#define SENSOR_IR_FRONT_LEFT A1
#define SENSOR_IR_FRONT_MIDDLE A0
#define SENSOR_IR_FRONT_RIGHT A2
#define SENSOR_IR_LEFT A3
#define SENSOR_IR_RIGHT A4

DualVNH5019MotorShield md; 

DistanceGP2Y0A21YK frontMiddle(0);
DistanceGP2Y0A21YK frontLeft(1);
DistanceGP2Y0A21YK frontRight(2);
DistanceGP2Y0A21YK left(3);
DistanceGP2Y0A21YK right(4);

int  motorLPrevAccmEncoderCount = 0;  //Previous accumulated encoder's ticks count of left motor
double  motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
int  motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
double  motorLDistInitPt = 0;            //Initial check point of left motor's encoder's ticks
double  motorLDistChkPt = 0;             //Final distance/encoder check point of left motor

int  motorRPrevAccmEncoderCount = 0;  //Previous accumulated encoder's ticks count of right motor
double  motorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
int  motorRNetEncoderCount = 0;       //Net encoder's ticks count of right motor
double  motorRDistInitPt = 0;            //Initial check point of right motor's encoder's ticks
double  motorRDistChkPt = 0;             //Final distance/encoder check point of right motor

double  motorLSpeed = 0;                 //Rotation speed of left motor
double  targetLSpeed = 0;                //Target speed of left motor
double  motorLPWM = 0;                   //PWM (0 - 255) of left motor

double  motorRSpeed = 0;                 //Rotation speed of right motor
double  targetRSpeed = 0;                //Target speed of right motor
double  motorRPWM = 0;                   //PWM (0 - 255) of right motor

double  motorAccmEncoderCount = 0;       //Average accumulated encoder's ticks count of both motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
double  targetMotorSpeed = 0;            //Target speed should be achieved by both motors

unsigned long prevTime = 0;              //Previous timestamp checked for motors' speeds
unsigned long currTime = 0;              //Current timestamp

PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 300, 0, 0, DIRECT);  //PID that controls the PWM of left motor
PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 300, 0, 0, DIRECT);  //PID that controls the PWM of right motor

const byte MODE_STOP = B00;
const byte MODE_EXPLORE = B01;
const byte MODE_SHORTESTPATH = B10;
const byte MODE_CALIBRATE = B11;

/************** Decoded Command **************/
byte    mode = MODE_STOP;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    dir = B11;          //Forward = 11, Left = 01, Right = 10, Backward = 00
int     travelDist = 260;     //Distance (in ticks count) required to be travelled by the motor
const   int DEGREE_90 = 290;          //90 degreee ange=le
/******************** END ********************/

/******************* Flags *******************/
boolean sense = true;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
/******************** END ********************/

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int lspeed = 0;
int rspeed = 0;
int sensorReadings[5] = {};

int LMag = 1;
int RMag = -1;

void setup(){
  Serial.begin(9600);
  
  inputString.reserve(20);
    
  pinMode(MOTOR_L_ENCODER_A, INPUT);
  pinMode(MOTOR_L_ENCODER_B, INPUT);
  pinMode(MOTOR_R_ENCODER_A, INPUT);
  pinMode(MOTOR_R_ENCODER_B, INPUT);
  
  frontMiddle.begin(SENSOR_IR_FRONT_MIDDLE);
  frontLeft.begin(SENSOR_IR_FRONT_LEFT);
  frontRight.begin(SENSOR_IR_FRONT_RIGHT);
  left.begin(SENSOR_IR_LEFT);
  right.begin(SENSOR_IR_RIGHT);
  
  /********** PID  Configurations **********/
  motorLPID.SetMode(AUTOMATIC);
  motorRPID.SetMode(AUTOMATIC);
  motorLPID.SetOutputLimits(98.9, 255);
  motorRPID.SetOutputLimits(100, 255);
  motorLPID.SetSampleTime(50);
  motorRPID.SetSampleTime(50);
  /****************** END ******************/
  
  motorLPID.SetTunings(300, 0, 0);
  motorRPID.SetTunings(300, 0, 0);
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, RISING);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, RISING);  //Attach right motor encoder interrupt pin to the ISR
  
}

void loop(){
  configMove();
  controlRobot();
  
  if(dir == B11)
    if(motorLRun || motorRRun)
      computePID();
  
  robotMove();
  
  // Do not sense if robot is moving
  if(sense && !motorLRun && !motorRRun){
//    delay(100);
    for(int i=0; i<10; i++) {
      readAllSensors();
    }
  
//    for(int i=0; i<5; i++){
//      Serial.print(sensorReadings[i]);
//      Serial.print(" ");
//    }
//    Serial.println();

    sense = false;
  }
  Serial.print(currTime);
  Serial.print(" ");
  Serial.print(motorLPWM);
  Serial.print(" ");
  Serial.print(motorRPWM);
  Serial.print(" ");
  Serial.print(motorLAccmEncoderCount);
  Serial.print(" ");
  Serial.print(motorRAccmEncoderCount);
  Serial.print(" ");
  Serial.print(targetLSpeed);
  Serial.print(" ");
  Serial.print(targetRSpeed);
  Serial.print(" ");
  Serial.print(motorLSpeed);
  Serial.print(" ");
  Serial.println(motorRSpeed);
//  
//  Serial.println(motorLAccmEncoderCount);

}

void readAllSensors() {  
  sensorReadings[0] = frontLeft.getDistanceMedian() - 6;
  sensorReadings[1] = frontMiddle.getDistanceMedian() - 6;
  sensorReadings[2] = frontRight.getDistanceMedian() - 6;
  sensorReadings[3] = right.getDistanceMedian() - 6;
  sensorReadings[4] = left.getDistanceMedian() - 6;
}

void robotMove(){
  if(mode == MODE_STOP) {
    robotStop();
    return;
  }
  
  if(motorLAccmEncoderCount < motorDistChkPt){
    md.setM2Speed(LMag*motorLPWM/255.0*400.0 );
  }else {
    robotStop();
  }
  
  if(motorRAccmEncoderCount < motorDistChkPt){
    md.setM1Speed(RMag*motorRPWM/255.0*400.0);
  }else {
    robotStop();
  }
}

void robotStop(){
  md.setM1Speed(0);
  md.setM2Speed(0);
  targetLSpeed = 0;
  targetRSpeed = 0;
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
  mode = MODE_STOP;
}

void serialEvent() {  //Read inputs sent from Raspberry Pi via USB serial communication
  byte command;
  Serial.flush();
  
  //When there's a data in the receiveing buffer, and both the motors have completed their moves
  if(Serial.available() && !motorLRun && !motorRRun) {
    int data1 = Serial.read();          //First byte of data is moving command
    int data2 = Serial.read();          //Second byte of data is distance
    Serial.flush();
    
    if(data1 != -1) {                   //If received a command, decode the command
      command = byte(data1);
      mode = (command & 0x0C) >> 2;         //Bit 0-1 represents start or stop
//      Serial.print("Mode ");
//      Serial.println(mode);
      dir = (command & 0x03);    //Bits 2-3 represent direction to move
    }
    
    if(data2 != -1 && dir == B11) {
      command = byte(data2);
//      Serial.print("Dir ");
//      Serial.println(dir);
      travelDist = int(command & 0x0F) * 260;  //Bits 3-6 represent number of grids required to move (1 grid = 10cm)
//      Serial.print("Dist ");
//      Serial.println(travelDist);


    }
    sense = false;
    configMove();                     //Configure variables for the next move
  }
}

void configMove(){
  if(mode == MODE_STOP) {
    robotStop();
    return;
  }
  
  if(!motorLRun && !motorRRun){
    
    motorLAccmEncoderCount = 0;
    motorRAccmEncoderCount = 0;
    motorLPrevAccmEncoderCount = 0;
    motorRPrevAccmEncoderCount = 0;
    motorLRun = true;
    motorRRun = true;
    sense = true;
    
    switch(dir){
      case B11:
        targetMotorSpeed = 0.5;
        LMag = -1;
        RMag = 1;
        motorDistChkPt = 250;
        break;
      case B01:
        targetMotorSpeed = 0.35;
        LMag = -1;
        RMag = -1;
        motorDistChkPt = 284;
        break;
      case B10:
        targetMotorSpeed = 0.35;
        LMag = 1;
        RMag = 1;
        motorDistChkPt = 283;
        break;
      case B00:
        targetMotorSpeed = 0.5;
        LMag = -1;
        RMag = -1;
        motorDistChkPt = 677;
    }
    
    if(dir != B11){
      motorLPWM = 160;
      motorRPWM = 160;
    }
    
//    delay(1000);
  }
}


void computePID() {         //Adjust PID of both motors to make the robot moves in straight line
  if(motorAccmEncoderCount < 150){
    if(targetLSpeed < targetMotorSpeed){
      targetLSpeed += targetMotorSpeed/4.0;
    }
    
    if(targetRSpeed < targetMotorSpeed){
      targetRSpeed += targetMotorSpeed/4.0;
    }
  }
  
  if(motorAccmEncoderCount > travelDist - 150){
    if(targetLSpeed > 0){
      targetLSpeed -= targetMotorSpeed/4.0;
    }
    
    if(targetRSpeed > 0){
      targetRSpeed -= targetMotorSpeed/4.0;
    }
  }

  motorLPID.Compute();  //Compute right motor PWM based on the right motor speed w.r.t the target speed
  motorRPID.Compute();  //Compute left motor PWM based on the left motor speed w.r.t the target speed
  
}

void controlRobot() {       //Control the motors based on the command and PWM values given
  
  currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.

  motorAccmEncoderCount = (motorLAccmEncoderCount + motorRAccmEncoderCount) / 2;
  motorLSpeed = abs((double)(motorLAccmEncoderCount - motorLPrevAccmEncoderCount)) / (currTime - prevTime);
  motorRSpeed = abs((double)(motorRAccmEncoderCount - motorRPrevAccmEncoderCount)) / (currTime - prevTime);
  prevTime = currTime;
  motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
  motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
  
  
  delay(50);
}

void motorLISR() {    //ISR for left motor encoder interrupt
  motorLAccmEncoderCount++;
  
  if(digitalRead(MOTOR_L_ENCODER_A) == HIGH) {  //Low-to-high edge on channel A
    //Check channel B to see which way the motor is turning
    if (digitalRead(MOTOR_L_ENCODER_B) == LOW) {
      motorLNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorLNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
  else {  //High-to-low edge on channel A
    //Check channel B to see which way encoder is turning
    if (digitalRead(MOTOR_L_ENCODER_B) == HIGH) {
      motorLNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorLNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
}

void motorRISR() {    //ISR for right motor encoder interrupt
  motorRAccmEncoderCount++;
  
  if(digitalRead(MOTOR_R_ENCODER_A) == HIGH) {  //Low-to-high edge on channel A
    //Check channel B to see which way the motor is turning
    if (digitalRead(MOTOR_R_ENCODER_B) == HIGH) {
      motorRNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorRNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
  else {  //High-to-low edge on channel A
    //Check channel B to see which way encoder is turning
    if (digitalRead(MOTOR_R_ENCODER_B) == LOW) {
      motorRNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorRNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
}

