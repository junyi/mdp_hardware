#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <DistanceGP2Y0A21YK.h>
#include <digitalWriteFast.h>

#define MOTOR_L_ENCODER_A 3
#define MOTOR_L_ENCODER_B 5

#define MOTOR_R_ENCODER_A 11
#define MOTOR_R_ENCODER_B 13

#define SENSOR_IR_FRONT_MIDDLE A0
#define SENSOR_IR_FRONT_LEFT   A1
#define SENSOR_IR_FRONT_RIGHT  A2
#define SENSOR_IR_LEFT_MIDDLE  A3
#define SENSOR_IR_RIGHT_MIDDLE A4
#define SENSOR_IR_RIGHT_TOP    A5

#define FORWARD B11
#define CW      B10
#define CCW     B01
#define BACK    B00

#define FRONT 0
//#define LEFT  2
#define RIGHT 4

#define DELAY_PERIOD 100

DualVNH5019MotorShield md; 

DistanceGP2Y0A21YK frontMiddle(0);
DistanceGP2Y0A21YK frontLeft(1);
DistanceGP2Y0A21YK frontRight(2);
DistanceGP2Y0A21YK leftMiddle(3);
DistanceGP2Y0A21YK rightMiddle(4);
DistanceGP2Y0A21YK rightTop(5);

int  motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
int  motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
int  motorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
int  motorRNetEncoderCount = 0;       //Net encoder's ticks count of right motor

double  motorLPWM = 0;                   //PWM (0 - 255) of left motor
double  motorRPWM = 0;                   //PWM (0 - 255) of right motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors

const byte MODE_STOP = B00;
const byte MODE_EXPLORE = B01;
const byte MODE_SHORTESTPATH = B10;
const byte MODE_CALIBRATE = B11;

/************** Decoded Command **************/
byte    mode = MODE_STOP;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    dir = FORWARD;          //Forward = 11, Left = 01, Right = 10, Backward = 00
/******************** END ********************/

/******************* Flags *******************/
boolean sense = false;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
boolean hasSent = false;
volatile boolean hasCalibrated = false;
volatile boolean start = false;
/******************** END ********************/

double FORWARD_PWM_L = 166;
double FORWARD_PWM_R = 167;
int FORWARD_DIST = 482;

double CCW_PWM_L = 200;
double CCW_PWM_R = 189;
int CCW_DIST = 783; // 783
double CW_PWM_L = 180;
double CW_PWM_R = 180;
int CW_DIST = 760; // 706
double BACK_PWM_L = 180;
double BACK_PWM_R = 180;
int BACK_DIST = 1610;

float RIGHT_TOP_OFFSET = 12; 

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int lspeed = 0;
int rspeed = 0;
float sensorReadings[6] = {};
float previousLeftReading = 0;
float previousRightReading = 0;

int LMag = 1;
int RMag = -1;

void setup(){
  Serial.begin(9600);
  
  // inputString.reserve(20);
    
  pinMode(MOTOR_L_ENCODER_A, INPUT);
  pinMode(MOTOR_L_ENCODER_B, INPUT);
  pinMode(MOTOR_R_ENCODER_A, INPUT);
  pinMode(MOTOR_R_ENCODER_B, INPUT);

  frontLeft.begin(SENSOR_IR_FRONT_LEFT);
  frontRight.begin(SENSOR_IR_FRONT_RIGHT);
  frontMiddle.begin(SENSOR_IR_FRONT_MIDDLE);
  leftMiddle.begin(SENSOR_IR_LEFT_MIDDLE);
  rightMiddle.begin(SENSOR_IR_RIGHT_MIDDLE);
  rightTop.begin(SENSOR_IR_RIGHT_TOP);
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, CHANGE);  //Attach right motor encoder interrupt pin to the ISR
  
}

void loop(){
  if(!start){
    return;
  }
    
  // if(sense && !motorLRun && !motorRRun){

  //   readAllSensors();
      
  //   for(int i=0; i<5; i++){
  //     Serial.print(sensorReadings[i]);
  //     Serial.print(" ");
  //   }
  //   Serial.println();

  //   sense = false;
  // }else{
  //   if(mode == MODE_EXPLORE){
  //     configMove();
    
  //     robotMove();
  //   }else if(mode == MODE_CALIBRATE){
  //     calibrateFront();
  //   }
  // }
  
  if(!hasSent && !motorLRun && !motorRRun){
    Serial.println("*");
    hasSent = true;
  }
}

void readAllSensors() {
  int l = mode == MODE_CALIBRATE ? 50 : 50;
  for(int i=0; i<l; i++) {
    sensorReadings[0] = frontLeft.getDistanceMedian();
    sensorReadings[1] = frontMiddle.getDistanceMedian();
    sensorReadings[2] = frontRight.getDistanceMedian();
    sensorReadings[3] = leftMiddle.getDistanceMedian();
    sensorReadings[4] = rightTop.getDistanceMedian();
    sensorReadings[5] = rightMiddle.getDistanceMedian();
  }
}

void serialEvent() {  //Read inputs sent from Raspberry Pi via USB serial communication
  byte command;
  Serial.flush();
  
  //When there's a data in the receiveing buffer, and both the motors have completed their moves
  if(Serial.available() && !motorLRun && !motorRRun) {
    int data1 = Serial.read();          //First byte of data is moving command
    int data2 = Serial.read();          //Second byte of data is distance
    
    //Initialization for robot
    if(((char) data1) == '*'){
      start = true;
      return;
    }
    
    Serial.flush();
    
    if(data1 != -1) {                   //If received a command, decode the command
      command = byte(data1);
      sense = (boolean) ((data1 & 0x40) >> 6) & 1;
      mode = (command & 0x0C) >> 2;         //Bit 0-1 represents start or stop
      dir = (command & 0x03);    //Bits 2-3 represent direction to move
    }
    
    if(data2 != -1 && dir == B11) {
      command = byte(data2);
      motorDistChkPt = int(command & 0x0F) * FORWARD_DIST;  //Bits 3-6 represent number of grids required to move (1 grid = 10cm)
    }
  }

  replyWithSensorData();
  if(mode == MODE_EXPLORE){
    parseMove();
  }else if(mode == MODE_CALIBRATE){
//    calibrateDistance();
    calibrateRotation(RIGHT);
//    calibrateDistance();
//    calibrateRotation(FRONT);
  }
}

void replyWithSensorData(){
  if(sense && !motorLRun && !motorRRun){

    readAllSensors();
      
    for(int i=0; i<6; i++){
      Serial.print(sensorReadings[i]);
      Serial.print(" ");
    }
    Serial.println();

    sense = false;
  }
}

void parseMove(){
  if(!motorLRun && !motorRRun){
    resetMove();
    hasSent = false;
    sense = true;
    
    switch(dir){
      case FORWARD:
        straight(10);
//        delay(DELAY_PERIOD);
//        calibrateSide();
        break;
      case CCW:
        rotate(90, CCW);
        break;
      case CW:
        rotate(90, CW);
        break;
      case BACK:
        back();
     }    
  }

  delay(DELAY_PERIOD);
}

void robotStop(){
  if(dir == FORWARD){
    md.setBrakes(300, 300);
  }else{
    md.setBrakes(400, 400);
  }
  
  motorRNetEncoderCount = 0;
  motorLNetEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
}

//void calibrateSide(){
//  readAllSensors();
//  float RL = sensorReadings[RIGHT]-RIGHT_TOP_OFFSET;
//  float RR = sensorReadings[RIGHT + 1];
//  
//  Serial.print(RL);
//  Serial.print(" ");
//  Serial.println(RR);
//
//  float leftTolerance = 30;
//  float rightTolerance = 30;
//  if((LL+LR)/2 < leftTolerance){
//    calibrateRotation(LEFT, false);
//  }else if((RR+RL)/2 < rightTolerance){
//    calibrateRotation(RIGHT, false);
//  }
//}

void calibrateRotation(int side){
  calibrateRotation(side, true);
}

void calibrateRotation(int side, bool readAll){
  if(readAll)
    readAllSensors();
  
  float L;
  float R;
  
  if(side == RIGHT){
    L = sensorReadings[RIGHT];
    R = sensorReadings[RIGHT + 1];
    L -= RIGHT_TOP_OFFSET;
  }else if(side == FRONT){
    L = sensorReadings[FRONT];
    R = sensorReadings[FRONT + 2];
  }
  
  float separation = 14.6;
  switch(side){
    case FRONT:
      separation = 14.6;
      break;
   case RIGHT:
      separation = 11;
  }
  
  Serial.print(L);
  Serial.print(" ");
  Serial.print(R);
  Serial.print(" ");
  
  float tolerance = 0;
  if(abs(L-R) > tolerance){
    float diff = L-R;
    float angle = abs(atan2(diff, separation) * 180 / M_PI);
    if(int(angle) <= 1)
      return;
    Serial.println(angle);
    if(diff > tolerance){
      rotate(angle, CW);
    }else if(diff < -tolerance){
      rotate(angle, CCW);
    }
  }
}

void calibrateDistance(){
  readAllSensors();
  float L = sensorReadings[FRONT];
  float R = sensorReadings[FRONT + 2];
  float avg = (L+R)/2;
  float cutoff = 8.5;
  float tolerance = 0.1;
  if(abs(avg - cutoff) > tolerance){
    float frontDiff = avg - cutoff;
    if(frontDiff > tolerance){
      straight(frontDiff);
    }else if(frontDiff < -tolerance){
      straight(frontDiff);
    }
  }
}

void resetMove(){
  motorLAccmEncoderCount = 0;
  motorRAccmEncoderCount = 0;
  motorRNetEncoderCount = 0;
  motorLNetEncoderCount = 0;
  motorLRun = true;
  motorRRun = true;
}

void move(float leftPWM, float rightPWM, int setPoint){
  while(true){
    if(motorLRun && abs(motorLNetEncoderCount) < setPoint){
      md.setM2Speed(leftPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
    
    if(motorRRun && abs(motorRNetEncoderCount) < setPoint){
      md.setM1Speed(rightPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
  }
}

void straight(float dist){
  resetMove();
  motorLPWM = FORWARD_PWM_L;
  motorRPWM = FORWARD_PWM_R;
  if(dist > 0){
    LMag = -1;
    RMag = 1;
  }else{
    dist = -dist;
    LMag = 1;
    RMag = -1;
  }
  motorLPWM = LMag * FORWARD_PWM_L;
  motorRPWM = RMag * FORWARD_PWM_R;
  motorDistChkPt = FORWARD_DIST*dist/10.0;

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void rotate(float angle, byte dir){
  resetMove();
  if(dir == CW){
    motorLPWM = -CW_PWM_L;
    motorRPWM = -CW_PWM_R;
    motorDistChkPt = CW_DIST*angle/90.0;
  }else{
    motorLPWM = CCW_PWM_L;
    motorRPWM = CCW_PWM_R;
    motorDistChkPt = CCW_DIST*angle/90.0;
  }

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void back(){
  resetMove();
  motorLPWM = BACK_PWM_L;
  motorRPWM = BACK_PWM_R;
  motorDistChkPt = BACK_DIST;

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void motorLISR() {    //ISR for left motor encoder interrupt
  motorLAccmEncoderCount++;
  
  if(digitalReadFast(MOTOR_L_ENCODER_A) == HIGH) {  //Low-to-high edge on channel A
    //Check channel B to see which way the motor is turning
    if (digitalReadFast(MOTOR_L_ENCODER_B) == LOW) {
      motorLNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorLNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
  else {  //High-to-low edge on channel A
    //Check channel B to see which way encoder is turning
    if (digitalReadFast(MOTOR_L_ENCODER_B) == HIGH) {
      motorLNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorLNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
}

void motorRISR() {    //ISR for right motor encoder interrupt
  motorRAccmEncoderCount++;
  
  if(digitalReadFast(MOTOR_R_ENCODER_A) == HIGH) {  //Low-to-high edge on channel A
    //Check channel B to see which way the motor is turning
    if (digitalReadFast(MOTOR_R_ENCODER_B) == HIGH) {
      motorRNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorRNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
  else {  //High-to-low edge on channel A
    //Check channel B to see which way encoder is turning
    if (digitalReadFast(MOTOR_R_ENCODER_B) == LOW) {
      motorRNetEncoderCount++;      //Motor is turning forward (clockwise)
    }
    else motorRNetEncoderCount--;   //Motor is moving backward (anti-clockwise)
  }
}

