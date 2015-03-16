#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <DistanceGP2Y0A21YK.h>
#include <digitalWriteFast.h>

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

//PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 300, 0, 0, DIRECT);  //PID that controls the PWM of left motor
//PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 300, 0, 0, DIRECT);  //PID that controls the PWM of right motor

const byte MODE_STOP = B00;
const byte MODE_EXPLORE = B01;
const byte MODE_SHORTESTPATH = B10;
const byte MODE_CALIBRATE = B11;

/************** Decoded Command **************/
byte    mode = MODE_STOP;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    dir = B11;          //Forward = 11, Left = 01, Right = 10, Backward = 00
int     travelDist = 260;     //Distance (in ticks count) required to be travelled by the motor
/******************** END ********************/

/******************* Flags *******************/
boolean sense = false;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
boolean hasSent = false;
volatile boolean start = false;
/******************** END ********************/

double FORWARD_PWM_L = 166;
double FORWARD_PWM_R = 167;
int FORWARD_DIST = 550;

double CCW_PWM_L = 200;
double CCW_PWM_R = 189;
int CCW_DIST = 783; // 783
double CW_PWM_L = 180;
double CW_PWM_R = 180;
int CW_DIST = 760; // 706
double BACK_PWM_L = 180;
double BACK_PWM_R = 180;
int BACK_DIST = 1610;

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int lspeed = 0;
int rspeed = 0;
float sensorReadings[5] = {};

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
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, CHANGE);  //Attach right motor encoder interrupt pin to the ISR
  
}

boolean offPath = false;

void loop(){
  if(!start){
    return;
  }
    
  if(checkFront()){
    moveStraight(10);
    delay(200);
  }else{
    if(offPath){
      robotStop();
      return;
    }
//    rotateCW(90);
//    delay(200);
//    moveStraight(10);
//    delay(200);
//    moveStraight(10);
//    delay(200);
//    rotateCCW(90);
//    delay(200);
//    for(int i = 0; i<5;i++){
//      moveStraight(10);
//      delay(200);
//    }
//    rotateCCW(90);
//    delay(200);
//    moveStraight(10);
//    delay(200);
//    moveStraight(10);
//    delay(200);
//    rotateCW(90);
//    delay(200);
    drift(true);
    delay(500);
    drift(false);
    delay(500);
    drift(false);
    delay(500);
    drift(true);
    delay(500);
    offPath = true;
  }
}

boolean checkFront(){
  for(int i=0; i<50; i++) {
    sensorReadings[0] = frontMiddle.getDistanceMedian();
//    sensorReadings[1] = frontMiddle.getDistanceMedian();
//    sensorReadings[2] = frontRight.getDistanceMedian();
  }
  Serial.println(sensorReadings[0]);
  return sensorReadings[0] > 30;
}

boolean checkLeft(){
  for(int i=0; i<50; i++) {
    sensorReadings[4] = left.getDistanceMedian();
  }

  return sensorReadings[4] > 12;
}

boolean checkRight(){
  for(int i=0; i<50; i++) {
    sensorReadings[4] = left.getDistanceMedian();
  }

  return sensorReadings[3] > 12;
}

void readAllSensors() {
  int l = mode == MODE_CALIBRATE ? 10 : 50;
  for(int i=0; i<l; i++) {
    sensorReadings[0] = frontLeft.getDistanceMedian();
    sensorReadings[1] = frontMiddle.getDistanceMedian();
    sensorReadings[2] = frontRight.getDistanceMedian();
    sensorReadings[3] = right.getDistanceMedian();
    sensorReadings[4] = left.getDistanceMedian();
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
      if(sense)
        return;
      mode = (command & 0x0C) >> 2;         //Bit 0-1 represents start or stop
//      if(mode == B11){
//        robotStop();
//        return;
//      }
      dir = (command & 0x03);    //Bits 2-3 represent direction to move
    }
    
    if(data2 != -1 && dir == B11) {
      command = byte(data2);
      motorDistChkPt = int(command & 0x0F) * travelDist;  //Bits 3-6 represent number of grids required to move (1 grid = 10cm)
    }
//    configMove();                     //Configure variables for the next move
  }
}

void configMove(){
//  if(mode == MODE_STOP) {
//    robotStop();
//    return;
//  }
  
  if(!motorLRun && !motorRRun){
    
    motorLAccmEncoderCount = 0;
    motorRAccmEncoderCount = 0;
    motorLPrevAccmEncoderCount = 0;
    motorRPrevAccmEncoderCount = 0;
    motorLRun = true;
    motorRRun = true;
    hasSent = false;
    sense = true;
    
    switch(dir){
      case B11:
//        targetMotorSpeed = 0.7;
        motorLPWM = FORWARD_PWM_L;
        motorRPWM = FORWARD_PWM_R;
        LMag = -1;
        RMag = 1;
        motorDistChkPt = FORWARD_DIST;
        break;
      case B10:
        motorLPWM = CCW_PWM_L;
        motorRPWM = CCW_PWM_R;
        LMag = 1;
        RMag = 1;
        motorDistChkPt = CCW_DIST;
//          rotateCW(90);
        break;
      case B01:
        motorLPWM = CW_PWM_L;
        motorRPWM = CW_PWM_R;
        LMag = -1;
        RMag = -1;
        motorDistChkPt = CW_DIST;
//          rotateCCW(90);
        break;
      case B00:
        motorLPWM = BACK_PWM_L;
        motorRPWM = BACK_PWM_R;
        LMag = 1;
        RMag = 1;
        motorDistChkPt = BACK_DIST;
     }    
  }
}

//void controlRobot() {       //Control the motors based on the command and PWM values given
//  
//  currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.
//
//  motorAccmEncoderCount = (motorLAccmEncoderCount + motorRAccmEncoderCount) / 2;
//  motorLSpeed = abs((double)(motorLAccmEncoderCount - motorLPrevAccmEncoderCount)) / (currTime - prevTime);
//  motorRSpeed = abs((double)(motorRAccmEncoderCount - motorRPrevAccmEncoderCount)) / (currTime - prevTime);
//  prevTime = currTime;
//  motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
//  motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
//  
//  
//  delay(10);
//}

void robotMove(){
  if(mode == MODE_STOP) {
    robotStop();
    return;
  }
  
  if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
    md.setM2Speed(LMag*motorLPWM/255.0*400.0);
  }else {
    robotStop();
  }
  
  if(motorRRun && motorRAccmEncoderCount < motorDistChkPt){
    md.setM1Speed(RMag*motorRPWM/255.0*400.0);
  }else {
    robotStop();
  }
}

void robotStop(){
  if(dir == B11){
    md.setBrakes(300, 300);
  }else{
    md.setBrakes(400, 400);
  }
  targetLSpeed = 0;
  targetRSpeed = 0;
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
  if(mode != MODE_CALIBRATE)
    mode = MODE_STOP;
}

int calibrateCount = 0;

void calibrateFront(){


  if(mode == MODE_STOP)
    return;
    
  readAllSensors();
  
  float FL = sensorReadings[0];
  float FM = sensorReadings[1];
  float FR = sensorReadings[2];
  
  float sensorMin = min(min(FL, FM), FR);
  if(sensorMin < 7){
    moveStraight(-7+sensorMin);
  }
  
  readAllSensors();
  FL = sensorReadings[0];
  FM = sensorReadings[1];
  FR = sensorReadings[2];
  
  // If left distance > right distance, turn clockwise
  while(abs(FL-FR) > 0.5 && calibrateCount < 20){
    Serial.print(FL);
    Serial.print(",");
    Serial.println(FR);
    float diff = FL-FR;
    float angle = max(1, abs(atan2(diff, 13) * 180 / M_PI));
    if(diff > 0.5){
      rotateCW(angle);
    }else if(diff < -0.5){
      rotateCCW(angle);
    }
    
    readAllSensors();
    calibrateCount++;
    FL = sensorReadings[0];
    FM = sensorReadings[1];
    FR = sensorReadings[2];
  }
  
  calibrateCount = 0;
  
  readAllSensors();
  FL = sensorReadings[0];
  FM = sensorReadings[1];
  FR = sensorReadings[2];
  
  float avg = (FL+FM+FR)/3;
  
  while(abs(avg-8.5) > 0.1 && calibrateCount < 5){
    float frontDiff = avg -8.5;
    if(frontDiff > 0.1){
      moveStraight(frontDiff);
    }else if(frontDiff < -0.1){
      moveStraight(frontDiff);
    }
    
    readAllSensors();
    calibrateCount++;
    FL = sensorReadings[0];
    FM = sensorReadings[1];
    FR = sensorReadings[2];
    avg = (FL+FM+FR)/3;
  }
  
   mode = MODE_STOP;
   hasSent = false;
   calibrateCount = 0;
}

void rotateCCW(float angle){
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;
  motorLRun = true;
  motorRRun = true;
  motorLPWM = CCW_PWM_L;
  motorRPWM = CCW_PWM_R;
  LMag = 1;
  RMag = 1;
  motorDistChkPt = CCW_DIST*angle/90.0;
  
//  Serial.print("CCW ");
//  Serial.println(motorDistChkPt);
  
  while(true){
    if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
      md.setM2Speed(LMag*motorLPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
    
    if(motorRRun && motorRAccmEncoderCount < motorDistChkPt){
      md.setM1Speed(RMag*motorRPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
  }
}

void moveStraight(float dist){
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;
  motorLRun = true;
  motorRRun = true;
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
  motorDistChkPt = FORWARD_DIST*dist/10.0;

//  Serial.print("F ");
//  Serial.println(motorDistChkPt);
  
  while(true){
    if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
      md.setM2Speed(LMag*motorLPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
    
    if(motorRRun && motorRAccmEncoderCount < motorDistChkPt){
      md.setM1Speed(RMag*motorRPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
  }
}

void drift(boolean isRight){
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;
  motorLRun = true;
  motorRRun = true;
  motorLPWM = -237;
  motorRPWM = 100;
  if(!isRight){
    motorLPWM = -100;
    motorRPWM = 237;
  }
  LMag = 1;
  RMag = 1;
  motorDistChkPt = 2400;

//  Serial.print("F ");
//  Serial.println(motorDistChkPt);
  
  while(true){
    if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
      md.setM2Speed(LMag*motorLPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
    
    if(motorRRun && motorRAccmEncoderCount < motorDistChkPt){
      md.setM1Speed(RMag*motorRPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
  }
}

void rotateCW(float angle){
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;
  motorLRun = true;
  motorRRun = true;
  motorLPWM = CW_PWM_L;
  motorRPWM = CW_PWM_R;
  LMag = -1;
  RMag = -1;
  motorDistChkPt = CW_DIST*angle/90.0;
  
//  Serial.print("CW ");
//  Serial.println(motorDistChkPt);
  
  while(true){
    if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
      md.setM2Speed(LMag*motorLPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
    
    if(motorRRun && motorRAccmEncoderCount < motorDistChkPt){
      md.setM1Speed(RMag*motorRPWM/255.0*400.0);
    }else {
      robotStop();
      break;
    }
  }
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

