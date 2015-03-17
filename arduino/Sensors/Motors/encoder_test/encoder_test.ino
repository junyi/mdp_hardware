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
int BACK_DIST = 1621;

float RIGHT_TOP_OFFSET = 12; 

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int lspeed = 0;
int rspeed = 0;
float sensorReadings[6] = {};
float previousLeftReading = 0;
float previousRightReading = 0;
volatile int motorLOldA=0, motorROldA=0, motorLNewB=0, motorRNewB=0;
int LMag = 1;
int RMag = -1;

void setup(){
  Serial.begin(115200);
  
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

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISRA, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_L_ENCODER_B, &motorLISRB, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISRA, CHANGE);  //Attach right motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_B, &motorRISRB, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  
}

void loop(){
  Serial.print(motorLNetEncoderCount);
  Serial.print(" ");
  Serial.println(motorRNetEncoderCount);
}

void motorLISRA() {    //ISR for left motor encoder interrupt
  motorLNewB ^ motorLOldA ? motorLNetEncoderCount++: motorLNetEncoderCount--;
  motorLOldA = digitalReadFast(MOTOR_L_ENCODER_A);
}

void motorLISRB() {    //ISR for left motor encoder interrupt
  motorLNewB = digitalReadFast(MOTOR_L_ENCODER_B);
  motorLNewB ^ motorLOldA ? motorLNetEncoderCount++ : motorLNetEncoderCount--;
}

void motorRISRA() {    //ISR for left motor encoder interrupt
  motorRNewB ^ motorROldA ? motorRNetEncoderCount--: motorRNetEncoderCount++;
  motorROldA = digitalReadFast(MOTOR_R_ENCODER_A);
}

void motorRISRB() {    //ISR for left motor encoder interrupt
  motorRNewB = digitalReadFast(MOTOR_R_ENCODER_B);
  motorRNewB ^ motorROldA ? motorRNetEncoderCount-- : motorRNetEncoderCount++;
}
