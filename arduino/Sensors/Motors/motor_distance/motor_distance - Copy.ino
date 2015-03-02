#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <MazebusterIR.h>
#include <PID_AutoTune_v0.h>

#define MOTOR_L_ENCODER_A 3
#define MOTOR_L_ENCODER_B 5
#define MOTOR_L_SPEED 9
#define MOTOR_L_DIR_F 2
#define MOTOR_L_DIR_B 4

#define MOTOR_R_ENCODER_A 11
#define MOTOR_R_ENCODER_B 13
#define MOTOR_R_DIR_F 8
#define MOTOR_R_DIR_B 7
#define MOTOR_R_SPEED 10

#define SENSOR_IR_FRONT A0
#define SENSOR_IR_LEFT A2
#define SENSOR_IR_RIGHT A1

DualVNH5019MotorShield md; 

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
double  motorLPWM = 150;                   //PWM (0 - 255) of left motor

double  motorRSpeed = 0;                 //Rotation speed of right motor
double  targetRSpeed = 0;                //Target speed of right motor
double  motorRPWM = 150;                   //PWM (0 - 255) of right motor

double  motorAccmEncoderCount = 0;       //Average accumulated encoder's ticks count of both motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
double  targetMotorSpeed = 0;            //Target speed should be achieved by both motors

unsigned long prevTime = 0;              //Previous timestamp checked for motors' speeds
unsigned long currTime = 0;              //Current timestamp

PID     motorSpeedPID(&motorAccmEncoderCount, &targetMotorSpeed, &motorDistChkPt, 0.001, 0, 0, DIRECT);  //PID for calculating target speed
PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 10, 0.1, 0, DIRECT);  //PID that controls the PWM of left motor
PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 10, 0.1, 0, DIRECT);  //PID that controls the PWM of right motor
//PID     motorLPID(&motorLAccmEncoderCount, &motorLPWM, &motorDistChkPt, 12.5, 1, 1, DIRECT);  //PID that controls the PWM of left motor
//PID     motorRPID(&motorRAccmEncoderCount, &motorRPWM, &motorDistChkPt, 10, 1, 1, DIRECT);  //PID that controls the PWM of right motor

MazebusterIR irFront(SENSOR_IR_FRONT, 11, 25, 93);
MazebusterIR irLeft(SENSOR_IR_LEFT, 3, 25, 93);
MazebusterIR irRight(SENSOR_IR_RIGHT, 1, 25, 93);
/******************** END ********************/

/******************* Flags *******************/
boolean sense = true;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
/******************** END ********************/

/************** Decoded Command **************/
byte    mode = B0;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    start = B0;         //Start = 1, Stop = 0
byte    dir = B10;          //Forward = 00, Left = 01, Right = 10, Backward = 11
int     travelDist = 300;     //Distance (in ticks count) required to be travelled by the motor
int     angle = 0;          //Turning angle instructed
/******************** END ********************/

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int x = 560; //distance in straight line
int lspeed = 0;
int rspeed = 0;
double sensorReadings[3] = {};
byte nextDir = B00;

int LMag = -1;
int RMag = 1;
int count = 0;
//byte seq[] = {B00, B10, B00, B10, B00, B10, B00, B10};
byte seq[] = {B00};


void setup(){
  Serial.begin(9600);
  
  inputString.reserve(20);
    
  pinMode(MOTOR_L_ENCODER_A, INPUT);
  pinMode(MOTOR_L_ENCODER_B, INPUT);
  pinMode(MOTOR_R_ENCODER_A, INPUT);
  pinMode(MOTOR_R_ENCODER_B, INPUT);
  
  pinMode(SENSOR_IR_FRONT, INPUT);
  pinMode(SENSOR_IR_LEFT, INPUT);
  pinMode(SENSOR_IR_RIGHT, INPUT);
  
  /********** PID  Configurations **********/
  motorSpeedPID.SetMode(AUTOMATIC);
  motorLPID.SetMode(AUTOMATIC);
  motorRPID.SetMode(AUTOMATIC);
  motorSpeedPID.SetOutputLimits(0.15, 0.73);
  motorLPID.SetOutputLimits(98.9, 255);
  motorRPID.SetOutputLimits(100, 255);
  motorSpeedPID.SetSampleTime(50);
  motorLPID.SetSampleTime(50);
  motorRPID.SetSampleTime(50);
  /****************** END ******************/
  
  motorLPID.SetTunings(10, 0.1, 0);
  motorRPID.SetTunings(10, 0.1, 0);
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, RISING);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, RISING);  //Attach right motor encoder interrupt pin to the ISR
  
}

void loop(){
  configMove();
  motorLPID.Compute();
//  motorLPID.Compute();
  motorRPID.Compute();
//  motorSpeedPID.Compute();
  controlRobot();
  
  readAllSensors();
    
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(sensorReadings[0]);
  Serial.print(" ");
  Serial.print(sensorReadings[1]);
  Serial.print(" ");
  Serial.print(sensorReadings[2]);
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
  Serial.print(motorRSpeed);
  Serial.print(" ");
  Serial.print(motorLPID.GetError());
  Serial.print(" ");
  Serial.println(motorRPID.GetError());
  //motorSpeedPID.Compute(); 
  
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.print("// Received: ");
    Serial.println(inputString);
    int index = inputString.indexOf(' ');
    if(index != -1){
      lspeed = inputString.substring(0, index).toInt()*400/255.0;      
      rspeed = inputString.substring(index + 1).toInt()*400/255.0;      
    }

    if(inputString.charAt(0) == 's'){
      lspeed = 0;
      rspeed = 0;
    };
    
    if(inputString.charAt(0) == 'd'){
      dir = inputString.substring(0, 1).toInt() % 4;
    };
    
    if(inputString.charAt(0) == 'b'){
      x = inputString.substring(1).toInt();
    };
    
    if(inputString.charAt(0) == 'k'){
      count = 0;
    };
    
    if(inputString.charAt(0) == 'p'){
      int spaceOne = inputString.indexOf(' ');
      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
      
      char buffer[10];

      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
      double kp = atof(buffer);
      
      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
      double ki = atof(buffer);
      
      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
      double kd = atof(buffer);
      
      motorLPID.SetTunings(kp, ki, kd);
      motorRPID.SetTunings(kp, ki, kd);
    };
    
    if(inputString.charAt(0) == 'l'){
      int spaceOne = inputString.indexOf(' ');
      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
      
      char buffer[10];

      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
      double kp = atof(buffer);
      
      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
      double ki = atof(buffer);
      
      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
      double kd = atof(buffer);
      
      motorLPID.SetTunings(kp, ki, kd);
    };
    
    if(inputString.charAt(0) == 'r'){
      int spaceOne = inputString.indexOf(' ');
      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
      
      char buffer[10];

      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
      double kp = atof(buffer);
      
      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
      double ki = atof(buffer);
      
      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
      double kd = atof(buffer);
      
      motorRPID.SetTunings(kp, ki, kd);
    };
    
    if(inputString.charAt(0) == 't'){
      Serial.print("//l ");
      Serial.print(motorLPID.GetKp());
      Serial.print(" ");
      Serial.print(motorLPID.GetKi());
      Serial.print(" ");
      Serial.println(motorLPID.GetKd());
      
      Serial.print("//r ");
      Serial.print(motorRPID.GetKp());
      Serial.print(" ");
      Serial.print(motorRPID.GetKi());
      Serial.print(" ");
      Serial.println(motorRPID.GetKd());
    }
    
    md.setM2Speed(lspeed);
    md.setM1Speed(rspeed);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}

void readAllSensors(){
  sensorReadings[0] = irFront.distance();
  sensorReadings[1] = irLeft.distance();
  sensorReadings[2] = irRight.distance();
}

void configMove(){
  if(!motorLRun && !motorRRun){
    delay(500);
    
    if(count < sizeof(seq)){
      dir = seq[count];
    }else{
      return;
    }
    
    Serial.print("..");
    Serial.println(dir);
    motorLAccmEncoderCount = 0;
    motorRAccmEncoderCount = 0;
    motorLPrevAccmEncoderCount = 0;
    motorRPrevAccmEncoderCount = 0;
    motorLRun = true;
    motorRRun = true;
    
    switch(dir){
      case B00:
        targetMotorSpeed = 0.5;
        LMag = -1;
        RMag = 1;
        motorDistChkPt = x;
        break;
      case B01:
        targetMotorSpeed = 0.35;
        LMag = 1;
        RMag = 1;
        motorDistChkPt = 560;
        break;
      case B10:
        targetMotorSpeed = 0.35;
        LMag = -1;
        RMag = -1;
        motorDistChkPt = 560;
        break;
      case B11:
        targetMotorSpeed = 0.5;
        LMag = 1;
        RMag = -1;
        motorDistChkPt = x;
    }
    
    targetRSpeed = targetMotorSpeed;
    targetLSpeed = targetMotorSpeed;

//    motorRPWM = targetRSpeed * 204.906 + 3.02;    
//    motorLPWM = motorRPWM + 0.71 - 7.409 * targetLSpeed;

    
    if(count < sizeof(seq)){
      count++;
    }
    
  }
  
  
}

void robotMove(){
  
  if(motorLAccmEncoderCount < motorDistChkPt){
    md.setM2Speed(LMag*motorLPWM/255*400 );
  }else {
    md.setM2Speed(0);
    motorLRun = false;
  }
  
  if(motorRAccmEncoderCount < motorDistChkPt){
    md.setM1Speed(RMag*motorRPWM/255*400);
  }else {
    md.setM1Speed(0);
    motorRRun = false;
  }
}

void robotStop(){
  md.setM2Speed(0);
  md.setM1Speed(0);
  motorLRun = false;
  motorRRun = false;
}

void robotMoveForward(){
  if(motorLAccmEncoderCount < motorDistChkPt){
    md.setM2Speed(motorLPWM);
  }else {
    md.setM2Speed(0);
    motorLRun = false;
  }
  
  if(motorRAccmEncoderCount < motorDistChkPt){
    md.setM1Speed(motorRPWM);
  }else {
    md.setM1Speed(0);
    motorRRun = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    Serial.println(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}


void controlRobot() {       //Control the motors based on the command and PWM values given
  
  currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.

  motorAccmEncoderCount = (motorLAccmEncoderCount + motorRAccmEncoderCount) / 2;
  motorLSpeed = abs((double)(motorLAccmEncoderCount - motorLPrevAccmEncoderCount)) / (currTime - prevTime);
  motorRSpeed = abs((double)(motorRAccmEncoderCount - motorRPrevAccmEncoderCount)) / (currTime - prevTime);
  prevTime = currTime;
  motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
  motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
  
  robotMove();
  
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

