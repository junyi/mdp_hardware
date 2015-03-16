#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <digitalWriteFast.h>

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
double  motorLPWM = 0;                   //PWM (0 - 255) of left motor

double  motorRSpeed = 0;                 //Rotation speed of right motor
double  targetRSpeed = 0;                //Target speed of right motor
double  motorRPWM = 0;                   //PWM (0 - 255) of right motor

double  motorAccmEncoderCount = 0;       //Average accumulated encoder's ticks count of both motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
double  targetMotorSpeed = 0;            //Target speed should be achieved by both motors

unsigned long prevTime = 0;              //Previous timestamp checked for motors' speeds
unsigned long currTime = 0;              //Current timestamp

//PID     motorSpeedPID(&motorAccmEncoderCount, &targetMotorSpeed, &motorDistChkPt, 0.001, 0, 0, DIRECT);  //PID for calculating target speed
//PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 600, 0, 0, DIRECT);  //PID that controls the PWM of left motor
//PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 600, 0, 0, DIRECT);  //PID that controls the PWM of right motor


/************** Decoded Command **************/
byte    mode = B0;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    start = B0;         //Start = 1, Stop = 0
byte    dir = B00;          //Forward = 00, Left = 01, Right = 10, Backward = 11x
int     travelDist = 300;     //Distance (in ticks count) required to be travelled by the motor
int     angle = 0;          //Turning angle instructed
/******************** END ********************/

/******************* Flags *******************/
boolean sense = true;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
/******************** END ********************/

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int x = 550; //distance in straight line
int lspeed = 0;
int rspeed = 0;
double sensorReadings[4] = {};
byte nextDir = B00;

int LMag = 1;
int RMag = -1;
int count = 0;
//byte seq[] = {B00, B10, B00, B10, B00, B10, B00, B10};
byte seq[] = {B01};


void setup(){
  Serial.begin(9600);
  
  inputString.reserve(20);
    
  pinMode(MOTOR_L_ENCODER_A, INPUT);
  pinMode(MOTOR_L_ENCODER_B, INPUT);
  pinMode(MOTOR_R_ENCODER_A, INPUT);
  pinMode(MOTOR_R_ENCODER_B, INPUT);
  
  
  /********** PID  Configurations **********/
//  motorSpeedPID.SetMode(AUTOMATIC);
//  motorLPID.SetMode(AUTOMATIC);
//  motorRPID.SetMode(AUTOMATIC);
//  motorSpeedPID.SetOutputLimits(0.15, 0.73);
//  motorLPID.SetOutputLimits(98.9, 255);
//  motorRPID.SetOutputLimits(100, 255);
//  motorSpeedPID.SetSampleTime(50);
//  motorLPID.SetSampleTime(10);
//  motorRPID.SetSampleTime(10);
  /****************** END ******************/
  
//  motorLPID.SetTunings(600, 0, 0);
//  motorRPID.SetTunings(600, 0, 0);
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, CHANGE);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, CHANGE);  //Attach right motor encoder interrupt pin to the ISR
  
}

void loop(){
  configMove();
//   controlRobot();
//    if(dir == B00)    
//      if(motorLRun || motorRRun)
//        computePID();
    
    robotMove();
  
//  Serial.print(currTime);
//  Serial.print(" ");
//  Serial.print(motorLPWM);
//  Serial.print(" ");
//  Serial.print(motorRPWM);
//  Serial.print(" ");
//  Serial.print(motorLAccmEncoderCount);
//  Serial.print(" ");
//  Serial.print(motorRAccmEncoderCount);
//  Serial.print(" ");
//  Serial.print(targetLSpeed);
//  Serial.print(" ");
//  Serial.print(targetRSpeed);
//  Serial.print(" ");
//  Serial.print(motorLSpeed);
//  Serial.print(" ");
//  Serial.println(motorRSpeed);
  //motorSpeedPID.Compute(); 
  
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.print("// Received: ");
    Serial.println(inputString);
    int index = inputString.indexOf(' ');
    if(index != -1){
      lspeed = inputString.substring(0, index).toFloat();      
      rspeed = inputString.substring(index + 1).toFloat();      
    }

    if(inputString.charAt(0) == 's'){
      lspeed = 0;
      rspeed = 0;
    };
    
    if(inputString.charAt(0) == 'd'){
      seq[0] = inputString.substring(0, 1).toInt() % 4;
    };
    
    if(inputString.charAt(0) == 'b'){
      x = inputString.substring(1).toInt();
    };
    
    if(inputString.charAt(0) == 'k'){
      count = 0;
    };
    
    if(inputString.charAt(0) == 't'){
      Serial.print(LMag*motorLPWM);
      Serial.print(" ");
      Serial.println(RMag*motorRPWM);
    }
    
//    if(inputString.charAt(0) == 'p'){
//      int spaceOne = inputString.indexOf(' ');
//      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
//      
//      char buffer[10];
//
//      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
//      double kp = atof(buffer);
//      
//      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
//      double ki = atof(buffer);
//      
//      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
//      double kd = atof(buffer);
//      
//      motorLPID.SetTunings(kp, ki, kd);
//      motorRPID.SetTunings(kp, ki, kd);
//    };
//    
//    if(inputString.charAt(0) == 'l'){
//      int spaceOne = inputString.indexOf(' ');
//      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
//      
//      char buffer[10];
//
//      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
//      double kp = atof(buffer);
//      
//      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
//      double ki = atof(buffer);
//      
//      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
//      double kd = atof(buffer);
//      
//      motorLPID.SetTunings(kp, ki, kd);
//    };
//    
//    if(inputString.charAt(0) == 'r'){
//      int spaceOne = inputString.indexOf(' ');
//      int spaceTwo = inputString.indexOf(' ', spaceOne + 1);
//      
//      char buffer[10];
//
//      inputString.substring(1, spaceOne).toCharArray(buffer, 10);
//      double kp = atof(buffer);
//      
//      inputString.substring(spaceOne + 1, spaceTwo).toCharArray(buffer, 10);
//      double ki = atof(buffer);
//      
//      inputString.substring(spaceTwo + 1).toCharArray(buffer, 10);
//      double kd = atof(buffer);
//      
//      motorRPID.SetTunings(kp, ki, kd);
//    };
//    
//    if(inputString.charAt(0) == 't'){
//      Serial.print("//l ");
//      Serial.print(motorLPID.GetKp());
//      Serial.print(" ");
//      Serial.print(motorLPID.GetKi());
//      Serial.print(" ");
//      Serial.println(motorLPID.GetKd());
//      
//      Serial.print("//r ");
//      Serial.print(motorRPID.GetKp());
//      Serial.print(" ");
//      Serial.print(motorRPID.GetKi());
//      Serial.print(" ");
//      Serial.println(motorRPID.GetKd());
//    }
    
//    md.setM2Speed(lspeed);
//    md.setM1Speed(rspeed);
    motorLPWM = lspeed;
    motorRPWM = rspeed;
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}


void configMove(){
  if(!motorLRun && !motorRRun){
  
    if(count < sizeof(seq)){
      dir = seq[count];
    }else{
      return;
    }
    
//    Serial.print("..");
//    Serial.println(dir);
    motorLAccmEncoderCount = 0;
    motorRAccmEncoderCount = 0;
    motorLPrevAccmEncoderCount = 0;
    motorRPrevAccmEncoderCount = 0;
    motorLRun = true;
    motorRRun = true;
    
    switch(dir){
      case B00:
         targetMotorSpeed = 0.8;
        LMag = -1;
        RMag = 1;
        motorDistChkPt = x;
        break;
      case B01:
        targetMotorSpeed = 0.35;
        LMag = -1;
        RMag = -1;
//        motorLPWM = 200;
//        motorRPWM = 189.5;
        motorDistChkPt = x;
        break;
      case B10:
        targetMotorSpeed = 0.35;
        LMag = 1;
        RMag = 1;
        motorDistChkPt = x;
        break;
      case B11:
        targetMotorSpeed = 0.5;
        LMag = 1;
        RMag = -1;
        motorDistChkPt = x;
    }
    
//    targetRSpeed = targetMotorSpeed;
//    targetLSpeed = targetMotorSpeed;

//    motorRPWM = targetRSpeed * 204.906 + 3.02;    
//    motorLPWM = motorRPWM + 0.71 - 7.409 * targetLSpeed;

    
    if(count < sizeof(seq)){
      count++;
    }
    
  }
}

void robotMove(){
  
  if(motorLRun && motorLAccmEncoderCount < motorDistChkPt){
    md.setM2Speed(LMag*motorLPWM/255.0*400.0 );
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
  if(dir == B00 || dir == B11){
    md.setM1Brake(300);
    md.setM2Brake(300);
  }else{
    md.setM1Brake(400);
    md.setM2Brake(400);
  }
  targetLSpeed = 0;
  targetRSpeed = 0;
  motorRAccmEncoderCount = 0;
  motorLAccmEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
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

void computePID() {         //Adjust PID of both motors to make the robot moves in straight line
  targetLSpeed = targetMotorSpeed;
  targetRSpeed = targetMotorSpeed;
//  if(motorAccmEncoderCount < 150){
//    if(targetLSpeed < targetMotorSpeed){
//      targetLSpeed += targetMotorSpeed/4.0;
//    }
//    
//    if(targetRSpeed < targetMotorSpeed){
//      targetRSpeed += targetMotorSpeed/4.0;
//    }
//  }
//  
//  if(motorAccmEncoderCount > 410){
//    if(targetLSpeed > 0){
//      targetLSpeed -= targetMotorSpeed/4.0;
//    }
//    
//    if(targetRSpeed > 0){
//      targetRSpeed -= targetMotorSpeed/4.0;
//    }
//  }

//  motorLPID.Compute();  //Compute right motor PWM based on the right motor speed w.r.t the target speed
//  motorRPID.Compute();  //Compute left motor PWM based on the left motor speed w.r.t the target speed
  
}

void controlRobot() {       //Control the motors based on the command and PWM values given
  
  currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.

  motorAccmEncoderCount = (motorLAccmEncoderCount + motorRAccmEncoderCount) / 2;
  motorLSpeed = abs((double)(motorLAccmEncoderCount - motorLPrevAccmEncoderCount)) / (currTime - prevTime);
  motorRSpeed = abs((double)(motorRAccmEncoderCount - motorRPrevAccmEncoderCount)) / (currTime - prevTime);
  prevTime = currTime;
  motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
  motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
  
  if(motorLRun || motorRRun)
    computePID();   

  robotMove();
  
  delay(10);
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

