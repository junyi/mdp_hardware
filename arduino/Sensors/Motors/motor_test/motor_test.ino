#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>

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
int  motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
int  motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
double  motorLDistInitPt = 0;            //Initial check point of left motor's encoder's ticks
double  motorLDistChkPt = 0;             //Final distance/encoder check point of left motor

int  motorRPrevAccmEncoderCount = 0;  //Previous accumulated encoder's ticks count of right motor
int  motorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
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

/******************** PID ********************/
/*** Short distance (10cm) PID ***/
//PID motorLPID(&motorLAccmEncoderCount, &motorLSpeed, &motorDistChkPt, 0.19, 0.0100, 0.050, DIRECT);  //PID of left motor
//PID motorRPID(&motorRAccmEncoderCount, &motorRSpeed, &motorDistChkPt, 0.19, 0.0085, 0.050, DIRECT);  //PID of right motor

/*** Long distance PID ***/
//PID     motorLPID(&motorLAccmEncoderCount, &motorLSpeed, &motorDistChkPt, 0.170, 0.0055, 0.01, DIRECT);  //PID of left motor
//PID     motorRPID(&motorRAccmEncoderCount, &motorRSpeed, &motorDistChkPt, 0.175, 0.0095, 0.01, DIRECT);  //PID of right motor

//PID that controls the speed of left motor with respect to the check point
//PID     motorLPID(&motorLAccmEncoderCount, &motorLSpeed, &motorDistChkPt, 0.13, 0, 0, DIRECT);
//PID that controls the speed of right motor with respect to the speed of left motor
//PID     motorRPID(&motorRAccmEncoderCount, &motorRSpeed, &motorLAccmEncoderCount, 6.0, 5.5, 0.001, DIRECT);

PID     motorSpeedPID(&motorAccmEncoderCount, &targetMotorSpeed, &motorDistChkPt, 0.001, 0, 0, DIRECT);  //PID for calculating target speed
PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 80, 1835, 1, DIRECT);  //PID that controls the PWM of left motor
PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 78, 1865, 1, DIRECT);  //PID that controls the PWM of right motor
/******************** END ********************/

/******************* Flags *******************/
boolean sense = true;       //Allow sensors to sense and transmit data
boolean motorLRun = false;  //Allow left motor to run (based on the command given)
boolean motorRRun = false;  //Allow right motor to run (based on the command given)
/******************** END ********************/

/************** Decoded Command **************/
byte    mode = B0;          //Map exploration mode = 0, Shortest path travelling mode = 1
byte    start = B0;         //Start = 1, Stop = 0
byte    dir = B00;          //Forward = 00, Left = 01, Right = 10, Backward = 11
int     travelDist = 300;     //Distance (in ticks count) required to be travelled by the motor
int     angle = 0;          //Turning angle instructed
/******************** END ********************/

String inputString = "";

boolean stringComplete = false;  // whether the string is complete

int lspeed = 0;
int rspeed = 0;

void setup(){
  Serial.begin(9600);
  
  inputString.reserve(20);
    
  pinMode(MOTOR_L_ENCODER_A, INPUT);
  pinMode(MOTOR_L_ENCODER_B, INPUT);
  pinMode(MOTOR_R_ENCODER_A, INPUT);
  pinMode(MOTOR_R_ENCODER_B, INPUT);
  
  /********** PID  Configurations **********/
  motorSpeedPID.SetMode(AUTOMATIC);
  motorLPID.SetMode(AUTOMATIC);
  motorRPID.SetMode(AUTOMATIC);
  motorSpeedPID.SetOutputLimits(0.15, 0.73);
  motorLPID.SetOutputLimits(80, 255);
  motorRPID.SetOutputLimits(82, 255);
  motorSpeedPID.SetSampleTime(50);
  motorLPID.SetSampleTime(50);
  motorRPID.SetSampleTime(50);
  /****************** END ******************/
  
  md.init();

  PCintPort::attachInterrupt(MOTOR_L_ENCODER_A, &motorLISR, RISING);  //Attach left motor encoder interrupt pin to the ISR
  PCintPort::attachInterrupt(MOTOR_R_ENCODER_A, &motorRISR, RISING);  //Attach right motor encoder interrupt pin to the ISR
 
}



int count = 0;

void loop(){
  controlRobot();
  Serial.print(motorLNetEncoderCount);
  Serial.print(" ");
  Serial.print(motorRNetEncoderCount);
  Serial.print(" ");
  Serial.print(motorLSpeed);
  Serial.print(" ");
  Serial.println(motorRSpeed);
  targetLSpeed = targetMotorSpeed;
  targetRSpeed = targetMotorSpeed;
  motorSpeedPID.Compute(); 
  for(int i = 0; i < 10; i++){
    robotMoveForward();
  }
  
  
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
    
    md.setM2Speed(lspeed);
    md.setM1Speed(rspeed);
    // clear the string:
    inputString = "";
    stringComplete = false;
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


void configMove() {    //Configure the variable settings for the next movement  
  if(dir == 0x00 || dir == 0x03) {       //Calculate target distance required to move forward/backward
    travelDist = travelDist * 1.02;      //Increase the travel distance by 2%, due to slipping wheels
    motorLDistChkPt = motorLDistChkPt + travelDist;
    motorRDistChkPt = motorRDistChkPt + travelDist;
    motorDistChkPt = motorDistChkPt + travelDist;
  }
  else if(dir == 0x01 || dir == 0x02) {  //Calculate target distance for turning left/right
    motorLPrevAccmEncoderCount = 0;
    motorLAccmEncoderCount = 0;
    motorLDistChkPt = 0;
    
    motorRPrevAccmEncoderCount = 0;
    motorRAccmEncoderCount = 0;
    motorRDistChkPt = 0;
    
    motorAccmEncoderCount = 0;
    motorDistChkPt = 0;
    
    travelDist = (401 / 90.0) * angle;
    motorLDistChkPt = motorLDistChkPt + travelDist;
    motorRDistChkPt = motorRDistChkPt + travelDist;
    motorDistChkPt = motorDistChkPt + travelDist;
  }
  
  motorLDistInitPt = motorLAccmEncoderCount;  //Set initial encoder ticks count before moving
  motorRDistInitPt = motorRAccmEncoderCount;  //Set initial encoder ticks count before moving
  
  if(travelDist >= 1492) {  //Travel distance longer than 50cm
    motorLPID.SetTunings(80, 1600, 1);
    motorRPID.SetTunings(75, 1600, 1);
  }
  else {
    motorLPID.SetTunings(80, 1835, 1);
    motorRPID.SetTunings(78, 1865, 1);
  }
  
  motorLRun = true;  //Allow left motor to run
  motorRRun = true;  //Allow right motor to run
  sense = true;      //Allow sensors to read and transmit data to Serial Port
}

void robotMoveForward() {  //Move the robot forward for certain distance
  //Check whether left motor has reached the target distance
  if(motorLAccmEncoderCount < motorDistChkPt) {
    md.setM2Speed(motorLPWM);
  }
  else {
    md.setM2Speed(0);
    motorLRun = false;
  }
  
  //Check whether right motor has reached the target distance
  if(motorRAccmEncoderCount < motorDistChkPt) {
    md.setM1Speed(motorRPWM);
  }
  else {
    md.setM1Speed(0);
    motorRRun = false;
  }
}

void robotTurnLeft() {     //Turn the robot to left for certain angle
  //Check whether left motor has reached the target distance
  if(motorLAccmEncoderCount < motorDistChkPt) {
    md.setM2Speed(motorLPWM);
  }
  else {
    md.setM2Speed(0);
    motorLRun = false;
  }
  
  //Check whether right motor has reached the target distance
  if(motorRAccmEncoderCount < motorDistChkPt) {
    md.setM1Speed(motorRPWM);
  }
  else {
    md.setM1Speed(0);
    motorRRun = false;
  }
  
}

void controlRobot() {       //Control the motors based on the command and PWM values given
  
  currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.

  motorAccmEncoderCount = (motorLAccmEncoderCount + motorRAccmEncoderCount) / 2;
  motorLSpeed = ((double)(motorLAccmEncoderCount - motorLPrevAccmEncoderCount)) / (currTime - prevTime);
  motorRSpeed = ((double)(motorRAccmEncoderCount - motorRPrevAccmEncoderCount)) / (currTime - prevTime);
  prevTime = currTime;
  motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
  motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
  
  
  //int motorLTravelDist = motorLDistChkPt - motorLDistInitPt;
  //int motorRTravelDist = motorRDistChkPt - motorRDistInitPt;
  //int motorTravelDist = (motorLTravelDist + motorRTravelDist) / 2;
  
  /*** PID for short travelling distance, typically less than 50cm ***/
  //motorLSpeed = map(motorLAccmEncoderCount, motorLDistInitPt+(motorLTravelDist*0), motorLDistInitPt+(motorLTravelDist*0.50), 255, 46);
  //motorRSpeed = map(motorRAccmEncoderCount, motorRDistInitPt+(motorRTravelDist*0), motorRDistInitPt+(motorRTravelDist*0.50), 255, 54);
  
  /*** PID for long travelling distance, typically more than 50cm ***/
  //motorLSpeed = map(motorLAccmEncoderCount, motorLDistInitPt+(motorLTravelDist*0.75), motorLDistInitPt+(motorLTravelDist*0.90), 255, 46);
  //motorRSpeed = map(motorRAccmEncoderCount, motorRDistInitPt+(motorRTravelDist*0.75), motorRDistInitPt+(motorRTravelDist*0.90), 255, 54);
  
  /*** PID that dynamically changes motor speed based on travelling distance to setpoint distance ***/
  //motorLSpeed = map(motorLAccmEncoderCount, motorLDistInitPt+(motorLTravelDist*0.75), motorLDistInitPt+(motorLTravelDist*0.90), 255*(motorLTravelDist/1350.0), 46);
  //motorRSpeed = map(motorRAccmEncoderCount, motorRDistInitPt+(motorRTravelDist*0.75), motorRDistInitPt+(motorRTravelDist*0.90), 255*(motorRTravelDist/1200.0), 54);
  
  /*** PID that enforces left motor and right motor to travel the same amount of total distance ***/
  //motorLSpeed = map(motorLAccmEncoderCount, motorLDistInitPt+(motorTravelDist*0.75), motorLDistInitPt+(motorTravelDist*0.90), 255*(motorTravelDist/1350.0), 46);
  //motorRSpeed = map(motorRAccmEncoderCount, motorRDistInitPt+(motorTravelDist*0.75), motorRDistInitPt+(motorTravelDist*0.90), 255*(motorTravelDist/1200.0), 54);
  
  //motorLSpeed = constrain(motorLSpeed, 46, min(255, 255*(motorTravelDist/1350.0)));
  //motorRSpeed = constrain(motorRSpeed, 54, min(255, 255*(motorTravelDist/1200.0)));
  
  /*** PID that uses Arduino PID Library ***/
  //computePID();             //Set the PID tunings of both motors, to align the robot back to the straight line
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
