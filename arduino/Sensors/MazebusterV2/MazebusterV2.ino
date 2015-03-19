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
#define SENSOR_IR_RIGHT_TOP    A3
#define SENSOR_IR_RIGHT_MIDDLE A4
#define SENSOR_IR_LEFT_MIDDLE  A5

#define ULTRA_RPWM  6                                        // PWM Output 0-25000us,every 50us represent 1cm
#define ULTRA_RTRIG 7                                       // PWM trigger pin

uint8_t CMD_DISTANCE[]={0x44,0x22,0xbb,0x01};          // distance measure command

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
DistanceGP2Y0A21YK rightTop(3);
DistanceGP2Y0A21YK rightMiddle(4);
DistanceGP2Y0A21YK leftMiddle(5);

int  motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
volatile long  motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
int  motorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
volatile long  motorRNetEncoderCount = 0;       //Net encoder's ticks count of right motor
volatile int motorLOldA=0, motorROldA=0, motorLNewB=0, motorRNewB=0;

double  motorLPWM = 0;                   //PWM (0 - 255) of left motor
double  motorRPWM = 0;                   //PWM (0 - 255) of right motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
int numGrids = 1;

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

double FORWARD_PWM_L = 165.5;
double FORWARD_PWM_R = 162.5; // actual lab
//double FORWARD_PWM_R = 163; // student lounge
int FORWARD_DIST = 482*2;

double CCW_PWM_L = 180;
double CCW_PWM_R = 180;
//int CCW_DIST = 16.02/4.0/6.0*2294;  // actual lab
int CCW_DIST = 15.7/4.0/6.0*2294; // student lounge
double CW_PWM_L = 180;
double CW_PWM_R = 180;
//int CW_DIST = 15.99/4.0/6.0*2294; // actual lab
int CW_DIST = 16.3/4.0/6.0*2294; // student lounge
double BACK_PWM_L = 180;
double BACK_PWM_R = 180;
int BACK_DIST = CW_DIST*2;

//float FRONT_CUTOFF = 8.5; // actual lab
float FRONT_CUTOFF = 8; // student lounge
float RIGHT_TOP_OFFSET = 2.5; 
float LEFT_MIDDLE_OFFSET = 7.5; 

float CALIBRATION_THRESHOLD = 20;

String inputString = "";

boolean stringComplete = false;  // whether the string is complete
int lspeed = 0;
int rspeed = 0;
float sensorReadings[6] = {};
float previousLeftReading = 0;
float previousRightReading = 0;

int LMag = 1;
int RMag = -1;

void ultrasonicSetup(){ 
  pinMode(ULTRA_RTRIG,OUTPUT);                            // A low pull on pin COMP/TRIG
  digitalWrite(ULTRA_RTRIG,HIGH);                         // Set to HIGH
  
  pinMode(ULTRA_RPWM, INPUT);                             // Sending Enable PWM mode command
  
  for(int i=0;i<4;i++){
      Serial.write(CMD_DISTANCE[i]);
  } 
  Serial.println();  
}

void setup(){
  Serial.begin(115200);
  
  // inputString.reserve(20);
//  ultrasonicSetup();

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

unsigned long timer = 0;
unsigned long lastTime = 0;
unsigned int ultrasonicDistance = 0;

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
    Serial.println("p*");
    hasSent = true;
  }
}

void readUltrasonic(){                                     // a low pull on pin COMP/TRIG  triggering a sensor reading
  ultrasonicSetup();  
  md.setSpeeds(0, 0);
  delay(50);
  digitalWrite(ULTRA_RTRIG, LOW); 
  digitalWrite(ULTRA_RTRIG, HIGH);                      // reading Pin PWM will output pulses   
  unsigned long distanceMeasured = pulseIn(ULTRA_RPWM, LOW);

  if(distanceMeasured==50000){                     // the reading is invalid.
    ultrasonicDistance = -1;
  }
  else{
    ultrasonicDistance = distanceMeasured/50;                  // every 50us low level stands for 1cm
  }
  md.init();
}

void readAllSensors() {
  int l = mode == MODE_CALIBRATE ? 10 : 10;
//  if(mode == MODE_CALIBRATE && dir == RIGHT)
//    l = 1;
  do{
    readUltrasonic();
  }while(ultrasonicDistance == 0 || ultrasonicDistance > 500);
  for(int i=0; i<l; i++) {
    sensorReadings[0] = frontLeft.getDistanceMedian2();
    sensorReadings[1] = frontMiddle.getDistanceMedian2();
    sensorReadings[2] = frontRight.getDistanceMedian2();
    sensorReadings[3] = ultrasonicDistance;
    sensorReadings[4] = rightTop.getDistanceMedian2() - RIGHT_TOP_OFFSET;
    sensorReadings[5] = rightMiddle.getDistanceMedian2();
  }
}

void readSensorsTillStable(int i, int j){
  double prevI, prevJ;
  int count = 0;
//  do{
//    readUltrasonic();
//  }while(ultrasonicDistance == 0 || ultrasonicDistance > 500);
  do{
    prevI = sensorReadings[i];
    prevJ = sensorReadings[j];
    sensorReadings[0] = frontLeft.getDistanceMedianStable() + 0.02;
    sensorReadings[1] = frontMiddle.getDistanceMedianStable();
    sensorReadings[2] = frontRight.getDistanceMedianStable();
//    sensorReadings[3] = ultrasonicDistance;
    sensorReadings[4] = rightTop.getDistanceMedianStable() - RIGHT_TOP_OFFSET;
    sensorReadings[5] = rightMiddle.getDistanceMedianStable();
    count++;
    if(count > 30)
      break;
  }while(abs(sensorReadings[i] - prevI) >= 0.02 || abs(sensorReadings[j] - prevJ) >= 0.02);
}

void readAllSensors2() {
  int l = mode == MODE_CALIBRATE ? 10 : 10;
  for(int i=0; i<l; i++) {
    sensorReadings[0] = frontLeft.getDistanceMedian();
    sensorReadings[1] = frontMiddle.getDistanceMedian();
    sensorReadings[2] = frontRight.getDistanceMedian();
    sensorReadings[3] = leftMiddle.getDistanceMedian() - LEFT_MIDDLE_OFFSET;
    sensorReadings[4] = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
    sensorReadings[5] = rightMiddle.getDistanceMedian();
  }
}

void serialEvent() {  //Read inputs sent from Raspberry Pi via USB serial communication
  byte command;
  byte buf[2];
  Serial.flush();
  
  //When there's a data in the receiveing buffer, and both the motors have completed their moves
  if(Serial.available() == 2 && !motorLRun && !motorRRun) {
    int data1 = Serial.read();          //First byte of data is moving command
    int data2 = Serial.read();          //Second byte of data is distance
    
//    Serial.print("serial received: ");
//    Serial.print((char) data1);
//    Serial.println((char) data2);
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
      
      if(mode == MODE_CALIBRATE)
        sense = false;
    }
    
    if(data1 == 'x'){
      if(data2 == '1'){
        rotate(1, CW);
      }else{
        rotate(1, CCW);
      }
    }
    
    if(sense && dir == B00){
      replyWithSensorData();
      return;
    }
    
    if(data1 == 'k'){
      calibrateRotation(RIGHT);
    }
    
    if(data2 != -1 && dir == FORWARD) {
      command = byte(data2);
      numGrids = int(command & 0x1F);
//      Serial.print("num grids: ");
//      Serial.println(numGrids);
//      motorDistChkPt = int(command & 0x0F) * 2249 / 2 / (6*PI) * 10;  //Bits 3-6 represent number of grids required to move (1 grid = 10cm)
    }
    
      if(mode == MODE_EXPLORE){
        parseMove();
      }else if(mode == MODE_CALIBRATE){
        if(dir == FORWARD){
          calibrateDistance();
          calibrateRotation(FRONT);
          calibrateDistance();
          calibrateRotation(FRONT);
          delay(50);
        }else if(dir == CW){
//          calibrateRotation(RIGHT);
//          delay(300);
          calibrateRotation(RIGHT, true);
          delay(50);
        }
        hasSent = false;
        return;
      }
  }


}

void replyWithSensorData(){
  if(sense && !motorLRun && !motorRRun && mode != MODE_CALIBRATE){

    readAllSensors();
    Serial.print("p");
    for(int i=0; i<6; i++){
      Serial.print(sensorReadings[i]);
      Serial.print(" ");
    }
    Serial.println();

    sense = false;
//    hasSent = false;
  }
}

void parseMove(){
  if(!motorLRun && !motorRRun){
    resetMove();
    hasSent = false;
    sense = true;
    
    switch(dir){
      case FORWARD:
        straight(numGrids * 10);
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
  //if(dir == FORWARD){
  md.setBrakes(400, 400);
  //
  
//  Serial.print("Stop: L ");
//  Serial.print(motorLNetEncoderCount);
//  Serial.print(" R ");
//  Serial.println(motorRNetEncoderCount);
  
  motorRNetEncoderCount = 0;
  motorLNetEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
}

//void calibrateRotation(int side){
//  calibrateRotation(side, true);
//}

void calibrateRotation(int side, bool readAll){
  float L, R;

  if(side == FRONT){
    if(readAll)
      readSensorsTillStable(FRONT, FRONT + 2);
    L = sensorReadings[FRONT];
    R = sensorReadings[FRONT + 2];
//    Serial.print("p");
//    Serial.print(L);
//    Serial.print(" ");
//    Serial.println(R);
  }else if(side == RIGHT){
    if(readAll)
      readSensorsTillStable(RIGHT, RIGHT + 1);
    L = sensorReadings[RIGHT];
    R = sensorReadings[RIGHT + 1];
  }
  
  if((L+R)/2 > CALIBRATION_THRESHOLD)
    return;
  
//  Serial.print(sensorReadings[RIGHT]);
//  Serial.print(" ");
//  Serial.println(sensorReadings[RIGHT + 1]);
  
  float separation = 14.6;
  switch(side){
    case FRONT:
      separation = 14.6;
      break;
   case RIGHT:
      separation = 11;
  }
  
//  Serial.print(L);
//  Serial.print(" ");
//  Serial.println(R);
//  Serial.print(" ");
//  
  float tolerance = 0;
  float diff = side == FRONT ? L-R : L-R;
  if(abs(diff) > tolerance){
    float angle = abs(atan2(diff, separation) * 180 / M_PI);
    if(angle > 25)
      return;
//    Serial.println(angle);
    if(diff > tolerance){
      rotate(angle, CW);
    }else if(diff < -tolerance){
      rotate(angle, CCW);
    }
  }
}

void calibrateRotation(int side){
  calibrateRotation(side, true);
}

void calibrateRotation2(int side, bool readAll){
  float L = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
  float R = rightMiddle.getDistanceMedian();
  
  float separation = 14.6;
  switch(side){
    case FRONT:
      separation = 14.6;
      break;
   case RIGHT:
      separation = 11;
  }
  
  float tolerance = 0;
  
  int count = 0;
  while(abs(L-R) > 0.1 && count < 100){
    delay(50);
    L = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
    R = rightMiddle.getDistanceMedian();
    float diff = L - R;
    if(diff > tolerance){
      rotate(0.5, CW);
    }else if(diff < -tolerance){
      rotate(0.5, CCW);
    }
    count++;
  }
}

void calibrateDistance(){
  readSensorsTillStable(FRONT, FRONT + 2);
  float L = sensorReadings[FRONT];
  float R = sensorReadings[FRONT + 2];
  float avg = (L+R)/2;
  if(avg > CALIBRATION_THRESHOLD)
    return;
  float cutoff = FRONT_CUTOFF; 
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
//      Serial.print("L ");
//      Serial.println(motorLNetEncoderCount);
      robotStop();
      break;
    }
    
    if(motorRRun && abs(motorRNetEncoderCount) < setPoint){
      md.setM1Speed(rightPWM/255.0*400.0);
    }else {
//      Serial.print("R ");
//      Serial.println(motorRNetEncoderCount);
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

  motorDistChkPt = dist * 2249 / (6*PI);
  
  if(dist == 10){
    motorDistChkPt = dist * 2249 / (6*PI) / 1.07182; // actual lab
//    motorDistChkPt = dist * 2249 / (6*PI) / 1.055; // student lounge
  }
  

  
//  Serial.println(motorDistChkPt);

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void rotate(float angle, byte dir){
  resetMove();
  if(dir == CW){
//    Serial.println("CW");
    motorLPWM = -CW_PWM_L;
    motorRPWM = -CW_PWM_R;
    motorDistChkPt = CW_DIST*angle/90.0;
  }else{
//    Serial.println("CCW");
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
