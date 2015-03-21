#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <DistanceGP2Y0A21YK.h>
#include <digitalWriteFast.h>
#include <NewPing.h>

#define STUDENT_LOUNGE
#define DEBUG false
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
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

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

NewPing sonar(ULTRA_RTRIG, ULTRA_RPWM, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

DistanceGP2Y0A21YK frontMiddle(0);
DistanceGP2Y0A21YK frontLeft(1);
DistanceGP2Y0A21YK frontRight(2);
DistanceGP2Y0A21YK rightTop(3);
DistanceGP2Y0A21YK rightMiddle(4);
DistanceGP2Y0A21YK leftMiddle(5);

volatile int motorLOldA = 0, motorROldA = 0, motorLNewB = 0, motorRNewB = 0;

int motorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
int motorLPrevAccmEncoderCount = 0;
int motorLPrevNetEncoderCount = 0;
volatile int motorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
int motorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
int motorRPrevAccmEncoderCount = 0;
int motorRPrevNetEncoderCount = 0;
volatile int  motorRNetEncoderCount = 0;       //Net encoder's ticks count of right motor
double motorEncoderDiff = 0;

double  motorLPWM = 0;                   //PWM (0 - 255) of left motor
double  motorRPWM = 0;                   //PWM (0 - 255) of right motor
double  targetLPWM = 0;
double  targetRPWM = 0;
double  motorLSpeed = 0;                 //Rotation speed of left motor
double  targetLSpeed = 0;                //Target speed of left motor
double  motorRSpeed = 0;                 //Rotation speed of right motor
double  targetRSpeed = 0;                //Target speed of right motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
double  motorDiffOutput = 0;
double  motorTargetDiff = 0;

/************* Saved states *************/
int savedMotorLAccmEncoderCount = 0;      //Accumulated encoder's ticks count of left motor
int savedMotorLPrevAccmEncoderCount = 0;
int savedMotorLPrevNetEncoderCount = 0;
volatile int savedMotorLNetEncoderCount = 0;       //Net encoder's ticks count of left motor
int savedMotorRAccmEncoderCount = 0;      //Accumulated encoder's ticks count of right motor
int savedMotorRPrevAccmEncoderCount = 0;
int savedMotorRPrevNetEncoderCount = 0;
volatile int  savedMotorRNetEncoderCount = 0;       //Net encoder's ticks count of right motor

double  savedTargetLPWM = 0;
double  savedTargetRPWM = 0;
/******************** END ********************/

int numGrids = 1;

unsigned long prevTime = 0;              //Previous timestamp checked for motors' speeds
unsigned long currTime = 0;              //Current timestamp

PID     motorLPID(&motorLSpeed, &motorLPWM, &targetLSpeed, 150, 0, 0, DIRECT);  //PID that controls the PWM of left motor
PID     motorRPID(&motorRSpeed, &motorRPWM, &targetRSpeed, 150, 0, 0, DIRECT);  //PID that controls the PWM of right motor
PID     motorDiffPID(&motorEncoderDiff, &motorDiffOutput, &motorTargetDiff, 1.7, 0.3, 0.7, DIRECT);  //PID that controls the PWM of right motor

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

double FORWARD_PWM_L = 165.5 * 1.2;
//double FORWARD_PWM_R = 161.8; // actual lab
double FORWARD_PWM_R = 159.6 * 1.2; // student lounge
double FORWARD_DIST_SHORT_FACTOR = 1.073;
double FORWARD_DIST = 2249 / (6*PI);

double CCW_RADIUS = 16.00;
double CCW_PWM_L = 180 * 1.1;
double CCW_PWM_R = 180 * 1.1;
//int CCW_DIST = 16.15/4.0/6.0*2294;  // actual lab
int CCW_DIST = CCW_RADIUS/4.0/6.0*2294; 
// int CCW_DIST = 16.15/4.0/6.0*2294; // student lounge
double CW_RADIUS = 16.00;
double CW_PWM_L = 180 * 1.1;
double CW_PWM_R = 180 * 1.1;
//int CW_DIST = 16.40/4.0/6.0*2294; // actual lab
int CW_DIST = CW_RADIUS/4.0/6.0*2294; 
// int CW_DIST = 16.81/4.0/6.0*2294; // student lounge
double BACK_PWM_L = 180 * 1.1;
double BACK_PWM_R = 180 * 1.1;
int BACK_DIST = 16.55/2.0/6.0*2294;

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

  motorLPID.SetMode(AUTOMATIC);
  motorRPID.SetMode(AUTOMATIC);
  motorDiffPID.SetMode(AUTOMATIC);
  motorLPID.SetOutputLimits(80, 255);
  motorRPID.SetOutputLimits(82, 255);
  motorDiffPID.SetOutputLimits(-200, 200);
  motorLPID.SetSampleTime(10);
  motorRPID.SetSampleTime(10);
  motorDiffPID.SetSampleTime(10);
  
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
  
  if(!hasSent && !motorLRun && !motorRRun){
    Serial.println("p*");
    hasSent = true;
  }
}

void readUltrasonic(){                                     // a low pull on pin COMP/TRIG  triggering a sensor reading
  // ultrasonicSetup();  
  // delay(50);
  // digitalWrite(ULTRA_RTRIG, LOW); 
  // digitalWrite(ULTRA_RTRIG, HIGH);                      // reading Pin PWM will output pulses   
  // unsigned long distanceMeasured = pulseIn(ULTRA_RPWM, LOW);

  // if(distanceMeasured==50000){                     // the reading is invalid.
  //   ultrasonicDistance = -1;
  // }
  // else{
  //   ultrasonicDistance = distanceMeasured/50;                  // every 50us low level stands for 1cm
  // }
  // md.init();
  md.setSpeeds(0, 0);
  delay(20);
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  ultrasonicDistance = uS / US_ROUNDTRIP_CM;
#if DEBUG
  Serial.print("Ping: ");
  Serial.print(uS /(double) US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
#endif
  md.init();
}

void readAllSensors() {
  int l = mode == MODE_CALIBRATE ? 2 : 2;
//  if(mode == MODE_CALIBRATE && dir == RIGHT)
//    l = 1;
#if DEBUG
  Serial.print("p");
  Serial.println(millis());
#endif

  do{
    readUltrasonic();
  }while(ultrasonicDistance == 0 || ultrasonicDistance == MAX_DISTANCE);
  for(int i=0; i<l; i++) {
    sensorReadings[0] = frontLeft.getDistanceCm();
    sensorReadings[1] = frontMiddle.getDistanceCm();
    sensorReadings[2] = frontRight.getDistanceCm();
    sensorReadings[3] = ultrasonicDistance;
    sensorReadings[4] = rightTop.getDistanceCm();
    sensorReadings[5] = rightMiddle.getDistanceCm();

//    sensorReadings[0] = frontLeft.getDistanceMedian2();
//    sensorReadings[1] = frontMiddle.getDistanceMedian2();
//    sensorReadings[2] = frontRight.getDistanceMedian2();
//    sensorReadings[3] = leftMiddle.getDistanceMedian2() - LEFT_MIDDLE_OFFSET;
//    sensorReadings[4] = rightTop.getDistanceMedian2() - RIGHT_TOP_OFFSET;
//    sensorReadings[5] = rightMiddle.getDistanceMedian2();
  }

#if DEBUG
  Serial.print("p");
  Serial.println(millis());
#endif
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
    if(count > 10)
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
//  Serial.flush();

  //When there's a data in the receiveing buffer, and both the motors have completed their moves
  if(Serial.available() >= 2 && !motorLRun && !motorRRun) {
    int data1 = Serial.read();          //First byte of data is moving command
    int data2 = Serial.read();          //Second byte of data is distance
    
//    Serial.print("pserial received: ");
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

    #if DEBUG
      if(data1 == 'x'){
        if(data2 == '1'){
          rotate(1, CW);
        }else{
          rotate(1, CCW);
        }
      }

      if(data1 == '1'){
        if(data2 == '1'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp() + 0.1, motorDiffPID.GetKi(), motorDiffPID.GetKd());
        }else if(data2 == '2'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp() - 0.1, motorDiffPID.GetKi(), motorDiffPID.GetKd());
        }
        return;
      }else if(data1 == '2'){
        if(data2 == '1'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp(), motorDiffPID.GetKi() + 0.1, motorDiffPID.GetKd());
        }else if(data2 == '2'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp(), motorDiffPID.GetKi() - 0.1, motorDiffPID.GetKd());
        }
        return;
      }else if(data1 == '3'){
        if(data2 == '1'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp(), motorDiffPID.GetKi(), motorDiffPID.GetKd() + 0.1);
        }else if(data2 == '2'){
          motorDiffPID.SetTunings(motorDiffPID.GetKp(), motorDiffPID.GetKi(), motorDiffPID.GetKd() - 0.1);
        }
        return;
      }else if(data1 == '4'){
        Serial.print("pPID: ");
        Serial.print(motorDiffPID.GetKp());
        Serial.print(" ");
        Serial.print(motorDiffPID.GetKi());
        Serial.print(" ");
        Serial.println(motorDiffPID.GetKd());
        return;
      }else if(data1 == '5'){
        if(data2 == '1'){
          CW_RADIUS += 0.01;
        }else if(data2 == '2'){
          CW_RADIUS -= 0.01;
        }else if(data2 == '3'){
          CW_RADIUS += 0.1;
        }else if(data2 == '4'){
          CW_RADIUS -= 0.1;
        }
        return;
      }else if(data1 == '6'){
        if(data2 == '1'){
          CCW_RADIUS += 0.01;
        }else if(data2 == '2'){
          CCW_RADIUS -= 0.01;
        }else if(data2 == '3'){
          CCW_RADIUS += 0.1;
        }else if(data2 == '4'){
          CCW_RADIUS -= 0.1;
        }
        return;
      }else if(data1 == '7'){
        Serial.print("pRadius: ");
        Serial.print(CW_RADIUS);
        Serial.print(" ");
        Serial.println(CCW_RADIUS);
        return;
      }else if(data1 == '8'){
        if(data2 == '1'){
          FORWARD_DIST_SHORT_FACTOR += 0.001;
        }else if(data2 == '2'){
          FORWARD_DIST_SHORT_FACTOR -= 0.001;
        }
        return;
      }else if(data1 == '9'){
        Serial.print("pForward factor: ");
        Serial.println(FORWARD_DIST_SHORT_FACTOR);
        return;
      }
    #endif
    
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
          delay(100);
        }else if(dir == CW){
//          calibrateRotation(RIGHT);
//          delay(300);
          calibrateRotation(RIGHT, true);
          delay(100);
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
  md.setBrakes(400, 400);
  
  motorLAccmEncoderCount = 0;
  motorRAccmEncoderCount = 0;
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;

  motorRNetEncoderCount = 0;
  motorLNetEncoderCount = 0;
  motorLRun = false;
  motorRRun = false;
}

void motorLStop(){
  if(dir == FORWARD)
    md.setM2Brake(300);
  else
    md.setM2Brake(400);
  motorLRun = false;
}

void motorRStop(){
  if(dir == FORWARD)
    md.setM1Brake(300);
  else
    md.setM1Brake(400);
  motorRRun = false;
}

bool emergencyStopIfNeeded(){
  float FL = frontLeft.getDistanceCm();
  float FM = frontMiddle.getDistanceCm();
  float FR = frontRight.getDistanceCm();

  if(min(min(FL, FM), FR) < 0.2){
    robotStop();
    return true;
  }

  return false;
}



void saveMotorState(){
  savedMotorLAccmEncoderCount     = motorLAccmEncoderCount;
  savedMotorLPrevAccmEncoderCount = motorLPrevAccmEncoderCount;
  savedMotorLPrevNetEncoderCount  = motorLPrevNetEncoderCount;
  savedMotorLNetEncoderCount      = motorLNetEncoderCount;
  savedMotorRAccmEncoderCount     = motorRAccmEncoderCount;
  savedMotorRPrevAccmEncoderCount = motorRPrevAccmEncoderCount;
  savedMotorRPrevNetEncoderCount  = motorRPrevNetEncoderCount;
  savedMotorRNetEncoderCount      = motorRNetEncoderCount;
  savedTargetLPWM                 = targetLPWM;
  savedTargetRPWM                 = targetRPWM;
}

void restoreMotorState(){
  motorLAccmEncoderCount     =  savedMotorLAccmEncoderCount;
  motorLPrevAccmEncoderCount =  savedMotorLPrevAccmEncoderCount;
  motorLPrevNetEncoderCount  =  savedMotorLPrevNetEncoderCount;
  motorLNetEncoderCount      =  savedMotorLNetEncoderCount;
  motorRAccmEncoderCount     =  savedMotorRAccmEncoderCount;
  motorRPrevAccmEncoderCount =  savedMotorRPrevAccmEncoderCount;
  motorRPrevNetEncoderCount  =  savedMotorRPrevNetEncoderCount;
  motorRNetEncoderCount      =  savedMotorRNetEncoderCount;
  targetLPWM                 = savedTargetLPWM;
  targetRPWM                 = savedTargetRPWM;
}

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
      separation = 17.6;
      break;
   case RIGHT:
      separation = 8.3;
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
  motorLPrevAccmEncoderCount = 0;
  motorRPrevAccmEncoderCount = 0;
  motorRNetEncoderCount = 0;
  motorLNetEncoderCount = 0;
  motorLPrevNetEncoderCount = 0;
  motorRPrevNetEncoderCount = 0; 
  motorLRun = true;
  motorRRun = true;
  motorLPID.Reset();
  motorRPID.Reset();
  motorDiffPID.Reset();
}

void computePID(){
  motorLPID.Compute();  //Compute left motor PWM based on the left motor speed w.r.t the target speed
  motorRPID.Compute();  //Compute right motor PWM based on the right motor speed w.r.t the target speed
}

void move(float finalLPWM, float finalRPWM, int setPoint){
  resetMove();
  // double finalLSpeed = 2.5;
  // double finalRSpeed = 2.5;
  // targetLSpeed = 0;
  // targetRSpeed = 0;
  double MIN_SPEED = 100;
  double MIN_DISTANCE = 2249 / (6*PI) / 1.073;

  float k1 = finalLPWM / min(MIN_DISTANCE, (1.0 * setPoint));
  float k2 = finalRPWM / min(MIN_DISTANCE, (1.0 * setPoint));

  targetLPWM = MIN_SPEED;
  targetRPWM = MIN_SPEED;

  // float constant = 1 - exp(k);

  // double deltaLPWM = (constant);
  // double deltaRPWM = (constant);

  int count = 0;
  double motorLDiff = 0, motorRDiff = 0;

  while(true){

    // if (mode != MODE_CALIBRATE && dir == FORWARD && emergencyStopIfNeeded()){
    //   Serial.println("pEmergency");
    //   return;
    // }

    motorEncoderDiff = abs(motorLAccmEncoderCount) - abs(motorRAccmEncoderCount);
    motorLDiff = abs(motorLAccmEncoderCount - motorLPrevAccmEncoderCount);
    motorRDiff = abs(motorRAccmEncoderCount - motorRPrevAccmEncoderCount);
    currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.
    motorLSpeed = motorLDiff / (currTime - prevTime);
    motorRSpeed = motorRDiff / (currTime - prevTime);
    
    // targetLPWM = constrain(targetLPWM, 0, finalLPWM);
    // targetRPWM = constrain(targetRPWM, 0, finalRPWM);

    delay(10);

    // Serial.print("pTime: ");
    // Serial.print(currTime);
    // Serial.print(" ");
    // Serial.println(prevTime);
    // Serial.print("pEncoder: ");
    // Serial.print(motorLAccmEncoderCount);
    // Serial.print(" ");
    // Serial.println(motorRAccmEncoderCount);

 

    // Serial.print("pSpeed: ");
    // Serial.print(motorLSpeed);
    // Serial.print(" ");
    // Serial.println(motorRSpeed);

    // Serial.print("pLEncoder: ");
    // Serial.print(motorLPrevNetEncoderCount);
    // Serial.print(" ");
    // Serial.println(motorLNetEncoderCount);

    // Serial.print("pREncoder: ");
    // Serial.print(motorRPrevNetEncoderCount);
    // Serial.print(" ");
    // Serial.println(motorRNetEncoderCount);
    // computePID();

    motorDiffPID.Compute();

    // Serial.print("pmotorDiffOutput: ");
    // Serial.println(motorDiffOutput);

    // if(motorEncoderDiff > 0){
    //   motorLPWM -= motorDiffPWM;
    //   motorRPWM += motorDiffPWM;
    // }else{
    //   motorLPWM += motorDiffPWM;
    //   motorRPWM -= motorDiffPWM;
    // }

    float offset = DistanceGP2Y0A21YK::mapf((float)motorDiffOutput, -200, 200, -10, 10);

  #if DEBUG
    Serial.print("pCount: ");
    Serial.print(count);
    Serial.print(" Diff: ");
    Serial.print(motorEncoderDiff);
    Serial.print(" Offset: ");
    Serial.println(offset);
  #endif

    if(offset > 0){
      if(offset + targetLPWM <= finalLPWM){
        targetLPWM += offset;
      }else if(targetRPWM - offset > MIN_SPEED){
        targetRPWM -= offset;
      }else{
        double avg = (targetLPWM + targetRPWM)/2;
        targetLPWM = avg;
        targetRPWM = avg;
      }
    }else{
      if(targetRPWM - offset <= finalRPWM){
        targetRPWM -= offset;
      }else if(targetLPWM + offset > MIN_SPEED){
        targetLPWM += offset;
      }else{
        double avg = (targetLPWM + targetRPWM)/2;
        targetLPWM = avg;
        targetRPWM = avg;
      }
    }

    targetLPWM = constrain(targetLPWM, 0, 255);
    targetRPWM = constrain(targetRPWM, 0, 255);

    motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
    motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
    prevTime = currTime;

  #if DEBUG
    Serial.print("pPWM: ");
    Serial.print(targetLPWM);
    Serial.print(" ");
    Serial.println(targetRPWM);
  #endif

    if(motorLRun && abs(motorLAccmEncoderCount) < setPoint){
      md.setM2Speed(LMag*targetLPWM/255.0*400.0);
    }else {
      motorLStop();
    }
    
    if(motorRRun && abs(motorRAccmEncoderCount) < setPoint){
      md.setM1Speed(RMag*targetRPWM/255.0*400.0);
    }else {
      motorRStop();
    }

    if(!motorLRun && !motorRRun)
      break;

    double LIncrement = motorLDiff * k1;
    double RIncrement = motorRDiff * k2;
    double avg = (LIncrement + RIncrement)/2;

    if(avg + targetLPWM < finalLPWM){
      targetLPWM += avg;
    }

    if(avg + targetRPWM < finalRPWM){
      targetRPWM += avg;
    }

    // Serial.print("pIncrement: ");
    // Serial.print(motorLDiff * k1);
    // Serial.print(" ");
    // Serial.println(motorRDiff * k2);

    if(motorLAccmEncoderCount * k1 > MIN_SPEED){
      targetLPWM = constrain(targetLPWM, 0, targetLPWM);
    }else{
      targetLPWM = constrain(targetLPWM, MIN_SPEED, targetLPWM);
    }

    if(motorRAccmEncoderCount * k2 > MIN_SPEED){
      targetRPWM = constrain(targetRPWM, 0, targetRPWM);
    }else{
      targetRPWM = constrain(targetRPWM, MIN_SPEED, targetRPWM);
    }

    count++;
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
  // motorLPWM = LMag * FORWARD_PWM_L;
  // motorRPWM = RMag * FORWARD_PWM_R;

  motorDistChkPt = dist * FORWARD_DIST;
  
  if(dist == 10){
    motorDistChkPt = dist * FORWARD_DIST / FORWARD_DIST_SHORT_FACTOR; // actual lab

//    motorDistChkPt = dist * 2249 / (6*PI) / 1.055; // student lounge
  }

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void rotate(float angle, byte dir){
  resetMove();
  if(dir == CW){
    LMag = -1;
    RMag = -1;
    motorLPWM = CW_PWM_L;
    motorRPWM = CW_PWM_R;
    motorDistChkPt = CW_DIST*angle/90.0;
  }else{
    LMag = 1;
    RMag = 1;
    motorLPWM = CCW_PWM_L;
    motorRPWM = CCW_PWM_R;
    motorDistChkPt = CCW_DIST*angle/90.0;
  }

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void back(){
  resetMove();
  LMag = 1;
  RMag = 1;
  motorLPWM = BACK_PWM_L;
  motorRPWM = BACK_PWM_R;
  motorDistChkPt = BACK_DIST;

  move(motorLPWM, motorRPWM, motorDistChkPt);
}

void motorLISRA() {    //ISR for left motor encoder interrupt
  motorLAccmEncoderCount++;
  motorLNewB ^ motorLOldA ? motorLNetEncoderCount++: motorLNetEncoderCount--;
  motorLOldA = digitalReadFast(MOTOR_L_ENCODER_A);
}

void motorLISRB() {    //ISR for left motor encoder interrupt
  motorLAccmEncoderCount++;
  motorLNewB = digitalReadFast(MOTOR_L_ENCODER_B);
  motorLNewB ^ motorLOldA ? motorLNetEncoderCount++ : motorLNetEncoderCount--;
}

void motorRISRA() {    //ISR for left motor encoder interrupt
  motorRAccmEncoderCount++;
  motorRNewB ^ motorROldA ? motorRNetEncoderCount--: motorRNetEncoderCount++;
  motorROldA = digitalReadFast(MOTOR_R_ENCODER_A);
}

void motorRISRB() {    //ISR for left motor encoder interrupt
  motorRAccmEncoderCount++;
  motorRNewB = digitalReadFast(MOTOR_R_ENCODER_B);
  motorRNewB ^ motorROldA ? motorRNetEncoderCount-- : motorRNetEncoderCount++;
}
