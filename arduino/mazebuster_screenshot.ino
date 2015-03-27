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
uint8_t CMD_DISTANCE[] = {0x44, 0x22, 0xbb, 0x01};     // distance measure command
#define FORWARD B11
#define CW      B10
#define CCW     B01
#define BACK    B00
#define FRONT 0
#define RIGHT 4
#define DELAY_PERIOD 200
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
double  currentLPWM = 0;
double  currentRPWM = 0;
double  targetLPWM = 0;
double  targetRPWM = 0;
double  motorLSpeed = 0;                 //Rotation speed of left motor
double  targetLSpeed = 0;                //Target speed of left motor
double  motorRSpeed = 0;                 //Rotation speed of right motor
double  targetRSpeed = 0;                //Target speed of right motor
double  motorDistChkPt = 100;              //Final estimated travelled distance/encoder check point of both motors
double  motorDiffOutput = 0;
double  motorTargetDiff = 0;
double  outputLPWM = 0;
double  outputRPWM = 0;
double  motorLPWMdiff = 0;
double  motorRPWMdiff = 0;
int numGrids = 1;
unsigned long prevTime = 0;              //Previous timestamp checked for motors' speeds
unsigned long currTime = 0;              //Current timestamp
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
double FORWARD_PWM_R = 161 * 1.2; // student lounge
double FORWARD_DIST_SHORT_FACTOR = 1.15;
double FORWARD_DIST = 2249 / (6 * PI);
double CCW_RADIUS = 15.30;
double CCW_PWM_L = 180 * 1.1;
double CCW_PWM_R = 180 * 1.1;
int CCW_DIST = CCW_RADIUS / 4.0 / 6.0 * 2294;
double CW_RADIUS = 15.50;
double CW_PWM_L = 180;
double CW_PWM_R = 180;
int CW_DIST = CW_RADIUS / 4.0 / 6.0 * 2294;
double BACK_PWM_L = 180 * 1.1;
double BACK_PWM_R = 180 * 1.1;
int BACK_DIST = 16.20 / 2.0 / 6.0 * 2294;
float FRONT_CUTOFF = 8; // student lounge
float RIGHT_TOP_OFFSET = 2.5;
float LEFT_MIDDLE_OFFSET = 7.5;
float CALIBRATION_THRESHOLD = 20;
float sensorReadings[6] = {};
int LMag = 1;
int RMag = -1;

void ultrasonicSetup()
{
    pinMode(ULTRA_RTRIG, OUTPUT);                           // A low pull on pin COMP/TRIG
    digitalWrite(ULTRA_RTRIG, HIGH);                        // Set to HIGH
    pinMode(ULTRA_RPWM, INPUT);                             // Sending Enable PWM mode command
    for (int i = 0; i < 4; i++)
    {
        Serial.write(CMD_DISTANCE[i]);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
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
void loop()
{
}

void sendFeedback()
{
    Serial.println("p*");
}

void readUltrasonic()                                      // a low pull on pin COMP/TRIG  triggering a sensor reading
{
    md.setSpeeds(0, 0);
    delay(20);
    unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    ultrasonicDistance = uS / US_ROUNDTRIP_CM;
#if DEBUG
#endif
    md.init();
}

void readAllSensors()
{
    int l = mode == MODE_CALIBRATE ? 2 : 2;
    do
    {
        readUltrasonic();
    }
    while (ultrasonicDistance == 0 || ultrasonicDistance == MAX_DISTANCE);
    for (int i = 0; i < l; i++)
    {
        sensorReadings[0] = frontLeft.getDistanceCm() - 2;
        sensorReadings[1] = frontMiddle.getDistanceCm();
        sensorReadings[2] = frontRight.getDistanceCm();
        sensorReadings[3] = rightMiddle.getDistanceCm();
        sensorReadings[4] = rightTop.getDistanceCm();
        sensorReadings[5] = ultrasonicDistance;
    }
#if DEBUG
#endif
}


void readSensorsTillStable(int i, int j)
{
    frontLeft.resetSmoothing();
    frontRight.resetSmoothing();
    double prevI, prevJ;
    int count = 0;
    do
    {
        prevI = sensorReadings[i];
        prevJ = sensorReadings[j];
        sensorReadings[0] = frontLeft.getDistance2();
        sensorReadings[2] = frontRight.getDistance2();
        count++;
        if (count > 10)
            break;
    }
    while (abs(sensorReadings[i] - prevI) >= 0.02 || abs(sensorReadings[j] - prevJ) >= 0.02);
}


void readAllSensors2()
{
    int l = mode == MODE_CALIBRATE ? 10 : 10;
    for (int i = 0; i < l; i++)
    {
        sensorReadings[0] = frontLeft.getDistanceMedian();
        sensorReadings[1] = frontMiddle.getDistanceMedian();
        sensorReadings[2] = frontRight.getDistanceMedian();
        sensorReadings[3] = leftMiddle.getDistanceMedian() - LEFT_MIDDLE_OFFSET;
        sensorReadings[4] = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
        sensorReadings[5] = rightMiddle.getDistanceMedian();
    }
}

float kp = 1.7, ki = 0.3, kd = 0.7;
void serialEvent()    //Read inputs sent from Raspberry Pi via USB serial communication
{
    byte command;
    if (Serial.available() >= 2 && !motorLRun && !motorRRun)
    {
        int data1 = Serial.read();          //First byte of data is moving command
        int data2 = Serial.read();          //Second byte of data is distance
        if (((char) data1) == '*')
        {
            sendFeedback();
            return;
        }
        if (((char) data1) == '@' && ((char) data2) == '@')
        {
            replyWithSensorData();
            return;
        }
        if (data1 != -1)                    //If received a command, decode the command
        {
            command = byte(data1);
            mode = (command & 0x0C) >> 2;         //Bit 0-1 represents start or stop
            dir = (command & 0x03);    //Bits 2-3 represent direction to move
        }
        if (data2 != -1 && dir == FORWARD)
        {
            command = byte(data2);
            numGrids = int(command & 0x1F);
        }
        if (mode == MODE_EXPLORE)
        {
            parseMove();
        }
        else if (mode == MODE_CALIBRATE)
        {
            if (dir == FORWARD)
            {
                calibrateDistance(CALIBRATION_THRESHOLD);
                calibrateRotation(FRONT);
                delay(300);
            }
            else if (dir == CW)
            {
                calibrateRotation(RIGHT);
                delay(300);
            }
            sendFeedback();
        }
        else if (mode == MODE_SHORTESTPATH)
        {
            if (dir == FORWARD)
            {
                calibrateDistance(100);
            }
            sendFeedback();
        }
    }
}

void replyWithSensorData()
{
    if (!motorLRun && !motorRRun && mode != MODE_CALIBRATE)
    {
        readAllSensors();
        Serial.print("p");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(sensorReadings[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void parseMove()
{
    if (!motorLRun && !motorRRun)
    {
        resetMove();
        sense = true;
        switch (dir)
        {
        case FORWARD:
            straight(numGrids * 10, 400, true, 1);
            break;
        case CCW:
            rotate(90, CCW, 400, 1);
            break;
        case CW:
            rotate(90, CW, 400, 1);
            break;
        case BACK:
            back();
        }
    }
    delay(DELAY_PERIOD);
    sendFeedback();
}

void robotStop()
{
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

void motorLStop(int brakeLevel = 400)
{
    if (!brakeLevel)
        md.setM2Speed(0);
    else
        md.setM2Brake(brakeLevel);
    motorLRun = false;
}

void motorRStop(int brakeLevel = 400)
{
    if (!brakeLevel)
        md.setM1Speed(0);
    else
        md.setM1Brake(brakeLevel);
    motorRRun = false;
}

bool emergencyStopIfNeeded()
{
    float FL = frontLeft.getDistanceCm();
    float FM = frontMiddle.getDistanceCm();
    float FR = frontRight.getDistanceCm();
    if (min(min(FL, FM), FR) < 0.2)
    {
        robotStop();
        return true;
    }
    return false;
}

void calibrateRotation(int side, bool readAll)
{
    float L, R;
    if (side == FRONT)
    {
        if (readAll)
            readSensorsTillStable(FRONT, FRONT + 2);
        L = sensorReadings[FRONT];
        R = sensorReadings[FRONT + 2];
        Serial.print("Calibrate ");
        Serial.print(L);
        Serial.print(" ");
        Serial.println(R);
    }
    else if (side == RIGHT)
    {
        if (readAll)
            readSensorsTillStable(RIGHT, RIGHT + 1);
        L = sensorReadings[RIGHT];
        R = sensorReadings[RIGHT + 1];
    }
    if ((L + R) / 2 > CALIBRATION_THRESHOLD)
        return;
    float separation = 14.6;
    switch (side)
    {
    case FRONT:
        separation = 17.6;
        break;
    case RIGHT:
        separation = 8.3;
    }
    //
    float tolerance = 0;
    float diff = side == FRONT ? L - R : L - R;
    if (abs(diff) > tolerance)
    {
        float angle = abs(atan2(diff, separation) * 180 / M_PI) * 0.995;
        Serial.print("angle: ");
        Serial.println(angle);
        if (angle > 25)
            return;
        if (diff > tolerance)
        {
            rotate(angle, CW, 400, 0.7);
        }
        else if (diff < -tolerance)
        {
            rotate(angle, CCW, 400, 0.7);
        }
    }
}

void calibrateRotation(int side)
{
    calibrateRotation(side, true);
}

void calibrateRotation2(int side, bool readAll)
{
    float L = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
    float R = rightMiddle.getDistanceMedian();
    float separation = 14.6;
    switch (side)
    {
    case FRONT:
        separation = 14.6;
        break;
    case RIGHT:
        separation = 11;
    }
    float tolerance = 0;
    int count = 0;
    while (abs(L - R) > 0.1 && count < 100)
    {
        delay(50);
        L = rightTop.getDistanceMedian() - RIGHT_TOP_OFFSET;
        R = rightMiddle.getDistanceMedian();
        float diff = L - R;
        if (diff > tolerance)
        {
            rotate(0.5, CW, 400, 1);
        }
        else if (diff < -tolerance)
        {
            rotate(0.5, CCW, 400, 1);
        }
        count++;
    }
}

void calibrateDistance(float threshold)
{
    readSensorsTillStable(FRONT, FRONT + 2);
    float L = sensorReadings[FRONT];
    float R = sensorReadings[FRONT + 2];
    float avg = min(L, R);
    if (avg > threshold)
        return;
    float tolerance = 0.1;
    float cutoff = FRONT_CUTOFF;
    if (abs(avg - cutoff) > tolerance)
    {
        straight(-0.8, 400, false, 0.5);
        float frontDiff = avg - cutoff + 1;
        if (frontDiff > tolerance)
        {
            straight(frontDiff, 400, false, 0.5);
        }
        else if (frontDiff < -tolerance)
        {
            straight(frontDiff, 400, false, 0.5);
        }
    }
}

double x = 0, y = 0;
double CONVERT_LEFT = 6.0 * PI / 2249;
double CONVERT_RIGHT = 6.0 * PI / 2249;
double motorLDiffCm = 0, motorRDiffCm = 0, motorAvgDiffCm = 0;
double deltaAngleRad = 0, currentAngleRad = 0;
double WHEELBASE_DIAMETER = 16.0;
void resetMove()
{
    motorDiffOutput = 0;
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
}

void moveWithoutPid(float finalLPWM, float finalRPWM, int setPoint, int brakeLevel = 400)
{
    resetMove();
    targetLPWM = finalLPWM;
    targetRPWM = finalRPWM;
    int count = 0;
    int motorLDiff = 0, motorRDiff = 0;
    while (true)
    {
        motorEncoderDiff = motorLAccmEncoderCount - motorRAccmEncoderCount;
        motorLDiff = motorLAccmEncoderCount - motorLPrevAccmEncoderCount;
        motorRDiff = motorRAccmEncoderCount - motorRPrevAccmEncoderCount;
        currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.
        motorLSpeed = motorLDiff / (currTime - prevTime);
        motorRSpeed = motorRDiff / (currTime - prevTime);

        motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
        motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
        prevTime = currTime;
        targetLPWM = constrain(targetLPWM, 0, 255);
        targetRPWM = constrain(targetRPWM, 0, 255);
        if (motorLRun && abs(motorLAccmEncoderCount) < setPoint)
        {
            md.setM2Speed(LMag * targetLPWM / 255.0 * 400.0);
        }
        else
        {
            motorLStop(brakeLevel);
        }
        if (motorRRun && abs(motorRAccmEncoderCount) < setPoint)
        {
            md.setM1Speed(RMag * targetRPWM / 255.0 * 400.0);
        }
        else
        {
            motorRStop(brakeLevel);
        }
        if (!motorLRun && !motorRRun)
        {
            break;
        }
    }
}

PID motorDiffPID(&motorEncoderDiff, &motorDiffOutput, &motorTargetDiff, 1.7, 0.3, 0.7, DIRECT);  //PID that controls the PWM of right motor
void go(float finalLPWM, float finalRPWM, int setPoint, int brakeLevel)
{
    motorDiffPID.Reset();
    motorDiffPID.SetMode(AUTOMATIC);
    motorDiffPID.SetOutputLimits(-2000, 2000);
    motorDiffPID.SetSampleTime(10);// double finalLSpeed = 2.5;
    motorDiffPID.SetTunings(kp, ki, kd);
    double MIN_SPEED = 100;
    double MIN_DISTANCE = 2249 / (6 * PI) / 1.073;
    int BAND_HEIGHT = 20;
    bool pastLowerBandL = false;
    bool pastLowerBandR = false;
    float s1 = 0.2, s2 = 0.2;
    float k1 = finalLPWM / min(MIN_DISTANCE, (s1 * setPoint));
    float k2 = finalRPWM / min(MIN_DISTANCE, (s2 * setPoint));
    targetLPWM = finalLPWM;
    targetRPWM = finalRPWM;
    int count = 0;
    int motorLDiff = 0, motorRDiff = 0;
    while (true)
    {
        motorEncoderDiff = motorLAccmEncoderCount - motorRAccmEncoderCount;
        motorLDiff = motorLAccmEncoderCount - motorLPrevAccmEncoderCount;
        motorRDiff = motorRAccmEncoderCount - motorRPrevAccmEncoderCount;
        currTime = millis() + 1;  //Current time since the start of execution. Plus 1 to avoid divide by 0.
        motorLSpeed = motorLDiff / (currTime - prevTime);
        motorRSpeed = motorRDiff / (currTime - prevTime);
        motorLPrevAccmEncoderCount = motorLAccmEncoderCount;
        motorRPrevAccmEncoderCount = motorRAccmEncoderCount;
        prevTime = currTime;
        motorDiffPID.Compute();
        if (pastLowerBandL || pastLowerBandR)
        {
            targetLPWM += motorDiffOutput / 50;
            targetRPWM -= motorDiffOutput / 50;
        }
        targetLPWM = constrain(targetLPWM, MIN_SPEED, 255);
        targetRPWM = constrain(targetRPWM, MIN_SPEED, 255);
        if (!pastLowerBandL && targetLPWM >= finalLPWM - BAND_HEIGHT)
            pastLowerBandL = true;
        if (!pastLowerBandR && targetRPWM >= finalRPWM - BAND_HEIGHT)
            pastLowerBandR = true;
        if (motorLRun && abs(motorLAccmEncoderCount) < setPoint)
        {
            md.setM2Speed(LMag * targetLPWM / 255.0 * 400.0);
        }
        else
        {
            motorLStop(brakeLevel);
        }
        if (motorRRun && abs(motorRAccmEncoderCount) < setPoint)
        {
            md.setM1Speed(RMag * targetRPWM / 255.0 * 400.0);
        }
        else
        {
            motorRStop(brakeLevel);
        }
        if (!motorLRun && !motorRRun)
        {
            break;
        }

        count++;
        delay(10);
    }
}

void straight(float dist, int brakeLevel, bool withPid, float speedRatio)
{
    resetMove();
    motorLPWM = FORWARD_PWM_L * speedRatio;
    motorRPWM = FORWARD_PWM_R * speedRatio;
    if (dist > 0)
    {
        LMag = -1;
        RMag = 1;
    }
    else
    {
        dist = -dist;
        LMag = 1;
        RMag = -1;
    }
    motorDistChkPt = dist * FORWARD_DIST;
    if (dist == 10)
    {
        motorDistChkPt = dist * FORWARD_DIST / FORWARD_DIST_SHORT_FACTOR; // actual lab
    }
    if (withPid)
        go(motorLPWM, motorRPWM, motorDistChkPt, brakeLevel);
    else
        moveWithoutPid(motorLPWM, motorRPWM, motorDistChkPt, brakeLevel);
}

void rotate(float angle, byte dir, int brakeLevel, float speedRatio)
{
    resetMove();
    if (dir == CW)
    {
        LMag = -1;
        RMag = -1;
        motorLPWM = CW_PWM_L * speedRatio;
        motorRPWM = CW_PWM_R * speedRatio;
        motorDistChkPt = CW_RADIUS / 4.0 / 6.0 * 2294 * angle / 90.0;
    }
    else
    {
        LMag = 1;
        RMag = 1;
        motorLPWM = CCW_PWM_L * speedRatio;
        motorRPWM = CCW_PWM_R * speedRatio;
        motorDistChkPt = CCW_RADIUS / 4.0 / 6.0 * 2294 * angle / 90.0;
    }
    go(motorLPWM, motorRPWM, motorDistChkPt, brakeLevel);
}

void back()
{
    resetMove();
    LMag = 1;
    RMag = 1;
    motorLPWM = BACK_PWM_L;
    motorRPWM = BACK_PWM_R;
    motorDistChkPt = BACK_DIST;
    go(motorLPWM, motorRPWM, motorDistChkPt, 400);
}

void motorLISRA()      //ISR for left motor encoder interrupt
{
    motorLAccmEncoderCount++;
    motorLNewB ^motorLOldA ? motorLNetEncoderCount++ : motorLNetEncoderCount--;
    motorLOldA = digitalReadFast(MOTOR_L_ENCODER_A);
}

void motorLISRB()      //ISR for left motor encoder interrupt
{
    motorLAccmEncoderCount++;
    motorLNewB = digitalReadFast(MOTOR_L_ENCODER_B);
    motorLNewB ^motorLOldA ? motorLNetEncoderCount++ : motorLNetEncoderCount--;
}

void motorRISRA()      //ISR for left motor encoder interrupt
{
    motorRAccmEncoderCount++;
    motorRNewB ^motorROldA ? motorRNetEncoderCount-- : motorRNetEncoderCount++;
    motorROldA = digitalReadFast(MOTOR_R_ENCODER_A);
}

void motorRISRB()      //ISR for left motor encoder interrupt
{
    motorRAccmEncoderCount++;
    motorRNewB = digitalReadFast(MOTOR_R_ENCODER_B);
    motorRNewB ^motorROldA ? motorRNetEncoderCount-- : motorRNetEncoderCount++;
}

