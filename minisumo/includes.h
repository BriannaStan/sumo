#include <VL53L1X.h>
#include <Wire.h>
#include "statics.h"

#define NUM_SENSORS 5
#define ST_DIST 50
#define SAFE_DISTANCE 300
#define LED_PIN 13


int countReads[NUM_SENSORS] = {0, 0, 0, 0, 0};
int countRealReads[NUM_SENSORS] = {0, 0, 0, 0, 0};
int countBadReads[NUM_SENSORS] = {0, 0, 0, 0, 0};
int sensorPin[NUM_SENSORS] = { 1, 22, 20, 16, 11 };
int sensorPinInt[NUM_SENSORS] = { 2, 21, 15, 14, 12 };


bool volatile availS0 = false;
bool volatile availS1 = false;
bool volatile availS2 = false;
bool volatile availS3 = false;
bool volatile availS4 = false;

uint8_t sensorAddress[NUM_SENSORS];
VL53L1X sensor[NUM_SENSORS];
int val[NUM_SENSORS];

#define MOTORL_IN1 3
#define MOTORL_IN2 4
#define MOTORR_IN1 9
#define MOTORR_IN2 10

#define motorL 1
#define motorR 1
#define motorLFw 1
#define motorRFw 1

#define LL_PIN A9
#define LR_PIN A3
int lineL = 0 ;
int lineR = 0;

int mrLastSpeed = 0;
int mlLastSpeed = 0;
boolean mLastBreaking = false;

int LastValue = 5; 
int LastSeen = 0;



void setupMotors(){
  //seting motor pwms
  analogWriteResolution(8);  
//  analogWriteFrequency(MOTORL_IN1, 20000);
//  analogWriteFrequency(MOTORL_IN2, 20000);
//  analogWriteFrequency(MOTORR_IN1, 20000);
//  analogWriteFrequency(MOTORR_IN2, 20000);
  pinMode(MOTORL_IN1, OUTPUT);
  pinMode(MOTORL_IN2, OUTPUT);
  pinMode(MOTORR_IN1, OUTPUT);
  pinMode(MOTORR_IN2, OUTPUT);
  analogWrite(MOTORL_IN1, 0);
  analogWrite(MOTORL_IN2, 0);
  analogWrite(MOTORR_IN1, 0);
  analogWrite(MOTORR_IN2, 0);
  
}

void setSpeeds(int leftSpeed, int rightSpeed, boolean breaking) {
  //adjust in case of wires switch
  mrLastSpeed=rightSpeed;
  mlLastSpeed=leftSpeed;
  mLastBreaking=breaking;
  if (motorLFw == -1)
    leftSpeed = -leftSpeed;
  if (motorRFw == -1)
    rightSpeed = -rightSpeed;
  //now set speeds for real

  int bPin = 0;
  if (breaking) {
    bPin = 254;
  }
  if (leftSpeed >= 0) {
    analogWrite(MOTORL_IN2, bPin);
    analogWrite(MOTORL_IN1, leftSpeed);
  } else if (leftSpeed == 0) { //bpin 0 or 1 coast or break
    analogWrite(MOTORL_IN1, bPin);
    analogWrite(MOTORL_IN2, bPin);
  } else {
    analogWrite(MOTORL_IN1, bPin);
    analogWrite(MOTORL_IN2, -leftSpeed);
  }

  if (rightSpeed > 0) {
    analogWrite(MOTORR_IN1, bPin);
    analogWrite(MOTORR_IN2, rightSpeed);
  } else if (rightSpeed == 0) {
    analogWrite(MOTORR_IN1, bPin);
    analogWrite(MOTORR_IN2, bPin);
  } else {
    analogWrite(MOTORR_IN2, bPin);
    analogWrite(MOTORR_IN1, -rightSpeed);
  }

}

void setSpeeds(int leftSpeed, int rightSpeed) {
  setSpeeds(leftSpeed, rightSpeed, false);
}



