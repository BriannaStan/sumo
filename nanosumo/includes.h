#include "statics.h"
#include <pwmWrite.h>
#include "Wire.h"

Pwm pwm = Pwm();
uint8_t sensorAddress[NUM_SENSORS];


unsigned char m0Speed;
unsigned char m1Speed;

int lineL = 0 ;
int lineR = 0;


void setupDistanceSensors(){
  
}

void setupStartSensor(){
  digitalWrite(START_VCC,HIGH);
  digitalWrite(START_GND,LOW);
  pinMode(START_PIN, INPUT);
}

void setupMotors(){
    //seting motor pwms
  pwm.write(A1IN, 0);
  pwm.write(A2IN, 0);
  pwm.write(B1IN, 0);
  pwm.write(B2IN, 0);
  pwm.writeFrequency(A1IN, 2000);
  pwm.writeFrequency(A2IN, 2000);
  pwm.writeFrequency(B1IN, 2000);
  pwm.writeFrequency(B2IN, 2000);
  pwm.writeResolution(A1IN, 8);
  pwm.writeResolution(A2IN, 8);
  pwm.writeResolution(B1IN, 8);
  pwm.writeResolution(B2IN, 8);
  pwm.printPinsStatus();
}

void setupSensors(){
  //TODO: Set sensors
}

void setSpeeds(int leftSpeed, int rightSpeed, boolean breaking) {
  //adjust in case of wires switch
  if (motorLFw == -1)
    leftSpeed = -leftSpeed;
  if (motorRFw == -1)
    rightSpeed = -rightSpeed;
  //now set speeds for real

  int bPin = LOW;
  if (breaking) {
    bPin = HIGH;
  }
  if (leftSpeed >= 0) {
    pwm.write(B2IN, bPin);
    pwm.write(B1IN, leftSpeed);
//    analogWrite(B2IN, bPin);
//    analogWrite(B1IN, leftSpeed);
  } else if (leftSpeed == 0) {
    pwm.write(B2IN, bPin);
    pwm.write(B1IN, bPin);
//    digitalWrite(B1IN, bPin);
//    digitalWrite(B2IN, bPin);
  } else {
    pwm.write(B1IN, bPin);
    pwm.write(B2IN, -leftSpeed);
//    digitalWrite(B1IN, bPin);
//    analogWrite(B2IN, -leftSpeed);
  }

  if (rightSpeed > 0) {
//    digitalWrite(A1IN, bPin);
//    analogWrite(A2IN, rightSpeed);
    pwm.write(A1IN, bPin);
    pwm.write(A2IN, rightSpeed);
  } else if (rightSpeed == 0) {
//    digitalWrite(A1IN, bPin);
//    digitalWrite(A2IN, bPin);
    pwm.write(A1IN, bPin);
    pwm.write(A2IN, bPin);
  } else {
//    digitalWrite(A2IN, bPin);
//    analogWrite(A1IN, -rightSpeed);
    pwm.write(A2IN, bPin);
    pwm.write(A1IN, -rightSpeed);
  }

}

void setSpeeds(int leftSpeed, int rightSpeed) {
  setSpeeds(leftSpeed, rightSpeed, false);
}
