//Include library for VL53L0X sensor
#include <VL53L0X.h>
//Inlcude library for i2c
#include <Wire.h>

//number of distance sensors
#define NUM_SENSORS 3
//number of strategies pins (dipswitch)
#define NUM_STRAT 4

//array of VL53L0X sensor object
VL53L0X sensor[NUM_SENSORS];
//array of sensor addreses
uint8_t sensorAddress[NUM_SENSORS];

//pin XSHUT distance sensors (on/off)
int sensorPin[NUM_SENSORS] = {1, A7, 12 };
//pin INT distance sensors (interrupt pin)
int sensorPinInt[NUM_SENSORS] = {0, A8, 11 };

//strategy pins (dipswitch)
int stratPin[NUM_STRAT] = {A1, A2, A3, A6};
//strategies number
const int stratCount = 1;
//strategies names
String strat[stratCount] = {"forward"};
//strategies values based on dipswitch configuration
int stratValue[stratCount]={6};
//default strategy index
int stratIdx = 0;
//startmodule pin
int startPin = A14;
//is strategy executed
bool stratRun = false;


//BLACK MAX VALUE for left sensor in case not similar values for left and right
#define LINE_MIN_VALUE_L 60
//BLACK MAX VALUE for right sensor
#define LINE_MIN_VALUE_R 60

//max valid distance to oponent everything over that should be ignored because it's outside perimeter
#define OPPONENT_MAX_VALUE 300

//led pin
#define LED_PIN 13

//right line senosr pin - anlog
#define LSR A0
//left line sensor pin - analog
#define LSL A9

//motor PWM pins
#define A1IN 6
#define B1IN 4
#define A2IN 5
#define B2IN 3

//constant to reverse or not the value of motor in case wires where connected reversed
#define motorLFw 1
#define motorRFw 1

//parameters for speed
#define MAX_SPEED 250
//max speed when executing strategy
#define MAX_STRAT_SPEED 180
//search speed used when turning
#define SEARCH_SPEED 100
//how long should move forward during fw strategy
#define FWDUR 150

//line sensor values
int lineL = 0 ;
int lineR = 0;

//values from distance sensors are read from interrupts so we need voltaile flag variables to know when to read new measurement from sensor
bool volatile availS0 = false;
bool volatile availS1 = false;
bool volatile availS2 = false;

long startTime, endTime;

int start = 0;

int minOponent, minIdx;
int val[NUM_SENSORS];

#define STRAT_FTZ   B0110

void setup() {
  //initialize i2c bus
  Wire.begin();
  //set i2c frequency to 400khz
  Wire.setClock(400000);
  //turn on led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Starting...");
  //setup analog read resolution to 8bit
  analogReadResolution(8);
  //set how many reads to average before answering
  analogReadAveraging(3);
  //set reference to 0
  analogReference(0);

  //seting motor pins as outputs
  pinMode(A1IN, OUTPUT);
  pinMode(A2IN, OUTPUT);
  pinMode(B1IN, OUTPUT);
  pinMode(B2IN, OUTPUT);

  //set everything to low means motors are off
  digitalWrite(A1IN, LOW);
  digitalWrite(A2IN, LOW);
  digitalWrite(B1IN, LOW);
  digitalWrite(B2IN, LOW);

  //set pwm frequency for each pin 20khz is a good value
  analogWriteFrequency(A1IN, 20000);
  analogWriteFrequency(A2IN, 20000);
  analogWriteFrequency(B1IN, 20000);
  analogWriteFrequency(B2IN, 20000);

  //set start pin as input
  pinMode(startPin, INPUT);
  //set left line sensor as input
  pinMode(LSL, INPUT);
  //set right line sensor as input
  pinMode(LSR, INPUT);

  //setup each sensor XSHUT pin to low so it will turn sensor off
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPin[i], OUTPUT);
    digitalWrite(sensorPin[i], LOW);
  }

  //setup strategy pins
  for (int i = 0; i < NUM_STRAT; i++) {
    pinMode(stratPin[i], INPUT_PULLUP); //HIGH when OFF, LOW WHEN ON - dipswitch wired to GND
  }

  //program sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    //each sensor new address
    uint8_t ad = 0x62 + i;
    setupSensor(i, ad);
  }
  delay(10);

  //make sure speeds are 0,0
  setSpeeds(0, 0);
  startTime = millis();
  Serial.println("Everything set");
  //turn off led to signal everything is ready
  digitalWrite(13, LOW);

}

//function that will setup a sensor, it will receive index in sensor array, new address
void setupSensor(int idx, uint8_t adr) {
  //set interrupt pin as input pulled up - high always except when triggers
  pinMode(sensorPinInt[idx], INPUT_PULLUP);
  //attach interrupt function for first sensor
  if (idx == 0)
    attachInterrupt(digitalPinToInterrupt(sensorPinInt[idx]), readS0, CHANGE);
  if (idx == 1)
    attachInterrupt(digitalPinToInterrupt(sensorPinInt[idx]), readS1, CHANGE);
  if (idx == 2)
    attachInterrupt(digitalPinToInterrupt(sensorPinInt[idx]), readS2, CHANGE);
  //turn sensor on
  digitalWrite(sensorPin[idx], HIGH);
  //wait 40ms for sensor to boot as in specs
  delay(40);
  //initialize sensor
  sensor[idx].init();
  delay(30);
  //set sensor new address
  sensor[idx].setAddress(adr);
  delay(20);
  //set sensor timeout period 26ms
  sensor[idx].setTimeout(26);
  //set sensor measurement timing budget to 25ms
  sensor[idx].setMeasurementTimingBudget(25000);
  //start countinuous reading every 10ms or as fast as possible
  sensor[idx].startContinuous(10);
  //store new address
  sensorAddress[idx] = adr;
  Serial.print("Sensor ");
  Serial.print(idx + 1);
  Serial.println(" set.");
}

//function to set flag that new measurement is available for sensor 0
void readS0() {
  availS0 = true;
}

//function to set flag that new measurement is available for sensor 1
void readS1() {
  availS1 = true;
}

//function to set flag that new measurement is available for sensor 2
void readS2() {
  availS2 = true;
}

//setSpeeds for motors without breaking
void setSpeeds(int leftSpeed, int rightSpeed) {
  setSpeeds(leftSpeed, rightSpeed, false);
}

//set speeds for motors with breaking flag
void setSpeeds(int leftSpeed, int rightSpeed, boolean breaking) {
  //adjust in case of wires switch based on variables
  if (motorLFw == -1) // means that speed should be actual negative
    leftSpeed = -leftSpeed;
  if (motorRFw == -1) // means that speed should be actual negative
    rightSpeed = -rightSpeed;

  //now set speeds for real
  //set no breaking for breaking pin
  int bPin = LOW;
  if (breaking) { // if breaking set breaking pin to high
    bPin = HIGH;
  }

  //left motor
  if (leftSpeed > 0) { //forward
    digitalWrite(B2IN, 0);
    analogWrite(B1IN, leftSpeed);
  } else if (leftSpeed == 0) { //breaking or coasting based on bPin
    analogWrite(B1IN, bPin);
    analogWrite(B2IN, bPin);
  } else { //reverse
    digitalWrite(B1IN, 0);
    analogWrite(B2IN, -leftSpeed);
  }

  //right motor
  if (rightSpeed > 0) { //forward
    digitalWrite(A1IN, 0);
    analogWrite(A2IN, rightSpeed);
  } else if (rightSpeed == 0) { //breaking or coasting based on bpin
    analogWrite(A1IN, bPin);
    analogWrite(A2IN, bPin);
  } else {//reverse
    digitalWrite(A2IN, 0);
    analogWrite(A1IN, -rightSpeed);
  }

}
//variables to remember where and when last time opponent was spotted
int LastValue = 5; //default forward - 5, right - 3, left - 7
int LastSeen = 0;

//decision function (rules that robot will follow)
void decide() {
  digitalWrite(LED_PIN, LOW);
  if (lineR < LINE_MIN_VALUE_R || lineL < LINE_MIN_VALUE_L) { //edge detected
    digitalWrite(LED_PIN, HIGH); //turn on led to signal it detected something
    if (lineR < LINE_MIN_VALUE_R && lineL > LINE_MIN_VALUE_L) { //right edge detected
      setSpeeds(0, 0);//stop
      delay(10);
      setSpeeds(-SEARCH_SPEED, -SEARCH_SPEED); //reverse
      delay(250); // reverse 250ms
      setSpeeds(0, 0);
      setSpeeds(0, SEARCH_SPEED);//turn left
      delay(190);
      setSpeeds(0, 0);
      setSpeeds(SEARCH_SPEED, SEARCH_SPEED); //go forward
      delay(20);
    }
    if (lineL < LINE_MIN_VALUE_L && lineR > LINE_MIN_VALUE_R) { //left edge detected
      setSpeeds(0, 0); //stop
      delay(10);
      setSpeeds(-SEARCH_SPEED, -SEARCH_SPEED); //reverse
      delay(250); // reverse 250ms
      setSpeeds(0, 0);
      setSpeeds(SEARCH_SPEED, 0); //turn right
      delay(190); //
      setSpeeds(0, 0);
      setSpeeds(SEARCH_SPEED, SEARCH_SPEED); //go forward
      delay(20);
    }
    if (lineL < LINE_MIN_VALUE_L && lineR < LINE_MIN_VALUE_R) {  //left and right edge
      setSpeeds(0, 0);//stop
      delay(10);
      setSpeeds(-SEARCH_SPEED, -SEARCH_SPEED);
      delay(250); // reverse 250ms
      setSpeeds(0, 0);
      setSpeeds(0, SEARCH_SPEED); //turn left
      delay(190);
      setSpeeds(0, 0);
      setSpeeds(SEARCH_SPEED, SEARCH_SPEED); //go forward
      delay(20);
    }
  } else if (val[1] < OPPONENT_MAX_VALUE) { //middle distance sensor detected
    digitalWrite(LED_PIN, HIGH);
    setSpeeds(0, 0);
    setSpeeds(MAX_SPEED, MAX_SPEED);
    delay(10);
    LastSeen = millis();
    LastValue = 5; //we detected it in front of us
  } else if (val[0] < OPPONENT_MAX_VALUE) { //left distancesensor detected
    digitalWrite(LED_PIN, HIGH);
    setSpeeds(0, 0);
    setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
    delay(100);
    LastSeen = millis();
    LastValue = 7;//we detected left side
  } else if (val[2] < OPPONENT_MAX_VALUE) {//right
    digitalWrite(LED_PIN, HIGH);
    setSpeeds(0, 0);
    setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
    delay(100);
    LastSeen = millis();
    LastValue = 3; //we detected on right side
  } else // no edge, no distance, just go forward until you find an edge or sense opponent
  {
    setSpeeds(0, 0);
    setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
    delay(2);
  }

}

//read distance sensors
void readOpponentSensors() {
  if (availS0) { //we need to read new value from sensor 0
    val[0] = sensor[0].readRangeContinuousMillimeters();
    if (val[0] > 300 ) val[0] = 300; // if we detected something over max distance consider it's max distance
    availS0 = false; //value read .. waiting for new value
  }
  if (availS1) {//we need to read new value from sensor 1
    val[1] = sensor[1].readRangeContinuousMillimeters();
    if (val[1] > 300 ) val[1] = 300;
    availS1 = false;
  }
  if (availS2) {//we need to read new value from sensor 2
    val[2] = sensor[2].readRangeContinuousMillimeters();
    if (val[2] > 300) val[2] = 300;
    availS2 = false;
  }
}

//function read line sensors
void readLineSensors() {
  lineR = analogRead(LSR);
  lineL = analogRead(LSL);
}


//function to read strategy .. each pin is a bit so the result will be a number composed from NUM_STRAT bits based on HIGH-1 LOw-0
void readStrat() {
  stratIdx = 0;
  for (int i = 0; i < NUM_STRAT; i++) {
    int val = analogRead(stratPin[i]);
    Serial.print(val); Serial.print(":");
    if (val < 10)
      stratIdx += 1 << i;
  }
}


void loop() {

  if (analogRead(startPin)>200) { // if pin is high
    start = 1; // set that is running
    if (!stratRun) { //if strat was not executed
      readStrat(); //read strategy
      stratRun = true;
      Serial.print("Strat:"); Serial.println(stratIdx);
      switch (stratIdx) {
        case STRAT_FTZ: //forward strategy;
          fw();
          break;
        default: // anything else just test motors
          test();
      }
    }
  } else { // start module pin is low so motors should be off
    start = 0; //set that is not running
    setSpeeds(0, 0, false);
  }

  readOpponentSensors();
  readLineSensors();

  //if it's started decide what to do next
  if (start == 1) { // no decision until start is done
    decide();
  }

}

//function to test motors correct spin
//the order is left forward, right forward, left reverse, right reverse
void test() {
  Serial.println("Testing motors");
  //Left forward
  setSpeeds(100, 0);
  delay(3000);
  setSpeeds(0, 0);
  delay(1000);
  //right forward
  setSpeeds(0, 100);
  delay(3000);
  setSpeeds(0, 0);
  delay(1000);
  //Left reverse
  setSpeeds(-100, 0);
  delay(3000);
  setSpeeds(0, 0);
  delay(1000);
  //Right reverse
  setSpeeds(0, -100);
  delay(3000);
  setSpeeds(0, 0);
}


//function for forward strategy
void fw() {
  Serial.println("ftz");
  setSpeeds(0, 0);
  setSpeeds(MAX_STRAT_SPEED, MAX_STRAT_SPEED);
  delay(FWDUR);
}
