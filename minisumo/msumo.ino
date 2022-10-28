#include <Wire.h>
#include <VL53L1X.h>
#include "includes.h"


#define START_PIN 0
#define START_INSTANT 1
#define START_5S 2

#define LINE_MIN_VALUE 50
#define OPONENT_MAX_VALUE 500


#define MAX_SPEED 255 
#define CORRECTION_DIFF (int)(MAX_SPEED*0.15)
#define SEARCH_SPEED 180
#define SPIN_TIMEOUT 700


int startType = START_INSTANT;

int endTime;
int displayTime = 0;
int cnt;

int timer, counter;
String btns = "";
int start = 0;
int fullStop = 0;

int minOponent, minIdx;

void setup() {
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  //start serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial setup");

  analogReadResolution(8);
  analogReadAveraging(1);
  analogReference(0);

  pinMode(START_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LL_PIN, INPUT);
  pinMode(LR_PIN, INPUT);

  setupMotors();
  delay(100);
  //program sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPin[i], OUTPUT);
    digitalWrite(sensorPin[i], LOW);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint8_t ad = 0x20 + i;
    setupSensor(sensorPin[i], i, ad, sensorPinInt[i]);
  }
  Serial.println("Sensors setup completed.");
  //setup buttons
  cnt = 0;
  displayTime = millis();
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void readS0() {
  availS0 = true;
}
void readS1() {
  availS1 = true;
}
void readS2() {
  availS2 = true;
}
void readS3() {
  availS3 = true;
}
void readS4() {
  availS4 = true;
}

void setupSensor(int pin, int ord, uint8_t adr, int pinInt) {
  Serial.print("Starting init on ");
  Serial.println(ord);
  pinMode(pinInt, INPUT_PULLUP);
  if (ord == 0)
    attachInterrupt(digitalPinToInterrupt(pinInt), readS0, CHANGE);
  if (ord == 1)
    attachInterrupt(digitalPinToInterrupt(pinInt), readS1, CHANGE);
  if (ord == 2)
    attachInterrupt(digitalPinToInterrupt(pinInt), readS2, CHANGE);
  if (ord == 3)
    attachInterrupt(digitalPinToInterrupt(pinInt), readS3, CHANGE);
  if (ord == 4)
    attachInterrupt(digitalPinToInterrupt(pinInt), readS4, CHANGE);
  digitalWrite(pin, HIGH);
  delay(20);
  sensor[ord].init();
  delay(20);
  sensor[ord].setAddress(adr);
  delay(10);
  sensorAddress[ord] = adr;
  Serial.println("Sensor changed address");
  sensor[ord].setDistanceMode(VL53L1X::Short);
  sensor[ord].setTimeout(21);
  sensor[ord].setMeasurementTimingBudget(10000);
  sensor[ord].startContinuous(10);
  delay(50);
  Serial.println("Sensor started continuous");

}

void readOponentSensors() {
  //  Serial.println("Reading sensor 1");
  if (availS0) {
    val[0] = sensor[0].readRangeContinuousMillimeters(false);
    if(sensor[0].ranging_data.range_status!=VL53L1X::RangeValid && sensor[0].ranging_data.range_status!=VL53L1X::None){
      val[0]=OPONENT_MAX_VALUE+1;
      countBadReads[0]++;
    }
    if(val[0]<OPONENT_MAX_VALUE)
      countRealReads[0]++;
    countReads[0]++;
    availS0 = false;
  }
  //  Serial.println("Reading sensor 2");
  if (availS1) {
    val[1] = sensor[1].readRangeContinuousMillimeters(false);
    if(sensor[1].ranging_data.range_status!=VL53L1X::RangeValid && sensor[1].ranging_data.range_status!=VL53L1X::None){
      val[1]=OPONENT_MAX_VALUE+1;
      countBadReads[1]++;
    }
    if(val[1]<OPONENT_MAX_VALUE)
      countRealReads[1]++;
    countReads[1]++;
    availS1 = false;
  }
  //  Serial.println("Reading sensor 3");
  if (availS2) {
    val[2] = sensor[2].readRangeContinuousMillimeters(false);
    if(sensor[2].ranging_data.range_status!=VL53L1X::RangeValid && sensor[2].ranging_data.range_status!=VL53L1X::None){
      val[2]=OPONENT_MAX_VALUE+1;
      countBadReads[2]++;
    }
    if(val[2]<OPONENT_MAX_VALUE)
      countRealReads[2]++;
    countReads[2]++;
    availS2 = false;
  }
  //  Serial.println("Reading sensor 4");
  if (availS3) {
    val[3] = sensor[3].readRangeContinuousMillimeters(false);
    if(sensor[3].ranging_data.range_status!=VL53L1X::RangeValid && sensor[3].ranging_data.range_status!=VL53L1X::None){
      val[3]=OPONENT_MAX_VALUE+1;
      countBadReads[3]++;
    }
    if(val[3]<OPONENT_MAX_VALUE)
      countRealReads[3]++;
    countReads[3]++;
    availS3 = false;
  }
  //  Serial.println("Reading sensor 5");
  if (availS4) {
    val[4] = sensor[4].readRangeContinuousMillimeters(false);
    if(sensor[4].ranging_data.range_status!=VL53L1X::RangeValid && sensor[4].ranging_data.range_status!=VL53L1X::None){
      val[4]=OPONENT_MAX_VALUE+1;
      countBadReads[4]++;
    }
    if(val[4]<OPONENT_MAX_VALUE)
      countRealReads[4]++;
    countReads[4]++;
    availS4 = false;
  }
  minOponent = OPONENT_MAX_VALUE + 1;
  minIdx = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    //    val[i] = sensor[i].readReg16Bit(0x14 + 10);
    if (val[i] > OPONENT_MAX_VALUE)
      val[i] = OPONENT_MAX_VALUE;
    if (val[i] < minOponent) {
      minIdx = i;
      minOponent = val[i];
    }
  }
  //  Serial.println("End reading sensors");
}

void readLineSensors(){
  lineR=analogRead(LR_PIN);
  lineL=analogRead(LL_PIN);
}

#define RUNNING 1
#define WAITING 0
#define STOPPED 2

#define STRATEGY_OCE 1 //10000
#define STRATEGY_OIN 2 //00001
#define STRATEGY_FRONTAL 3 //00100
#define STRATEGY_FRONTAL_WATCH 4 //01110
#define STRATEGY_LEFT_WATCH 5 //01000
#define STRATEGY_RIGHT_WATCH 6 //00010
#define STRATEGY_TEST 7 //10001

int status = WAITING;

int strategy = STRATEGY_FRONTAL;


void loop() {

  readOponentSensors();
  readLineSensors();

  int startVal = digitalRead(START_PIN);
  if (startVal == 1 && startType == START_5S) {
    if (status == WAITING) {
      digitalWrite(LED_PIN,HIGH);
      delay(5000);
      digitalWrite(LED_PIN,LOW);
      status = RUNNING;
      strategyRun();
    } else if (status == RUNNING) {
      status = STOPPED;
      setSpeeds(0, 0);
      Serial.println("STOPPED");
    }
  } else if (startVal == 1 && startType == START_INSTANT) {
    if (status == WAITING) {
      status = RUNNING;
      strategyRun();
    }
  } else if (startVal == 0 && startType == START_INSTANT) {
    if (status == RUNNING) { //running we need to stop
      status = STOPPED;
      setSpeeds(0, 0);
      Serial.println("STOPPED");
    }
  }
  if (status == WAITING) {
    if (val[0] < ST_DIST && val[1] > ST_DIST && val[2] > ST_DIST && val[3] > ST_DIST && val[4] > ST_DIST) { //left
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      strategy = STRATEGY_OCE;
    }
    if (val[0] > ST_DIST && val[1] > ST_DIST && val[2] > ST_DIST && val[3] > ST_DIST && val[4] < ST_DIST) { //right
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      strategy = STRATEGY_OIN;
    }
    if (val[0] > ST_DIST && val[1] < ST_DIST && val[2] < ST_DIST && val[3] < ST_DIST && val[4] > ST_DIST) { //01110
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      strategy = STRATEGY_FRONTAL_WATCH;
    }
    if (val[0] > ST_DIST && val[1] > ST_DIST && val[2] < ST_DIST && val[3] > ST_DIST && val[4] > ST_DIST) { //01110
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      strategy = STRATEGY_FRONTAL;
    }
    if (val[0] > ST_DIST && val[1] < ST_DIST && val[2] > ST_DIST && val[3] > ST_DIST && val[4] > ST_DIST) { //01000
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(400);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      strategy = STRATEGY_LEFT_WATCH;
    }
    if (val[0] > ST_DIST && val[1] > ST_DIST && val[2] > ST_DIST && val[3] < ST_DIST && val[4] > ST_DIST) { //00010
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(400);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(400);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      strategy = STRATEGY_RIGHT_WATCH;
    }
    if (val[0] < ST_DIST && val[1] > ST_DIST && val[2] > ST_DIST && val[3] > ST_DIST && val[4] < ST_DIST) { //10001
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(1000);
      strategy = STRATEGY_TEST;
    }
  }


  displayData();
  decide();
}

void strategyRun() {
  if (status == RUNNING) {
    switch (strategy) {
      case STRATEGY_TEST:
        testMotors();
        break;
    }
  }
}

int lastValue = 0;
int lastSeen = 0;

void decide() {
  if (status == RUNNING) {
    if (lineL < LINE_MIN_VALUE && lineR > LINE_MIN_VALUE) { //edgel
      digitalWrite(LED_PIN,HIGH);
      setSpeeds(-MAX_SPEED, -MAX_SPEED);
      delay(250);
      setSpeeds(MAX_SPEED,-MAX_SPEED);
      delay(180);
      setSpeeds(SEARCH_SPEED,SEARCH_SPEED);
      delay(20);
      lastValue = 5;
    }
    else  if (lineL > LINE_MIN_VALUE && lineR < LINE_MIN_VALUE) {//edger
      digitalWrite(LED_PIN,HIGH);
      setSpeeds(-MAX_SPEED, -MAX_SPEED);
      delay(250);
      setSpeeds(-MAX_SPEED,MAX_SPEED);
      delay(180);
      setSpeeds(SEARCH_SPEED,SEARCH_SPEED);
      delay(20);
      lastValue = 5;
    }
    else  if (lineL < LINE_MIN_VALUE && lineR < LINE_MIN_VALUE) {//edgerl
      digitalWrite(LED_PIN,HIGH);
      setSpeeds(-MAX_SPEED, -MAX_SPEED);
      delay(250);
      setSpeeds(-MAX_SPEED,MAX_SPEED);
      delay(150);
      setSpeeds(SEARCH_SPEED,SEARCH_SPEED);
      delay(20);
      lastValue = 5;
      lastSeen = millis();
      lastValue = 5;
    } else if (val[1] < OPONENT_MAX_VALUE && val[2] < OPONENT_MAX_VALUE) {//01100
      digitalWrite(LED_PIN,HIGH);      
      setSpeeds(MAX_SPEED-(MAX_SPEED>>2),MAX_SPEED);
      delay(2);
      lastSeen = millis();
      lastValue = 7;
    } else if (val[1] < OPONENT_MAX_VALUE && val[2] < OPONENT_MAX_VALUE) {//00110
      digitalWrite(LED_PIN,HIGH);      
      setSpeeds(MAX_SPEED,MAX_SPEED-(MAX_SPEED>>2));
      delay(2);
      lastSeen = millis();
      lastValue = 3;
    }  else if (val[2] < OPONENT_MAX_VALUE) { //00100
      digitalWrite(LED_PIN,HIGH);      
      setSpeeds(MAX_SPEED,MAX_SPEED);
      delay(2);
      lastSeen = millis();
      lastValue = 5;
    } else if (val[0]<OPONENT_MAX_VALUE || val[1] < OPONENT_MAX_VALUE) {
      digitalWrite(LED_PIN,HIGH);      
      setSpeeds(-MAX_SPEED,MAX_SPEED);
      delay(20);
      lastSeen = millis();
      lastValue = 7;
    } else if (val[3]<OPONENT_MAX_VALUE || val[4]<OPONENT_MAX_VALUE) {
      digitalWrite(LED_PIN,HIGH);      
      setSpeeds(MAX_SPEED,-MAX_SPEED);
      delay(20);
      lastSeen = millis();
      lastValue = 3;
    } else //do based on history
    {
      if (lastValue == 5) {
        setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
        delay(2);
      } else if (LastValue == 7) {
          setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
          delay(2);
          if (millis() - lastSeen > SPIN_TIMEOUT) {
            setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
            delay(40);
            lastSeen = millis();
          }
        } else if (LastValue == 3) {
          setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
            if (millis() - lastSeen > SPIN_TIMEOUT) {
              setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
              delay(40);
              lastSeen = millis();
            }
          }
        
    }
  }


}

void displayData() {
  endTime = millis();
  cnt++;
  //check if it's time to show .. only in waiting
  if ((status == WAITING || status == STOPPED) && endTime - displayTime > 1000) {
    Serial.println("Reading values");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(val[i]);
      Serial.print(" ");
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensor[i].rangeStatusToString(sensor[i].ranging_data.range_status));
      Serial.print(";");
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(countReads[i]);
      Serial.print(" ");
//      countReads[i] = 0;
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(countReads[i]-countBadReads[i]);
      Serial.print(" ");
      countBadReads[i] = 0;
      countReads[i]=0;
    }
    Serial.println();
    Serial.print("START:");
    Serial.print(digitalRead(START_PIN));
    Serial.print(" STATUS:");
    Serial.print(status);
    Serial.print(" LL:");
    Serial.print(analogRead(LL_PIN));
    Serial.print(" LR:");
    Serial.println(analogRead(LR_PIN));
    Serial.print("TOTAL COUNTS:");
    Serial.println(cnt);
    displayTime = millis();
    cnt = 0;
  }
}

void calibrateLoop() {
  //rotate robot at dif speeds and record degrees/second
  //memorize minspeed when robot rotates

  //set motorspeed 50%  with shortest acceleration

  //go to edge
  //check if both  leds on edge if not rotate 10 back 250ms (quarter max speed) and repeat previous
  //rotate 180 robot


  //save orientation
  //run to oposite edge then brake and record time
  //check orientation to see if motors are synced

  //if motors are synced in 10% range
}

void testMotors() {
  Serial.println("Running test");
  setSpeeds(100, -100);
  delay(2000);
  setSpeeds(-100, 100);
  delay(2000);
  setSpeeds(0, 0);
}


