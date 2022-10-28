#include "includes.h"

bool volatile availS0 = false;
bool volatile availS1 = false;

int countReads[2] = {0, 0};

int startTime, endTime;
long cnt;

int timer, counter;
String btns = "";
int start = 0;
int fullStop = 0;

bool stratRun = false;

int detected = 0;
bool running = false;

int startButtonTime = 0;

int minOponent, minIdx;
int val[NUM_SENSORS];

volatile IRAM_ATTR unsigned long pulseInTimeBegin0 = micros();
volatile IRAM_ATTR unsigned long pulseInTimeEnd0 = micros();
volatile IRAM_ATTR bool newPulseDurationAvailable0 = false;
volatile IRAM_ATTR unsigned long pulseInTimeBegin1 = micros();
volatile IRAM_ATTR unsigned long pulseInTimeEnd1 = micros();
volatile IRAM_ATTR bool newPulseDurationAvailable1= false;



void IRAM_ATTR sensorPinInterrupt0()
{
  if (digitalRead(sensorPin[0]) == HIGH) {
    // start measuring
    pulseInTimeBegin0 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd0 = micros();
    pulseDuration[0] = pulseInTimeEnd0 - pulseInTimeBegin0;
    newPulseDurationAvailable0 = true;
  }
}

void IRAM_ATTR sensorPinInterrupt1()
{
  if (digitalRead(sensorPin[1]) == HIGH) {
    // start measuring
    pulseInTimeBegin1 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd1 = micros();
    pulseDuration[1] = pulseInTimeEnd1 - pulseInTimeBegin1;
    newPulseDurationAvailable1 = true;
  }
}




void setup() {
  Wire.begin();

  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");

  setupMotors();
  Serial.println("Motors set");

  pinMode(sensorPin[0], INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin[0]),
                  sensorPinInterrupt0,
                  CHANGE);
                  
  pinMode(sensorPin[1], INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin[1]),
                  sensorPinInterrupt1,
                  CHANGE);

  setupStartSensor();


  setSpeeds(0, 0);

  startTime = millis();
  startButtonTime = millis();
  start = 0;
}

int LastValue = 5; // Last Value Variable for remembering last Opponent sensor sense.
int LastSeen = 0;


void decide() {
   if (val[0] < OPONENT_MAX_VALUE && val[1] < OPONENT_MAX_VALUE) { //both
    setSpeeds(MAX_SPEED, MAX_SPEED);
    delay(1);
    LastSeen = millis();
    LastValue = 5;
  } else if (val[0] < OPONENT_MAX_VALUE) { //right
    setSpeeds(MAX_SPEED, MAX_SPEED>>1);
    delay(2);
    LastSeen = millis();
    LastValue = 7;
  } else if (val[1] < OPONENT_MAX_VALUE) { //left
    setSpeeds(MAX_SPEED>>1, MAX_SPEED);
    delay(2);
    LastSeen = millis();
    LastValue = 3;
  } else // no  detection
  {
    if (LastValue == 5) {
      setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
      delay(2);
    } else 
      if (LastValue == 7) {
        setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
        delay(2);
        if (millis() - LastSeen > 700) {
          setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
          delay(60);
        }
      } else {// Left Turning Based on SPD (A7) Trimpot
        if (LastValue == 3) {
          setSpeeds(-SEARCH_SPEED, SEARCH_SPEED); // Right Turning Based on SPD (A7) Trimpot
          delay(2);
          if (millis() - LastSeen > 700) {
            setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
            delay(60);
          }
        }
      }
  }

}

void readOponentSensors() {
  if (newPulseDurationAvailable0) {
    newPulseDurationAvailable0 = false;
    val[0] = 3.0/4.0*(pulseDuration[0]-1000.0);
    if (val[0] > OPONENT_MAX_VALUE) 
      val[0] = OPONENT_MAX_VALUE;
    countReads[0]++;
  }

  if (newPulseDurationAvailable1) {
    newPulseDurationAvailable1 = false;
    val[1] = 3.0/4.0*(pulseDuration[1]-1000.0);
    if (val[1] > OPONENT_MAX_VALUE) 
      val[1] = OPONENT_MAX_VALUE;
    countReads[1]++;
    //Serial.println(3.0/4.0*(pulseDuration-1000.0));
  }

    minOponent = OPONENT_MAX_VALUE + 1;
    minIdx = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (val[i] > OPONENT_MAX_VALUE)
        val[i] = OPONENT_MAX_VALUE;
      if (val[i] < minOponent) {
        minIdx = i;
        minOponent = val[i];
      }
    }
}


void readStrat() {
  //TODO: read stragey
}

void loop() {

  int startVal = analogRead(START_PIN);
  if (startVal > 128) {
    start = 1;
      if (!stratRun) {
        readStrat();
        start = 1;
        stratRun = true;
        ftz();
      }
    } else {
      setSpeeds(1, 1);
      start = 0;
      delay(1);
    }

  detected = 0;
  readOponentSensors();

    endTime = millis();
    cnt++;
    if (endTime - startTime >= 1000) {
      for(int i=0;i<NUM_SENSORS;i++){
        Serial.print("val[");
        Serial.print(i);
        Serial.print("]=");
        Serial.println(val[i]);
      }
      for(int i=0;i<NUM_SENSORS;i++){
        Serial.print("count[");
        Serial.print(i);
        Serial.print("]=");
        Serial.println(countReads[i]);
        countReads[i] = 0;
      }
      Serial.print("Strat:");
      Serial.println(stratIdx);
      Serial.print("LastSeen:");Serial.println(millis()-LastSeen);
      Serial.print("LastValue:");Serial.println(LastValue);
      cnt = 0;
      startTime = millis();
    }

  //if it's started decide what to do next
  if (start == 1 && fullStop == 0) {
    decide();
  }

}

void ftz() {
  Serial.println("Fatza");
  setSpeeds(MAX_SPEED, MAX_SPEED);
  delay(RUNDUR);
  setSpeeds(MAX_SPEED, MAX_SPEED);
  delay(RUNDUR);
  LastValue=5;
}

void test() {
  Serial.println("Testing motors");
  setSpeeds(250, 0);
  delay(3000);
  setSpeeds(0, 250);
  delay(3000);
  setSpeeds(-250, 0);
  delay(3000);
  setSpeeds(0, -250);
  delay(3000);
}

void rce() {
  Serial.println("rce");
  setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
  delay(ROT90DUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
  delay(RUNDUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
}

void rin() {
  Serial.println("rin");
  setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
  delay(ROT90DUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
  delay(RUNDUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
}

void oce() {
  Serial.println("oce");
  setSpeeds(MAX_SPEED, MAX_SPEED>>1);
  delay(RUNDURL);
  setSpeeds(MAX_SPEED, -MAX_MAX);
  delay(ROT45DUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
  LastValue=5;
}

void oin() {
  Serial.println("oin");
  setSpeeds(MAX_SPEED>>1, MAX_SPEED);
  delay(RUNDURL);
  setSpeeds(-MAX_SPEED, MAX_MAX);
  delay(ROT45DUR);
  setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
  LastValue=5;
}
