#define NUM_SENSORS 2
#define NUM_STRAT 3
#define NUMPIXELS        1


int sensorPin[NUM_SENSORS] = { 18, 8 };
volatile IRAM_ATTR long pulseDuration[NUM_SENSORS] = {0,0};

#define START_PIN A1
#define START_GND A3
#define START_VCC 7

#define A1IN 6
#define A2IN 36
#define B1IN 37
#define B2IN 35

#define STATE_STOP 0
#define STATE_FORWARD 1
#define STATE_SPIN_LEFT 2
#define STATE_SPIN_RIGHT 3
#define STATE_REVERSE 4

const int stratCount = 3;
String strat[stratCount] = {"oce", "oin","ftz"};
int stratValue[stratCount]={8,1,12};
int stratIdx = 0;

#define LINE_MIN_VALUE 18
#define OPONENT_MAX_VALUE 200

#define motorL 1
#define motorR 1
#define motorLFw 1
#define motorRFw 1

#define ROT90DUR 100
#define ROT45DUR 30
#define RUNDURS 100
#define RUNDUR 200
#define RUNDURL 280
#define ROTDIFF (int)(MAX_SPEED*0.3)
#define EDGE_TURN 180
#define EDGE_FULL_TURN 260
#define EDGE_BACK 35
#define MOTOR_BREAK 1
#define MOTOR_ACCEL 127

#define MAX_SPEED 255 
#define MAX_MAX 255 
#define CORRECTION_DIFF (int)(MAX_SPEED*0.15)
#define SEARCH_SPEED 180


#define STRAT_TEST  B0000
#define STRAT_RCE   B1000
#define STRAT_OCE   B1100
#define STRAT_RIN   B0001
#define STRAT_OIN   B0011 
#define STRAT_FTZ   B0110
