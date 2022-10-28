//This file contains all static variables that program uses
//number of distance sensors
#define NUM_SENSORS 3
//number of strategies pins (dipswitch)
#define NUM_STRAT 4

//pin XSHUT distance sensors (on/off)
int sensorPin[NUM_SENSORS] = {1,  A7, 12 };
//pin INT distance sensors (interrupt pin)
int sensorPinInt[NUM_SENSORS] = {0, A8, 11 };

//strategy pins (dipswitch)
int stratPin[NUM_STRAT] = {A1, A2, A3, A6};

//led pin
#define LED_PIN 13

//startmodule pin
#define START_PIN A14
//right line senosr pin - anlog
#define LSR A0
//left line sensor pin - analog
#define LSL A9

//motor PWM pins
#define A1IN 6
#define B1IN 4
#define A2IN 5
#define B2IN 3


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


//#define motorL 1
//#define motorR 1
#define motorLFw 1
#define motorRFw 1

#define MAX_SPEED 250
#define MAX_STRAT_SPEED 180 
#define GL_MAX_SPEED 110 
#define CORRECTION_DIFF (int)(MAX_SPEED*0.15)
#define SEARCH_SPEED 100

#define ROT90DUR 125
#define ROTFTZDUR 200
#define ROT120DUR 130
#define ROT180DUR 250
#define ROT45DUR 70
#define RUNDUR 150
#define RUNDURL 380
#define ROTDIFF (int)(MAX_SPEED*0.7)
#define EDGE_TURN 180
#define EDGE_FULL_TURN 260
#define EDGE_BACK 35
#define MOTOR_BREAK 1
#define MOTOR_ACCEL 127

#define STRAT_FTZ   B1001
