//This file includes all libraries required
//Include library for VL53L0X sensor
#include <VL53L0X.h>
//Inlcude library for i2c
#include <Wire.h>
//include statics.h file to be able to use statics defined
#include "statics.h"

//array of VL53L0X sensor object
VL53L0X sensor[NUM_SENSORS];
//array of sensor addreses
uint8_t sensorAddress[NUM_SENSORS];

int lineL = 0 ;
int lineR = 0;


void setupMotors(){
}
