#ifndef motorControl_H
#define motorControl_H

#include "PathPlanning.h"

// external variables
extern int sensorValues[5];
extern const int sensorCount;
extern const int whiteThreshold;

extern unsigned long drsStartTime;
extern bool drsActive;
extern bool drsPending;

extern int current;
extern int next;

// external functions
extern void switchDRS(bool DRSPosition);

//file specific variables

// Motor pins
const int motor1PWM = 37;  // Left motor enable (PWM)
const int motor1Phase = 38;  // Left motor phase
const int motor2PWM = 35;  // Right motor enable (PWM)
const int motor2Phase = 36;  // Right motor phase

const int weights[5] = {-2000, -1240, 0, 1000, 2000};

extern int leftSpeed;
extern int rightSpeed;

// Function for line following with PID
void followLine();

// Motor drive function
void driveMotor(int left, int right);

// function to turn left
void left();

// function to do a 180 degree turn
void reverse();

// function to turn right
void right();


#endif