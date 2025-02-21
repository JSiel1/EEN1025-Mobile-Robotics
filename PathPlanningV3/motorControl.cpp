#include <Arduino.h>
#include "motorControl.h"
#include "settings.h"
#include "innovation.h"

int leftSpeed = 0;
int rightSpeed = 0;

static int previousError = 0;
static int integralError = 0;

unsigned long straightStartTime = 0;
bool isOnStraight = false;

//-------------------------------------------------------------
//-----------------Line Following Logic------------------------
//-------------------------------------------------------------

// Function for line following with PID
void followLine() {
  // Weighted average position
  int position = 0;
  int total    = 0;
  for (int i = 0; i < sensorCount; i++) {
    position += sensorValues[i] * weights[i];
    total    += sensorValues[i];
  }

  if (total != 0) {
    position /= total; // Normalize position
  } else {
    position = 0; // If no line detected
  }

   // Default PID values
  float currentKp = Kp;
  float currentKd = Kd;

  // Check if on a straight line
  bool enteringStraight = (current == 0 && next == 4) || (current == 4 && next == 0) || (current == 2 && next == 3) || (current == 3 && next == 2);
  
  if (enteringStraight && atNode) {
    // Enter straight mode
    isOnStraight = true;
    straightStartTime = millis(); // Record entry time
  }

  if (isOnStraight) {
    currentKp = straightKp;
    currentKd = straightKd;

    Serial.println("on Straight");

    // Stay in straight mode for a short time
    if (millis() - straightStartTime > straightDelay) { // Adjust delay if needed
      isOnStraight = false;
    }
  }


  // PID error
  int error  = -position;
  int derivativeError = error - previousError;
  previousError = error;
  integralError += error;

  float pidValue = (currentKp * error) + (Ki * integralError) + (currentKd * derivativeError);

  //// Middle sensor correction
  //if (sensorValues[2] < whiteThreshold) { 
  //  pidValue /= 2;  // Reduce corrections when middle sensor sees the line
  //}

  // if outer sensor detected
  if (sensorValues[0] < whiteThreshold) {
    leftSpeed  = 0;
    rightSpeed = pivotSpeed;
  }
  else if (sensorValues[4] < whiteThreshold) {
    leftSpeed  = pivotSpeed;
    rightSpeed = 0;
  }
  else {
    if (isOnStraight) {
      leftSpeed  = straightSpeed - pidValue;
      rightSpeed = straightSpeed + pidValue;
    } else {
      leftSpeed  = baseSpeed - pidValue;
      rightSpeed = baseSpeed + pidValue;
    }
  }

  if (isOnStraight) {
    leftSpeed  = constrain(leftSpeed, 0, straightSpeed);
    rightSpeed = constrain(rightSpeed, 0, straightSpeed);
    switchDRS(0);
    obstacleThreshold = 3000;
  } else {
    leftSpeed  = constrain(leftSpeed, 0, constrainSpeed);
    rightSpeed = constrain(rightSpeed, 0, constrainSpeed);
    switchDRS(1);
    obstacleThreshold = 2500;
  }

  driveMotor(leftSpeed - motorOffset, rightSpeed);
}

//-------------------------------------------------------------
//-----------------Motor Control Logic-------------------------
//-------------------------------------------------------------

// Motor drive function
void driveMotor(int left, int right) {
  left -= motorOffset;

  if (left >= 0) {
    digitalWrite(motor1Phase, LOW);
  } else {
    digitalWrite(motor1Phase, HIGH);
    left = -left;
  }

  if (right > 0) {
    digitalWrite(motor2Phase, LOW);
  } else {
    digitalWrite(motor2Phase, HIGH);
    right = -right;
  }

  analogWrite(motor1PWM, left);
  analogWrite(motor2PWM, right);
}

// function to turn left
void left() {
  driveMotor(turnSpeed, -turnSpeed);    // Rotate in place
  delay(turningTime);                   // Adjust delay for a 180-degree turn
  driveMotor(80, 80);
  delay(forwardDelay);
  return;
}

// function to do a 180 degree turn
void reverse() {
  driveMotor(turnSpeed, -turnSpeed);
  delay(rotationTime);
  driveMotor(80, 80);
  delay(forwardDelay);
}

// function to turn right
void right() {
  driveMotor(-turnSpeed, turnSpeed);    // Rotate in place
  delay(turningTime);                   // Adjust delay for a 180-degree turn
  driveMotor(80, 80);
  delay(forwardDelay);
  return;

}
