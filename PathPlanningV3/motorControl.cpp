#include <Arduino.h>
#include "motorControl.h"
#include "settings.h"

int leftSpeed = 0;
int rightSpeed = 0;

static int previousError = 0;
static int integralError = 0;

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

  // PID error
  int error  = -position;
  int derivativeError = error - previousError;
  previousError = error;
  integralError += error;

  float pidValue = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);

  // Middle sensor correction
  if (sensorValues[2] < whiteThreshold) { 
    pidValue /= 2;  // Reduce corrections when middle sensor sees the line
  }

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
    
    leftSpeed  = baseSpeed - pidValue;
    rightSpeed = baseSpeed + pidValue;
  }


  leftSpeed  = constrain(leftSpeed, 0, constrainSpeed);
  rightSpeed = constrain(rightSpeed, 0, constrainSpeed);

  // DRS CALCULATIONS
  float avgSpeed = (leftSpeed + rightSpeed) / 2;  // Compute average speed

  // Detect speed drop (possible turn)
  if (!drsActive && !drsPending && avgSpeed < drsThreshold) {
    drsPending = true;  
    drsStartTime = millis();  // Start delay timer
  }

  // Activate DRS after delay
  if (drsPending && millis() >= drsDelay) {
    switchDRS(true);
    drsActive = true;
    drsPending = false;
  }

  // Deactivate DRS when speed increases (back on a straight)
  if (drsActive && avgSpeed > drsThreshold + 20) {
    switchDRS(false);
    drsActive = false;
  }


  Serial.print(leftSpeed);
  Serial.print("\t");
  Serial.println(rightSpeed);

  driveMotor(leftSpeed, rightSpeed);
}

//-------------------------------------------------------------
//-----------------Motor Control Logic-------------------------
//-------------------------------------------------------------

// Motor drive function
void driveMotor(int left, int right) {
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
