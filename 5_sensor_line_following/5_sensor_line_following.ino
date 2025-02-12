

#include <Arduino.h>

/*--------------- Pin Definitions ---------------*/
#define motor1PWM   37   // Left motor enable (PWM)
#define motor1Phase 38   // Left motor phase
#define motor2PWM   35   // Right motor enable (PWM)
#define motor2Phase 36   // Right motor phase

// Line sensor pins
const int IR_PINS[] = {4, 7, 6, 5, 15};  // {Left Outer, Left Inner, Middle, Right Inner, Right Outer}
const int sensorCount = 5;
int weights[5]      = {-2000, -1000, 0, 1000, 2000};  // Assign weight to each sensor
int sensorValues[5] = {0, 0, 0, 0, 0}; 

/*--------------- Robot Parameters ---------------*/
int leftSpeed  = 0;
int rightSpeed = 0;

// Adjustable speeds
int baseSpeed        = 200;  // Normal driving speed
int pivotSpeed       = 255;  // Speed for sharp turns
int middleCorrection = 50;   // Small correction if middle sensor detects line

// IR thresholds
const int whiteThreshold = 270; 

/*--------------- PID Parameters ---------------*/
float Kp = 0.4;  // Proportional Gain
float Ki = 0.00; // Integral Gain
float Kd = 3.5; // Derivative Gain
int previousError   = 0;
int integralError   = 0;

/*----------------------------------------------------------
 *                       SETUP
 *--------------------------------------------------------*/
void setup() {
  Serial.begin(115200);

  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  
  for (int i = 0; i < sensorCount; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  Serial.println("PID Line Follower Initialized");
}

/*----------------------------------------------------------
 *                       LOOP
 *--------------------------------------------------------*/
void loop() {
  readLineSensors();
  followLine();
  delay(5); 
}

/*----------------------------------------------------------
 *               READ LINE SENSORS
 *--------------------------------------------------------*/
void readLineSensors() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
  }
}

/*----------------------------------------------------------
 *               FOLLOW LINE (PID)
 *--------------------------------------------------------*/
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
  int dError = error - previousError;
  previousError = error;
  integralError += error;

  float pidOut = (Kp * error) + (Ki * integralError) + (Kd * dError);

  // Middle sensor correction
  if (sensorValues[2] < whiteThreshold) { 
    pidOut /= 2;  // Reduce corrections when middle sensor sees the line
  }

  // 
  if (sensorValues[0] < whiteThreshold) {
   
    leftSpeed  = 0;
    rightSpeed = pivotSpeed;
  }
  else if (sensorValues[4] < whiteThreshold) {
   
    leftSpeed  = pivotSpeed;
    rightSpeed = 0;
  }
  else {
    
    leftSpeed  = baseSpeed - pidOut;
    rightSpeed = baseSpeed + pidOut;
  }

  leftSpeed  = constrain(leftSpeed, 0, 250);
  rightSpeed = constrain(rightSpeed, 0, 250);


  driveMotors(leftSpeed, rightSpeed);
}


void driveMotors(int leftVal, int rightVal) {
  if (leftVal >= 0) {
    digitalWrite(motor1Phase, LOW);
  } else {
    digitalWrite(motor1Phase, HIGH);
    leftVal = -leftVal; 
  }

  if (rightVal >= 0) {
    digitalWrite(motor2Phase, LOW);
  } else {
    digitalWrite(motor2Phase, HIGH);
    rightVal = -rightVal;
  }

  analogWrite(motor1PWM, leftVal);
  analogWrite(motor2PWM, rightVal);
}
