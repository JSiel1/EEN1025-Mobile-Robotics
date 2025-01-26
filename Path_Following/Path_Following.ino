/*************************************************
* File Name:        [Path_Following.ino]
* Description:      [Adding functionality of following designated path]
* Author:           [Group 14]
* Created On:       [26/01/2025]
* Last Modified On: [26/01/2025]
* Version:          [0]
* Last Changes:     []
*************************************************/

// Motor pins
#define motor1PWM 37  // Left motor PWM
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor PWM
#define motor2Phase 36  // Right motor phase
#define stopSensor A0  // Distance sensor 

// IR sensor pins 
const uint8_t IR_Pins[] = {4, 7, 5, 15}; // 2 sensors on the left, 2 on the right
const uint8_t sensorCount = 4;
const int sensorWeights[] = {-2000, -1000, 1000, 2000};

//Line detection Sensitivity
const int whiteThreshold = 270; // Around 200 for white line
const int blackThreshold = 2700; // Around 2700 for black surface
const int obstacleThreshold = 1800;

// PID parameters
float Kp = 0.35; // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.2;  // Derivative gain
int previousError = 0;

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 220; // Base speed for the motors (0–255)

//Node detection settings
const int forwardDelay = 200;
const int stopDelay = 1000;
const int rotationTime = 600;

void setup() {
  // Pin Initialisation
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(stopSensor, INPUT);

  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_Pins[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {

}

void line_following() {
  // Read and debug sensor values
  int sensorValues[sensorCount];
  for (uint8_t i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(IR_Pins[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorValues[i]);
  }

  // Check for node detection
  if (detectNode(sensorValues)) {
    driveMotor(0, 0);           //Stop on the line
    delay(100);                   
    driveMotor(80, 80);
    delay(forwardDelay);          // Move slightly forward to cross the line
    driveMotor(0, 0);
    delay(stopDelay);                  //wait before continuing
    return;
  }

  // Calculate the position of the line
  int position = 0;
  int total = 0;


  for (uint8_t i = 0; i < sensorCount; i++) {
    position += sensorValues[i] * sensorWeights[i];
    total += sensorValues[i];
  }

  // Avoid division by zero and calculate position
  if (total != 0) {
    position /= total;
  } else {
    position = 0;     //Default to centre
  }

  // Calculate error from line
  float error = 0 - position;

  // PID calculations
  float P = error;
  static float I = 0;
  float D = error - previousError;

  I += error;

  float PIDvalue = P*Kp + I*Ki + D*Kd;
  previousError = error;

  // Check outer sensors for sharp turns
  if (sensorValues[0] < whiteThreshold) {
    leftSpeed = 0;
    rightSpeed = baseSpeed;
  } else if (sensorValues[3] < whiteThreshold) {
    leftSpeed = baseSpeed;
    rightSpeed = 0;
  } else {
    leftSpeed = baseSpeed - PIDvalue;
    rightSpeed = baseSpeed + PIDvalue;
  }

  // Drive motors
  driveMotor(leftSpeed, rightSpeed);
}

// Detect node (3 or more sensors detecting white)
bool detectNode(int sensorValues[]) {
  int whiteCount = 0;
  for (int i = 0; i < sensorCount && whiteCount < 3; i++) {
    if (sensorValues[i] < whiteThreshold) {
      whiteCount++;
    }
  }
  return (whiteCount >= 3); // Node detected if 3 or more sensors see white
}

// Motor drive function
void driveMotor(int left, int right) {
  //limit speed
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left > 0) {
    digitalWrite(motor1Phase, LOW);
    analogWrite(motor1PWM, left);
  } else {
    digitalWrite(motor1Phase, HIGH);
    analogWrite(motor1PWM, -left);
  }

  if (right > 0) {
    digitalWrite(motor2Phase, LOW);
    analogWrite(motor2PWM, right);
  } else {
    digitalWrite(motor2Phase, HIGH);
    analogWrite(motor2PWM, -right);
  }
}

void left() {
  driveMotor(-baseSpeed, baseSpeed); // Rotate in place
  delay(300);
  driveMotor(0, 0);                  // Stop after turning
  delay(100);
  return;
}

void right() {
  driveMotor(baseSpeed, -baseSpeed); // Rotate in place
  delay(300);
  driveMotor(0, 0);                  // Stop after turning
  delay(100);
  return;
}

void obstacleDetection(){
  // Debugging: Check stop sensor value
  int stopSensorValue = analogRead(stopSensor);
  Serial.print("Stop Sensor Value: ");
  Serial.println(4095 - stopSensorValue);

  if ((4095 - stopSensorValue) < obstacleThreshold) {
    driveMotor(-baseSpeed, baseSpeed);   //rotate 180 degrees
    delay(rotationTime);
    driveMotor(0, 0);
    delay(100);
    return;
  }
}

void 