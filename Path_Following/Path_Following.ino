/*************************************************
* File Name:        [LineFollowingPID]
* Description:      [Full PID control for line following robot with stopping sensor]
* Author:           [Group 14]
* Created On:       [21/01/2025]
* Last Modified On: [26/01/2025]
* Version:          [2.5]
* Last Changes:     [Added turning time Variable]
*************************************************/

// Motor pins
#define motor1PWM 37  // Left motor enable (PWM)
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor enable (PWM)
#define motor2Phase 36  // Right motor phase
#define stopSensor 1

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 220; // Base speed for the motors (0–255)

// IR sensor pins (only outermost sensors are used)
const int IR_PINS[] = {4, 7, 5, 15}; // 2 sensors on the left, 2 on the right
const int sensorCount = 4;
int weights[] = {-2000, -1000, 1000, 2000};
int sensorValues[sensorCount];

//Line detection Sensitivity
const int whiteThreshold = 270; // Around 200 for white line
const int blackThreshold = 2700; // Around 2700 for black surface
const int obstacleThreshold = 1100;  //Obstacle Sensitivity

// PID parameters
float Kp = 0.5; // Proportional gain (0.35)
float Ki = 0.0;  // Integral gain (set to 0 initially)
float Kd = 0.3;  // Derivative gain   (0.2)

float Pvalue = 0;
float Ivalue = 0;
float Dvalue = 0;
int previousError = 0;

// Adjacency Matrix: -1 means no path
const int nodeCount = 8; // Number of nodes
const int adjacencyList[nodeCount][3] = {
  { 4, 6, -1 },     // Node 0: {Back=-4, Straight=6, Left=NONE}
  { 6, 7, -1 },     // Node 1: {Back=6, Straight=7, Left=NONE}
  { 6, 3, -1 },     // Node 2: {Back=6, Straight=3, Left=NONE}
  { 2, 7, -1 },     // Node 3: {Back=2, Straight=7, Left=NONE}
  { 7, 0, -1 },     // Node 4: {Back=7, Straight=0, Left=NONE}
  { -1, -1, -1 },   // Node 5: {Back=NONE, Straight=NONE, Left=NONE}
  { 0, 2, 1 },      // Node 6: {Back=0, Straight=2, Left=1}
  { 3, 4, 1 }       // Node 7: {Back=3, Straight=4, Left=1}
};

// Path Following vairables
const int path[] = {0, 6, 1, 6, 2, 3, 7, 4, 7, 1};
const int pathLength = sizeof(path) / sizeof(path[0]);
int currentPosition = path[0];
int currentPathIndex = 0;

bool forwardDirection = true;   //Start with forward direction
int lastPosition = -1;

//Node detection settings
const int forwardDelay = 200;   // Time to move across line slightly
const int stopDelay = 400;     // Stopping Time at node
const int rotationTime = 400;   // Time to turn 180 degrees
const int turningTime = 400;    // Time to make a 90 degree turn 


void setup() {
  // Set motor pins as output
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  pinMode(stopSensor, INPUT);
  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_PINS[i], INPUT);
  }

  // Start serial communication for debugging
  Serial.begin(115200);

}

void loop() {

  followPath();
  //obstacleDetection();
  // Perform line following
  followLine();
}

// PID Line following Function
void followLine() {
  // Read and debug sensor values
  readLineSensors();
  // Check for node detection 
  if (detectNode()) {
    driveMotor(0, 0); // Stop the robot
    driveMotor(80, 80); // Drive forward at low speed
    delay(200);          // Move slightly forward to cross the line
    driveMotor(0, 0);   // Stop again
    delay(stopDelay);         // Wait for 1 second before resuming
    return;              // Skip the rest of the loop iteration
  }

  // Calculate the position of the line (weighted average method using outer sensors)
  int position = 0;
  int total = 0;

  for (int i = 0; i < sensorCount; i++) {
    position += sensorValues[i] * weights[i];
    total += sensorValues[i];
  }

  // Avoid division by zero and calculate position
  if (total != 0) {
    position /= total;
  } else {
    position = 0;
  }

  // Calculate error (target is position 0, the center)
  int error = 0 - position;

  // PID calculations
  int P = error;
  static int I = 0;
  int D = error - previousError;

  Pvalue = Kp * P;
  I += error;
  Ivalue = Ki * I;
  Dvalue = Kd * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
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

  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  driveMotor(leftSpeed, rightSpeed);
}

// Detect node (3 or more sensors detecting white)
bool detectNode() {
  int whiteCount = 0;
  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] < whiteThreshold) {
      whiteCount++;
    }
  }
  return (whiteCount >= 3); // Node detected if 3 or more sensors see white
}

// Read IR sensor values and update array
void readLineSensors(){
  // Read and debug sensor values
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
  }
}

// Motor drive function
void driveMotor(int left, int right) {
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
  driveMotor(baseSpeed, -baseSpeed); // Rotate in place
  delay(turningTime);                        // Adjust delay for a 180-degree turn
  return;
}

void right() {
  driveMotor(-baseSpeed, baseSpeed); // Rotate in place
  delay(turningTime);                        // Adjust delay for a 180-degree turn
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

void choosePath(int direction){
  switch (direction) {
    case 0:                       // reverse
      left(); left();                         // 180-degree turn
      break;
    case 1:                                   // Straight
      driveMotor(baseSpeed, baseSpeed);
      delay(500);                             // Adjust the delay based on distance
      break;
    case 2:                                   // Left
      left();
      break;
    case 3:
      right();
      if (forwardDirection) {
        forwardDirection = false;
      }
      break;
    default:
      Serial.println("Error: Direction Invalid");
      driveMotor(0, 0);
      break;
  }
}

void followPath(){
  if (detectNode()){
    driveMotor(0, 0);           //Stop on the line

    //Check if mobot is at the end of path
    if (currentPathIndex >= pathLength - 1) {
      Serial.println("Path complete");
      driveMotor(-220, 220);
      delay(1000);
      while (true) {
        driveMotor(0, 0);
      }
    }

    // Get next position and direction
    int nextPosition = path[currentPathIndex + 1];
    int direction = getDynamicDirection(currentPosition, nextPosition, lastPosition);

    // Check if Direction is valid
    if (direction != -1) {
      choosePath(direction);
      lastPosition = currentPosition;
      currentPosition = nextPosition;
      currentPathIndex++;
    } else {
      Serial.println("Error: No valid path found");
    }
  } else {
    return;
  }
}

int getDynamicDirection(int currentNode, int targetPosition, int lastPosition) {
  // Handle dynamic mapping for Node 6
  if (currentNode == 6) {
    if (lastPosition == 1) {
      // Entering Node 6 from Node 1
      if (targetPosition == 2) return 2; // Left -> Node 2
      if (targetPosition == 0) {
        forwardDirection == !forwardDirection;
        return 3; // Right -> Node 0
      }
      if (targetPosition == 1) return 0; // Back -> Node 1
    } else if (lastPosition == 2) {
      // Entering Node 6 from Node 2
      if (targetPosition == 0) return 1; // Straight -> Node 0
      if (targetPosition == 1) return 3; // Right -> Node 1
      if (targetPosition == 2) return 0; // Back -> Node 2
    } else if (lastPosition == 0) {
      // Entering Node 6 from Node 0
      if (targetPosition == 1) return 2; // Left -> Node 1
      if (targetPosition == 2) return 3; // Right -> Node 2
      if (targetPosition == 0) return 0; // Back -> Node 0
    }
  }
  
  // Handle dynamic mapping for Node 7
  if (currentNode == 7) {
    if (lastPosition == 1) {
      // Entering Node 7 from Node 1
      if (targetPosition == 5) return 1; // Straight -> Node 5
      if (targetPosition == 4) return 2; // Left -> Node 4
      if (targetPosition == 1) return 0; // Back -> Node 1
      if (targetPosition == 3) {
        return 3; // right -> Node 3
      }
    } else if (lastPosition == 5) {
      // Entering Node 7 from Node 5
      if (targetPosition == 4) return 3; // Right -> Node 4
      if (targetPosition == 1) return 2; // Left -> Node 1
      if (targetPosition == 5) return 0; // Back -> Node 5
    } else if (lastPosition == 4) {
      // Entering Node 7 from Node 4
      if (targetPosition == 1) return 3; // Right -> Node 1
      if (targetPosition == 5) return 2; // Left -> Node 5
      if (targetPosition == 4) return 0; // Back -> Node 4
    }
  }

  // Default case for non-junction nodes or when no special handling is needed
  for (int direction = 0; direction < 3; direction++) {
    if (adjacencyList[currentNode][direction] == targetPosition) {
      if (targetPosition == lastPosition) return 0;
      return forwardDirection ? direction : (direction == 0 ? 1 : (direction == 1 ? 0 : direction));
    }
  }

  return -1; // Invalid path
}