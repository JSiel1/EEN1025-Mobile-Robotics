// Pre-Defined variables ( Don't merge)
#define motor1PWM 37
#define motor1Phase 38
#define motor2PWM 35
#define motor2Phase 36

const int turningTime = 300;
const int baseSpeed = 120;

const int whiteThreshold = 500;
const uint8_t IR_Pins[] = {4, 7, 5, 15};
const int sensorCount = 4;
//-----------------------------------------------------------

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
const int path[] = {0, 6, 2, 3};
const int pathLength = sizeof(path) / sizeof(path[0]);
int currentPosition = path[0];
int currentPathIndex = 0;

bool forwardDirection = true;   //Start with forward direction

//Sensor variables
int sensorValues[sensorCount];    //Senor readings

//-----------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  readLineSensors();
  followPath();
  //lineFollow();
}
//-----------------------------------------------------------------------

// Function to get the next direction (Reverse=0, Forward=1, Left=2)
int getNextDirection(int currentNode, int targetPosition) {
  for (int direction = 0; direction < 3; direction++) {
    if (adjacencyList[currentNode][direction] == targetPosition) {
      return direction; // Return the direction index
    }
  }
  return -1; // Invalid path
}

// Read current Sensor values
void readLineSensors() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(IR_Pins[i]); 
    //debug
    //Serial.print("Sensor ");
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(sensorValues[i]);
  }
}

//Move mobot based on direction
void choosePath(int direction){
  switch (direction) {
    case 0:                       // reverse
      left(); left();                         // 180-degree turn
      forwardDirection = !forwardDirection;   //Switch direction
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
      while (true) {
        driveMotor(0, 0);
      }
    }

    // Get next position and direction
    int nextPosition = path[currentPathIndex + 1];
    int direction = getNextDirection(currentPosition, nextPosition);

    // Check if Direction is valid
    if (direction != -1) {
      choosePath(direction);
      currentPosition = nextPosition;
      currentPathIndex++;
    } else {
      Serial.println("Error: No valid path found");
    }
  }
}

//------------------------------------------------------------------------------
//Already Defined Functions (Don't merge)

bool detectNode() {
  int whiteCount = 0;
  for (int i = 0; i < sensorCount && whiteCount < 3; i++) {
    if (sensorValues[i] < whiteThreshold) {
      whiteCount++;
    }
  }
  return (whiteCount >= 3); // Node detected if 3 or more sensors see white
}

void left() {
  driveMotor(-baseSpeed, baseSpeed); // Rotate in place
  delay(turningTime);
}

void right() {
  driveMotor(baseSpeed, -baseSpeed); // Rotate in place
  delay(turningTime);
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