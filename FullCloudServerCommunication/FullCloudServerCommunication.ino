/*************************************************
* File Name:        [FullCloudCommunication.ino]
* Description:      [Full PID, path following and cloud communication with obstacle sensor]
* Author:           [Group 14]
* Created On:       [29/01/2025]
* Last Modified On: [30/01/2025]
* Version:          [1.1]
* Last Changes:     [Fixed Issues with Virtual node looping]
*************************************************/

#include <WiFi.h>
#include <HTTPClient.h>

// Buffer Size
#define BUFSIZE 512

// Motor pins
#define motor1PWM 37  // Left motor enable (PWM)
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor enable (PWM)
#define motor2Phase 36  // Right motor phase
#define stopSensor 1    //obbstacle detection sensor

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 220; // Base speed for the motors (0–255)

//Node detection settings
const int forwardDelay = 50;   // Time to move across line slightly
const int stopDelay = 400;     // Stopping Time at node
const int rotationTime = 630;   // Time to turn 180 degrees
const int turningTime = 350;    // Time to make a 90 degree turn 

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password

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
float Kp = 0.62; // Proportional gain (0.35)
float Ki = 0.00001;  // Integral gain (set to 0.00001 initially)
float Kd = 0.25;  // Derivative gain   (0.2)

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

// Server details
const char *serverIP = "3.250.38.184"; // Server IP address
const int serverPort = 8000;          // Server port
const char *teamID = "rhtr2655";      // Replace with your team's ID

bool isRunning = false;

// Position Variables
int startingPosition = 0;  // Initial position of the robot
int currentPosition = startingPosition;   // Track the robot's current position
int nextPosition = 0;
int originalDestination = -1;
int lastPosition = -1;
bool forwardDirection = true;   //Start with forward direction

// Wifi class
WiFiClient client;

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

  // Start serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectToWiFi();

  //delay before starting 
  delay(1000);
}

void loop() {
  readLineSensors();
  followPath();
  followLine();
}

//-------------------------------------------------------------
//-----------------Cloud Server Communication------------------
//-------------------------------------------------------------

// Connect to wifi
void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  // Wait for the connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to send current position 
int sendPosition(int position) {
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed!");
    return -1; // Indicate failure
  }

  String postBody = "position=" + String(position);

  // Send HTTP POST request
  client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println(); // End headers
  client.println(postBody); // Send body

  // Read response
  String response = "";
  while (client.connected() || client.available()) {
    if (client.available()) {
      response = client.readString();
      break;
    }
  }

  client.stop(); // Close connection

  // Debug print the full response
  Serial.println("Full Server Response:");
  Serial.println(response);

  // Extract the HTTP status code
  int statusCode = getStatusCode(response);
  if (statusCode != 200) {
      Serial.println("Error: Failed to retrieve next position. HTTP Status: " + String(statusCode));
      return -1;
  }

  // Extract the response body (which is the next position)
  String body = getResponseBody(response);
  body.trim(); // Remove any extra whitespace

  Serial.print("Next Position: ");
  Serial.println(body);

  if (body.equals("Finished")) {
     Serial.println("Final destination reached.");
      return -2;
  }

  return body.toInt(); // Convert to integer and return
}

// Function to read the HTTP response
String readResponse() {
  char buffer[BUFSIZE];
  memset(buffer, 0, BUFSIZE);
  client.readBytes(buffer, BUFSIZE);
  String response(buffer);
  return response;
}

// Function to get the status code from the response
int getStatusCode(String& response) {
  String code = response.substring(9, 12);
  return code.toInt();
}

// Function to get the body from the response
String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  String body = response.substring(split + 4, response.length());
  body.trim();
  return body;
}

//-------------------------------------------------------------
//-----------------Line Following Logic------------------------
//-------------------------------------------------------------

// Function for line following with PID
void followLine() {
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

//-------------------------------------------------------------
//-----------------Motor Control Logic-------------------------
//-------------------------------------------------------------

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

// function to turn left
void left() {
  driveMotor(baseSpeed, -baseSpeed);    // Rotate in place
  delay(turningTime);                   // Adjust delay for a 180-degree turn
  driveMotor(80, 80);
  delay(forwardDelay);
  return;
}

// function to do a 180 degree turn
void reverse() {
  driveMotor(baseSpeed, -baseSpeed);
  delay(rotationTime);
  driveMotor(80, 80);
  delay(forwardDelay);
}

// function to turn right
void right() {
  driveMotor(-baseSpeed, baseSpeed);    // Rotate in place
  delay(turningTime);                   // Adjust delay for a 180-degree turn
  driveMotor(80, 80);
  delay(forwardDelay);
  return;

}

//-------------------------------------------------------------
//---------------Obstacle Detection Logic----------------------
//-------------------------------------------------------------

// Detect an obstacle in front of the sensor
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

//-------------------------------------------------------------
//-----------------Path Following Logic------------------------
//-------------------------------------------------------------

void followPath(){
  if (!detectNode()){
    return;
  }

  // Stop robot at line and move slightly over
  driveMotor(0, 0); // Stop the robot
  driveMotor(80, 80); // Drive forward at low speed
  delay(forwardDelay);          // Move slightly forward to cross the line
  driveMotor(0, 0);   // Stop again
  delay(stopDelay);         // Wait for 1 second before resuming

  //Dont update position if node is 0 and 6 and 7
  if (currentPosition != 6 && currentPosition != 7) {
    nextPosition = sendPosition(currentPosition);

    // Handle finished logic
    if (nextPosition == -2) {
      Serial.println("Destination Reached");
      driveMotor(0, 0);       //stop mobot
      // Keep mobot stopped
      while (true) {
        delay(1000);
      }
    }
  } else if (currentPosition == 6 || currentPosition == 7) {
    nextPosition = originalDestination;
  }

  bool virtualNode = requiresVirtualNode(currentPosition, nextPosition);

  if (virtualNode) {
    originalDestination = nextPosition;
    nextPosition = getVirtualNode(currentPosition, nextPosition);
    Serial.println("Original Destination saved and Virtual Node inserted");
  } else {
    // Debug Remove After
    Serial.println("Direct Path found, No virtual node needed");          ///////REMOVE AFER
  }

  int direction = getDynamicDirection(currentPosition, nextPosition, lastPosition);

  // Check if finished

  if (direction != -1) {
    choosePath(direction);
    lastPosition = currentPosition;
    currentPosition = nextPosition;
  } else {
    // Debug
    Serial.println("Error: No Valid Path found between " + String(currentPosition) + " -> " + String(nextPosition));
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

bool requiresVirtualNode(int current, int next) {
  // Check if the next node is not directly connected to the current node
  for (int i = 0; i < 3; i++) {
    if (adjacencyList[current][i] == next) {
      return false;  // Direct connection exists, no virtual node needed
    }
  }
  return true;  // No direct path, so a virtual node is needed
}

int getVirtualNode(int current, int next) {
  // Check for a virtual node that connects both current and next position
  for (int i = 6; i <= 7; i++) {  // Virtual nodes are 6 and 7
    bool connectsCurrent = false;
    bool connectsNext = false;

    for (int j = 0; j < 3; j++) {
      if (adjacencyList[i][j] == current) connectsCurrent = true;
      if (adjacencyList[i][j] == next) connectsNext = true;
    }

    if (connectsCurrent && connectsNext) {
      return i;  // Found a valid virtual node transition
    }
  }
  return next;  // No virtual node needed, return the original next position
}

void choosePath(int direction){
  switch (direction) {
    case 0:                       // reverse
      reverse();                         // 180-degree turn
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
