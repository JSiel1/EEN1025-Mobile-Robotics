/*************************************************
* File Name:        [PathPlanningV3.ino]
* Description:      []
* Author:           [Group 14]
* Created On:       [06/02/2025]
* Last Modified On: [08/02/2025]
* Version:          [3]
* Last Changes:     []
*************************************************/

#include <WiFi.h>

// Buffer Size
#define BUFSIZE 512
#define MAX_PATH_SIZE 20
#define INF 9999

// Motor pins
#define motor1PWM 37  // Left motor enable (PWM)
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor enable (PWM)
#define motor2Phase 36  // Right motor phase
#define stopSensor 1    //obbstacle detection sensor

#define redPin 10
#define greenPin 11
#define bluePin 12

#define DRSPin 9

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 160; // Base speed for the motors (0–255)

//Node detection settings
const int forwardDelay = 50;   // Time to move across line slightly
const int stopDelay = 0;     // Stopping Time at node
const int rotationTime = 630;   // Time to turn 180 degrees
const int turningTime = 350;    // Time to make a 90 degree turn 

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password
//const char *password = "overtechnicality7petrophilous";   // Secondary ESP32 


// IR sensor pins (only outermost sensors are used)
const int IR_PINS[] = {4, 7, 5, 15}; // 2 sensors on the left, 2 on the right
const int sensorCount = 4;
int weights[] = {-2000, -1000, 1000, 2000};
int sensorValues[sensorCount];

//Line detection Sensitivity
const int whiteThreshold = 270; // Around 200 for white line
const int blackThreshold = 2700; // Around 2700 for black surface
const int obstacleThreshold = 1100;  //Obstacle Sensitivity

//LED variables
unsigned long previousMillis = 0;
int colorIndex = 0;

//DRS variables
const int PIDThreshold = 20;

// PID parameters
float Kp = 0.35; // Proportional gain (0.35)
float Ki = 0.0;  // Integral gain (set to 0.00001 initially)
float Kd = 0.2;  // Derivative gain   (0.2)

float Pvalue = 0;
float Ivalue = 0;
float Dvalue = 0;
int previousError = 0;


const int nodeCount = 8; // Number of nodes

// Adjacency Matrix
int weightMatrix[nodeCount][nodeCount] = {
  //   0    1     2     3    4    5     6     7
  {    0, INF,  INF,  INF,   1, INF,    1, INF },    // Node 0: connects to 4 and 6
  { INF,    0,  INF,  INF, INF, INF,    1,   1 },    // Node 1: connects to 6 and 7
  { INF,  INF,    0,    1, INF, INF,    1, INF },    // Node 2: connects to 3 and 6
  { INF,  INF,    1,    0, INF, INF,  INF,   1 },    // Node 3: connects to 2 and 7
  {   1,  INF,  INF,  INF,   0, INF,  INF,   1 },    // Node 4: connects to 0 and 7
  { INF,  INF,  INF,  INF, INF,   0,  INF, INF },    // Node 5: isolated
  {   1,    1,    1,  INF, INF, INF,    0, INF },    // Node 6: junction (nodes 0,1,2)
  { INF,    1,  INF,    1,   1, INF,  INF,   0 }     // Node 7: junction (nodes 1,3,4)
};

// Server details
const char *serverIP = "3.250.38.184"; // Server IP address
const int serverPort = 8000;          // Server port
const char *teamID = "rhtr2655";      // Replace with your team's ID

// Position Variables
int startingPosition = 0;  // Initial position of the robot
int currentPosition = startingPosition;   // Track the robot's current position
int nextPosition = 0;
int lastPosition = -1;

bool forwardDirection = true;   //Start with forward direction

//Route re-writing
String route = "";

int path[MAX_PATH_SIZE];  // Final path with virtual nodes
int pathLength = 0;  // Size of the updated path
int updatedPath[MAX_PATH_SIZE];
int updatedPathLength = 0;

int pathIndex = 0;

// Wifi class
WiFiClient client;

void setup() {
  // Set motor pins as output
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(DRSPin, OUTPUT);

  pinMode(stopSensor, INPUT);
  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_PINS[i], INPUT);
  }

  // Start serial communication
  Serial.begin(115200);
  
  switchDRS(1);   //Flip DRS On

  // Connect to Wi-Fi
  connectToWiFi();

  // Obtain path
  route = getRoute();

  //Convert string route to path array
  adjustPath();

  //Find shortest path and save to updatedPath arrray
  computePath();

  //Serial.print("Shortest path: ");
  //for (int i = updatedPathLength - 1; i >= 0; i--) {
  //  Serial.print(updatedPath[i]);
  //  if (i > 0) Serial.print(" -> ");
  //}
  //Serial.println();

  //delay before starting 
  delay(1000);

  switchDRS(0);   //Flip drs off
}

void loop() {
  readLineSensors();
  processPath();
  followLine();
  //rainbowFade(10);
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
void sendPosition(int position) {
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed!");
    return;
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
  //String response = "";
  //while (client.connected() || client.available()) {
  //  if (client.available()) {
  //    response = client.readString();
  //    break;
  //  }
  //}

  client.stop(); // Close connection

  // Debug print the full response
  //Serial.println("Full Server Response:");
  //Serial.println(response);

  //// Extract the HTTP status code
  //int statusCode = getStatusCode(response);
  //if (statusCode != 200) {
  //    Serial.println("Error: Failed to retrieve next position. HTTP Status: " + String(statusCode));
  //    return;
  //}
}

// Function to read Route
String getRoute() {
  // Connect to server
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed!");
  return "-1";
  }

  //Make GET request 
  client.println("GET /api/getRoute/" + String(teamID) + " HTTP/1.1");
  client.println("Connection: close");
  client.println();
  
  String response  = "";

  // Save response when available
  while (client.connected() || client.available()) {
    if (client.available()) {
      response = client.readString();
      Serial.println(response);
      break;
    }
  }

  //Get error code
  int statusCode = getStatusCode(response);
  if (statusCode != 200) {
      Serial.println("Error: Failed to retrieve next position. HTTP Status: " + String(statusCode));
      return "-1";
  }

  client.stop();          // Stop connection
  return getResponseBody(response);     // Return full Path
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

  if (PIDvalue > PIDThreshold){
    switchDRS(1);
  } else {
    switchDRS(0);
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
//------------------------------------------------------------

// Get next direction
int getDirection(int currentNode, int lastNode, int nextNode) {
  // At the starting position, assume going straight.
  if (lastNode == -1)
    return 1;
    
  // Count valid neighbors for the current node.
  int validCount = 0;
  for (int j = 0; j < nodeCount; j++) {
    if (j == currentNode)
      continue;
    if (weightMatrix[currentNode][j] != INF)
      validCount++;
  }
  
  // If more than two connections exist, treat it as a junction.
  if (validCount > 2)
    return getJunctionDirection(currentNode, lastNode, nextNode);
    
  // For nodes with only two connections, if the next node equals the last node,
  // that indicates a 180° turn.
  if (nextNode == lastNode)
    return 0;
    
  // Otherwise, go straight.
  return 1;
}

// Handle directions for junctions
int getJunctionDirection(int currentNode, int lastNode, int nextNode) {
  // Node 6 mapping: connected to nodes 0, 1, and 2.
  if (currentNode == 6) {
    if (lastNode == 1) {
      if (nextNode == 2) return 2; // left -> node 2
      if (nextNode == 0) return 3; // right -> node 0
      if (nextNode == 1) return 0; // back (180° turn)
    } else if (lastNode == 2) {
      if (nextNode == 0) return 1; // straight -> node 0
      if (nextNode == 1) {
        forwardDirection = !forwardDirection;
        return 3; // right (with flip)
      }
      if (nextNode == 2) return 0; // back
    } else if (lastNode == 0) {
      if (nextNode == 1) return 2; // left -> node 1
      if (nextNode == 2) return 1; // straight -> node 2
      if (nextNode == 0) return 0; // back
    }
  }
  
  // Node 7 mapping: connected to nodes 1, 3, and 4.
  if (currentNode == 7) {
    if (lastNode == 1) {
      if (nextNode == 4) return 2; // left -> node 4
      if (nextNode == 3) return 3; // right -> node 3
      if (nextNode == 1) return 0; // back
    } else if (lastNode == 4) {
      if (nextNode == 1) return 3; // right -> node 1
      if (nextNode == 3) return 2; // left -> node 3
      if (nextNode == 4) return 0; // back
    } else if (lastNode == 3) {
      if (nextNode == 1) {
        forwardDirection = !forwardDirection;
        return 2; // left (with flip)
      }
    }
  }
  return -1; // error: mapping not found
}


void choosePath(int direction){
  switch (direction) {
    case 0:                              // reverse
      reverse();                         // 180-degree turn
      break;
    case 1:                              // Straight
      driveMotor(baseSpeed, baseSpeed);
      delay(forwardDelay);                             
      break;
    case 2:                              // Left
      left();
      break;
    case 3:
      right();                          // Right
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


// Original processPath function with correct path handling
void processPath() {
  // Check if there is a next segment.
  if (!detectNode()) {
    return;
  }

  if (pathIndex < updatedPathLength - 1) {
    int current = updatedPath[pathIndex];
    int next = updatedPath[pathIndex + 1];
    int lastNode = (pathIndex == 0) ? -1 : updatedPath[pathIndex - 1];

    if (current != 6 && current != 7) {
      sendPosition(current);
    }

    int turnCode = getDirection(current, lastNode, next);
    Serial.print("At node ");
    Serial.print(current);
    Serial.print(" -> next node ");
    Serial.print(next);
    Serial.print(" : Turn code = ");
    Serial.println(turnCode);

    // Perform Action based on direction
    choosePath(turnCode);

    // Increment the global path index to move to the next segment.
    pathIndex++;
  } else {
    // If we have reached the end of the path, indicate completion.
    Serial.println("Finished path. Waiting...");
    // Enter an infinite loop to halt further processing.
    while (true) {
      setColour(0,255,0);
      delay(300);
      setColour(0,0,0);
      delay(300);
    }
  }
}

//-------------------------------------------------------------
//----------------String-To-Array-Conversion-------------------
//-------------------------------------------------------------

// convert string route to global array
void adjustPath() {
  // Ensure there is a valid route to process.
  if (route.length() == 0) {
    Serial.println("No valid route to process.");
    return;
  }
  
  // Step 1: Count the number of segments in the route and determine the required size
  pathLength = 1;
  for (int i = 0; i < route.length(); i++) {
    if (route[i] == ',') {
      pathLength++;
    }
  }

  // Check if the pathLength exceeds the max size
  if (pathLength > MAX_PATH_SIZE) {
    Serial.println("Error: Path length exceeds MAX_PATH_SIZE.");
    return;
  }

  // Step 2: Parse the route string into the array
  int index = 0;
  char* routeCopy = strdup(route.c_str());
  char* token = strtok(routeCopy, ",");
  while (token != nullptr && index < pathLength) {
    path[index++] = atoi(token);
    token = strtok(nullptr, ",");
  }
  free(routeCopy);
  
  // Step 3: Print the parsed path for debugging
  Serial.println("Route parsed successfully into an array.");
  Serial.print("Path: ");
  for (int i = 0; i < pathLength; i++) {
    Serial.print(path[i]);
    if (i < pathLength - 1) {
      Serial.print(" -> ");
    }
  }
  Serial.println();
}

//-------------------------------------------------------------
//-----------------Shortest Path Calculation-------------------
//-------------------------------------------------------------

// Find min distance between nodes
int findMinDistance(int distances[], bool visited[]) {
  int minDistance = INF;
  int minIndex = -1;

  for (int i = 0; i < nodeCount; i++) {
    if (!visited[i] && distances[i] < minDistance) {
      minDistance = distances[i];
      minIndex = i;
    }
  }
  return minIndex;
}

//Find Dijkstra shortest path and update path array 
void shortestPath(int startNode, int endNode, int tempPath[], int &tempPathLength) {
  int distances[nodeCount];
  bool visited[nodeCount];
  int previous[nodeCount];

  for (int i = 0; i < nodeCount; i++) {
    distances[i] = INF;
    visited[i] = false;
    previous[i] = -1;
  }
  distances[startNode] = 0;

  for (int i = 0; i < nodeCount - 1; i++) {
    int currentNode = findMinDistance(distances, visited);
    if (currentNode == -1) break;
    visited[currentNode] = true;

    for (int neighbor = 0; neighbor < nodeCount; neighbor++) {
      if (weightMatrix[currentNode][neighbor] != INF && !visited[neighbor]) {
        int newDistance = distances[currentNode] + weightMatrix[currentNode][neighbor];
        if (newDistance < distances[neighbor]) {
          distances[neighbor] = newDistance;
          previous[neighbor] = currentNode;
        }
      }
    }
  }
  
  tempPathLength = 0;
  for (int at = endNode; at != -1; at = previous[at]) {
    tempPath[tempPathLength++] = at;
  }
  
  if (distances[endNode] == INF) {
    Serial.println("No path found.");
    tempPathLength = 0;
  }
}

// calculate shortest path for entire route
void computePath() {
  updatedPathLength = 0;
  
  // If there is no or only one node in the route, just copy it over.
  if (pathLength <= 0)
    return;
  if (pathLength == 1) {
    updatedPath[0] = path[0];
    updatedPathLength = 1;
    return;
  }
  
  // Temporary array to hold the shortest path between two nodes.
  int tempPath[MAX_PATH_SIZE];
  int tempPathLength;
  
  // Iterate over each consecutive pair in the global route.
  for (int i = 0; i < pathLength - 1; i++) {
    // Compute the shortest path from path[i] to path[i+1]
    shortestPath(path[i+1], path[i], tempPath, tempPathLength);

    // If no path was found, print an error and reset updatedPathLength.
    if (tempPathLength == 0) {
      Serial.print("No path found between ");
      Serial.print(path[i]);
      Serial.print(" and ");
      Serial.println(path[i + 1]);
      updatedPathLength = 0;
      return;
    }
        
    // append the path
    int startIndex = (i == 0) ? 0 : 1;
    for (int j = startIndex; j < tempPathLength; j++) {
      updatedPath[updatedPathLength++] = tempPath[j];
    }
  }
}

//-------------------------------------------------------------
//---------------------------LED-------------------------------
//-------------------------------------------------------------

// Function to set RGB color
void setColour(int r, int g, int b) {
    analogWrite(redPin, r);
    analogWrite(greenPin, g);
    analogWrite(bluePin, b);
}

// Function to generate rainbow colors
void rainbowFade(int wait) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= wait) {
        previousMillis = currentMillis;

        int r = sin((colorIndex * 3.14159 / 128) + 0) * 127 + 128;
        int g = sin((colorIndex * 3.14159 / 128) + 2.09439) * 127 + 128;
        int b = sin((colorIndex * 3.14159 / 128) + 4.18878) * 127 + 128;

        setColour(r, g, b);

        colorIndex++;
        if (colorIndex >= 256) colorIndex = 0; // Reset after full cycle
    }
}

//-------------------------------------------------------------
//---------------------------DRS-------------------------------
//-------------------------------------------------------------

void switchDRS(bool DRSPosition){
  if (DRSPosition) {
    digitalWrite(DRSPin, HIGH);
  } else {
    digitalWrite(DRSPin, LOW);
  }
}


// Debug version: Skip detectNode() for testing
void processPathDebug() {
  // This version skips detectNode() to test the rest of the path logic.
  
  if (pathIndex < updatedPathLength - 1) {
    int current = updatedPath[pathIndex];
    int next = updatedPath[pathIndex + 1];
    int lastNode = (pathIndex == 0) ? -1 : updatedPath[pathIndex - 1];
    
    int turnCode = getDirection(current, lastNode, next);
    Serial.print("At node ");
    Serial.print(current);
    Serial.print(" -> next node ");
    Serial.print(next);
    Serial.print(" : Turn code = ");
    Serial.println(turnCode);
    
    // Perform Action based on direction
    choosePath(turnCode);
    
    // Increment the global path index to move to the next segment.
    pathIndex++;
  } else {
    // If we have reached the end of the path, indicate completion.
    Serial.println("Finished path. Waiting...");
    // Enter an infinite loop to halt further processing.
    while (true) {
      Serial.println("Finished");
      delay(2000);
    }
  }
}


