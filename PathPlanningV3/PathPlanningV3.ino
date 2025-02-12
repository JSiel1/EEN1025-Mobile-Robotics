/*************************************************
* File Name:        [PathPlanningV3.ino]
* Description:      []
* Author:           [Group 14]
* Created On:       [11/02/2025]
* Last Modified On: [12/02/2025]
* Version:          [3]
* Last Changes:     [Changed the way detect node is handled. added bool atNode and check for node directly on sensor read
Added extra cases for node handling at junction 7,
Added parking; updated obstacle detection and check if current node = 5 ]
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
#define obstacleSensor 1    //obbstacle detection sensor

#define redPin 10
#define greenPin 11
#define bluePin 12

#define DRSPin 9

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
const int baseSpeed = 160;   // Base speed for the motors (0–255) 200
const int turnSpeed = 160;   // Turning Speed 190

//Node detection settings
const int forwardDelay = 60;   // Time to move across line slightly
const unsigned long stopDelay = 50;     // Stopping Time at node
const int rotationTime = 900;   // Time to turn 180 degrees
const int turningTime = 450;    // Time to make a 90 degree turn 

// PID parameters
const float Kp = 0.35; // Proportional gain (0.35)
const float Ki = 0.0;  // Integral gain (set to 0.00001 initially)
const float Kd = 0.22;  // Derivative gain   (0.2)

//Line detection Sensitivity
const int whiteThreshold = 270; // Around 200 for white line. Greater means higher sensitivity
const int blackThreshold = 2700; // Around 2700 for black surface
const int obstacleThreshold = 2500;  //Obstacle Sensitivity. Higher means further sensing

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password
//const char *password = "overtechnicality7petrophilous";   // Secondary ESP32 

// IR sensor pins (only outermost sensors are used)
const int IR_PINS[] = {4, 7, 5, 15}; // 2 sensors on the left, 2 on the right
const int sensorCount = 4;
const int weights[] = {-2000, -1000, 1000, 2000};

int sensorValues[sensorCount];

//LED variables
unsigned long previousMillis = 0;
int colorIndex = 0;
float colourBrightness = 0.5;

//DRS variables
const int PIDThreshold = 20;

float Pvalue = 0;
float Ivalue = 0;
float Dvalue = 0;
int previousError = 0;


bool atNode = false;
const int nodeCount = 8; // Number of nodes

// Adjacency Matrix
int weightMatrix[nodeCount][nodeCount] = {
  //   0    1     2     3    4    5     6     7
  {    0, INF,  INF,  INF,   1, INF,    3, INF },    // Node 0: connects to 4 and 6
  { INF,    0,  INF,  INF, INF, INF,    1,   3 },    // Node 1: connects to 6 and 7
  { INF,  INF,    0,    1, INF, INF,    2, INF },    // Node 2: connects to 3 and 6
  { INF,  INF,    1,    0, INF, INF,  INF,   2 },    // Node 3: connects to 2 and 7
  {   2,  INF,  INF,  INF,   0, INF,  INF,   1 },    // Node 4: connects to 0 and 7
  { INF,  INF,  INF,  INF, INF,   0,  INF,   1 },    // Node 5: isolated
  {   2,    1,    2,  INF, INF, INF,    0, INF },    // Node 6: junction (nodes 0,1,2)
  { INF,    2,  INF,    2,   1,   1,  INF,   0 }     // Node 7: junction (nodes 1,3,4,5)
};

int path[MAX_PATH_SIZE];  // Final path with virtual nodes
int pathLength = 0;  // Size of the updated path
int updatedPath[MAX_PATH_SIZE];
int updatedPathLength = 0;

int pathIndex = 0;

// Obstacle re-routing variables
int tempPath[MAX_PATH_SIZE];
int tempPathLength = 0;
int reRouteIndex = 0;
bool reRouteActive = false;

int storeWeight = -1;
int storeCurrent = -1;
int storeNext = -1;

// Server details
const char *serverIP = "3.250.38.184"; // Server IP address
const int serverPort = 8000;          // Server port
const char *teamID = "rhtr2655";      // Replace with your team's ID

// Position
bool forwardDirection = true;   //Start with forward direction
int lastNode = -1;

//Route re-writing
String route = "";

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

  pinMode(obstacleSensor, INPUT);
  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_PINS[i], INPUT);
  }

  driveMotor(0,0);
  // initialise LEDS
  setColour(255, 0, 0);
  switchDRS(1);
  
  // Start serial communication
  Serial.begin(115200);

  
  connectToWiFi();        // Connect to Wi-Fi
  route = getRoute();     // Obtain path
  adjustPath();           //Convert string route to path array
  computePath();          //Find shortest path and save to updatedPath arrray

  //delay before starting
  delay(1000);
  setColour(0, 255, 0);
  switchDRS(0);
}

void loop() {
  readLineSensors();
  
  if (!reRouteActive) {
    //Handle path with global path
    processPath(updatedPath, pathIndex, updatedPathLength, false);
  } else {
    //Handle path with temporary path
    processPath(tempPath, reRouteIndex, tempPathLength, true);

    if (reRouteIndex >= tempPathLength - 1) {
      reRouteActive = false;
      Serial.println("Re-route deactivated");

      if (storeWeight != -1) {
        Serial.println("Restoring original path.");
        weightMatrix[storeCurrent][storeNext] = storeWeight;
        weightMatrix[storeNext][storeCurrent] = storeWeight;
        storeWeight = -1;
        storeCurrent = -1;
        storeNext = -1;
      }
    }
  }
  
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

  // Wait for a response but only for a short time (non-blocking)
  unsigned long timeout = millis() + stopDelay;  // 100ms max wait time
  while (millis() < timeout) {
    if (client.available()) {
      String response = client.readString();  // Read response
      Serial.println("Full Server Response:");
      Serial.println(response);
      break; // Exit loop after reading response
    }
  }

  client.stop(); // Close connection

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

  //flip drs if PID threshold reached
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

// Read IR sensor values and update array
void readLineSensors(){
  int whiteCount = 0;
  // Read sensor values
  for (int i = 0; i < sensorCount; i++) {
    // Read sensor Values and update array
    sensorValues[i] = analogRead(IR_PINS[i]);
    // Check for node if more than 3 sensors detect white line
    if (sensorValues[i] < whiteThreshold) {
      whiteCount++;
    }
  }
  if (whiteCount >= 3) {
    atNode = true;
    Serial.println("Node detected");
  } else {
    atNode = false;
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

//-------------------------------------------------------------
//---------------Obstacle Detection Logic----------------------
//-------------------------------------------------------------

// Detect an obstacle in front of the sensor
bool detectObstacle() {
  int totalValue = 0;
  int numSamples = 2;

  // Take multiple readings and compute the average
  for (int i = 0; i < numSamples; i++) {
    totalValue += analogRead(obstacleSensor);
    delay(5); // Small delay to allow readings to stabilize
  }


  //totalValue = analogueRead(obstacleSensor);
  //check for error
  //if (totalValue < 500) {
  //  totalValue = analogueRead(aobstacleSensor);
  //}

  int avgSensorValue = totalValue / numSamples;
  int adjustedValue = 4095 - avgSensorValue;

  Serial.println(adjustedValue);

  // Check distance to obstacle
  if (adjustedValue < obstacleThreshold && adjustedValue > 1500) {
    Serial.println("Obstacle Detected!");
    return true;
  }
  return false;
}


//-------------------------------------------------------------
//-----------------Path Following Logic------------------------
//------------------------------------------------------------

// Get next direction
int getDirection(int currentNode, int lastNode, int nextNode) {
  // At the starting position, assume going straight.
  if (lastNode == -1)
    return (forwardDirection) ? 1 : 0;
    
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
    
  // For nodes with only two connections, if the next node equals the last node, indicates a 180° turn.
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
      if (nextNode == 1) return 0; // back -> node 1
      if (nextNode == 3) {
        forwardDirection = !forwardDirection;
        return 3; // right -> node 3
      }
      if (nextNode == 4) return 2; // left -> node 4
      if (nextNode == 5) return 1; // striaght -> node 5
    } else if (lastNode == 4) {
      if (nextNode == 1) {
        forwardDirection = !forwardDirection; // flip direction
        return 3; // right -> node 1
      }
      if (nextNode == 3) return 1; // straight -> node 3
      if (nextNode == 4) return 0; // back -> node 4
      if (nextNode == 5) return 2; // left -> node 5
    } else if (lastNode == 3) {
      if (nextNode == 1) {
        forwardDirection = !forwardDirection;
        return 2; // left
      }
      if (nextNode == 1) return 2; // left -> node 1
      if (nextNode == 3) return 0; // back -> node 3
      if (nextNode == 4) return 1; // straight -> node 4
      if (nextNode == 5) return 3; // right -> node 5
    }
  }
  return -1; // error: mapping not found
}

// Choose Path based on direction
void choosePath(int direction){
  switch (direction) {
    case 0:                              // reverse
      Serial.println("Turning Around");
      reverse();                         // 180-degree turn
      break;
    case 1:                              // Straight
      Serial.println("Going Straight");
      driveMotor(baseSpeed, baseSpeed);
      delay(forwardDelay);                             
      break;
    case 2:                              // Left
      Serial.println("Turning Left");
      left();
      break;
    case 3:
      Serial.println("Turning Right");
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

// Process path path array and keep track of position 
void processPath(int currentPath[], int &index, int pathLength, bool isTempRoute) {
  int current = currentPath[index];
  int next = currentPath[index + 1];
  
  //print path for debug
  //Serial.print("processing Path: ");
  //for (int j = 0; j < pathLength; j++) {
  //  Serial.print(currentPath[j]);
  //  if ( j >= 0 && j < pathLength-1) Serial.print(" -> ");
  //}
  //Serial.println("");

  // Obstacle detection & temporary re-routing.
  if (!isTempRoute && detectObstacle()) {
    // Update position index if re-routing
    if (index > 0) {
      current = currentPath[index - 1];
      next = currentPath[index];
    } 

    if (!reRoute(current, next)) {
      Serial.println("Error in Re-routing: No alternate route possible.");
      driveMotor(0, 0);
      return;
    }
    return; // Exit and let the next loop iteration process the temporary path
  }

  //Return if not at node
  if (!atNode) {
    return;
  }
  
  driveMotor(0, 0); // Stop the robot
  driveMotor(80, 80); // Drive forward at low speed
  delay(forwardDelay);          // Move slightly forward to cross the line
  driveMotor(0, 0);   // Stop again
  
  //Debug
  delay(500);         // Wait for 1 second before resuming

  if (index < pathLength - 1) {
    // Only send position on nodes and not during re-routing    
    if (current != 6 && current != 7 && isTempRoute) {
      sendPosition(current);
    }

    int turnCode = getDirection(current, lastNode, next);
    Serial.print("At node ");
    Serial.print(current);
    Serial.print(" -> next node ");
    Serial.print(next);
    Serial.print(" : Turn code = ");
    Serial.println(turnCode);
    
    // Perform action based on turn code
    choosePath(turnCode);
    delay(1000);
    
    if (next == 5) {
      while (!detectObstacle()){
        Serial.println("Waiting for wall");

        //drive straight at wall
        driveMotor(170, 180);
      }
      //Updated final position and stop
      driveMotor(0,0);
      sendPosition(5);
    }
    // Increment the global path index to move to the next segment.
    index++;
    lastNode = current;
  } else if (!isTempRoute) {
    // If we have reached the end of the path, indicate completion.
    Serial.println("Finished path.");
    sendPosition(updatedPath[updatedPathLength - 1]);
    // Enter an infinite loop after finishing.
    while (true) {
      driveMotor(0,0);
      setColour(0, 255, 0);
      delay(300);
      setColour(0,0,0);
      delay(300);
    }
  } else {
    Serial.println("Temp Route exit ");
    Serial.print("Temp index: ");
    Serial.println(reRouteIndex);
    Serial.print("Path Length: ");
    Serial.println(tempPathLength);
    delay(2000);    //debug
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

    // If no path was found, print an debug and reset updatedPathLength.
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

  // check if next position is behind starting position
  if (updatedPathLength > 1) {  // Ensure at least two positions exist
    int start = updatedPath[0];
    int next = updatedPath[1];

    // Check if which node has higher weighting to indicate if starting in reverse direction
    if (weightMatrix[next][start] != INF && weightMatrix[start][next] != INF) {
      Serial.print(weightMatrix[next][start]);
      Serial.print(" > ");
      Serial.println(weightMatrix[start][next]);
      if (weightMatrix[next][start] > weightMatrix[start][next]) {
        forwardDirection = false;  // Flip direction
        Serial.println("Starting Direction flipped");
      }
    }
  }

  Serial.print("Shortest path: ");
  for (int i = updatedPathLength - 1; i >= 0; i--) {
    Serial.print(updatedPath[i]);
    if (i > 0) Serial.print(" -> ");
  }
  Serial.println();
}

// re calculate route between current node and next node if obstacle detected.
bool reRoute(int current, int next) {
    Serial.println("Obstacle detected! Calculating temporary route...");


    // stop the robot before the obstacle
    driveMotor(0,0);

    // Backup the original weight before removing the connection
    storeWeight = weightMatrix[current][next];
    storeCurrent = current;
    storeNext = next;

    // Temporarily remove the direct connection
    weightMatrix[current][next] = INF;
    weightMatrix[next][current] = INF;

    // Compute temporary route
    int newPath[MAX_PATH_SIZE];
    int newPathLength = 0;
    shortestPath(next, current, newPath, newPathLength);    //changed this

    if (newPathLength == 0) {
        Serial.println("Re-route Error: No alternate path found! Stopping robot.");
        return false;  // Indicate failure
    }

    // Copy the new path into the tempPath array AND DEBUG
    for (int i = 0; i < newPathLength; i++) {
        tempPath[i] = newPath[i];
    }
    tempPathLength = newPathLength;

    // DEBUG
    Serial.print("Re-route: ");
    for (int j = 0; j < tempPathLength; j++) {
      Serial.print(tempPath[j]);
      if (j < tempPathLength - 1) Serial.print(" -> ");
    }
    Serial.println("");

    // Activate temporary routing mode
    reRouteActive = true;
    reRouteIndex = 0;

    //turn the robot around.
    reverse();

    return true;  // Indicate success
}

//-------------------------------------------------------------
//---------------------------LED-------------------------------
//-------------------------------------------------------------

// Function to set RGB color
void setColour(int r, int g, int b) {
    analogWrite(redPin, r*colourBrightness);
    analogWrite(greenPin, g*colourBrightness);
    analogWrite(bluePin, b*colourBrightness);
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


