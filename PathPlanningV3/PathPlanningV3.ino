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
const int baseSpeed = 120;   // Base speed for the motors (0–255) 200
const int turnSpeed = 110;   // Turning Speed for 90 degree turns 190 basseSpeed - 10

int pivotSpeed = 140;  // Speed for outside sensor turns (255) baseSpeed + 20 
int middleCorrection = 50;   // Small correction if middle sensor detects line
int constrainSpeed = 170; 

// PID parameters
const float Kp = 0.5; // Proportional gain (0.7)
const float Ki = 0.00;  // Integral gain (set to 0.00001 initially)
const float Kd = 4.25;  // Derivative gain   (4.25)

//Node detection settings
const int forwardDelay = 80;   // Time to move across line slightly
const unsigned long stopDelay = 50;     // Stopping Time at node
const int rotationTime = 800;   // Time to turn 180 degrees
const int turningTime = 400;    // Time to make a 90 degree turn 

//Line detection Sensitivity
const int whiteThreshold = 400; // Around 200 for white line. Greater means higher sensitivity
const int obstacleThreshold = 2500;  //Obstacle Sensitivity. Higher means further sensing

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password
//const char *password = "overtechnicality7petrophilous";   // Secondary ESP32 


// IR sensor pins (only outermost sensors are used)
const int sensorCount = 5;
const int IR_PINS[sensorCount] = {4, 7, 6, 5, 15}; // 2 sensors on the left, 2 on the right
const int weights[sensorCount] = {-2000, -1240, 0, 1000, 2000};

int sensorValues[sensorCount];

//LED variables
unsigned long previousMillis = 0;
int colorIndex = 0;
float colourBrightness = 0.5;

//DRS variables
unsigned long drsStartTime = 0;
const int drsThreshold = 70;
int drsDelay = 500;
bool drsState = false;
bool drsPending = false;



int previousError = 0;
int integralError = 0;

bool atNode = false;
const int nodeCount = 8; // Number of nodes

// Adjacency Matrix
int weightMatrix[nodeCount][nodeCount] = {
  //   0    1     2     3    4    5     6     7
  {    0, INF,  INF,  INF,   1, INF,    2, INF },    // Node 0: connects to 4 and 6
  { INF,    0,  INF,  INF, INF, INF,    2,   1 },    // Node 1: connects to 6 and 7
  { INF,  INF,    0,    2, INF, INF,    1, INF },    // Node 2: connects to 3 and 6
  { INF,  INF,    2,    0, INF, INF,  INF,   1 },    // Node 3: connects to 2 and 7
  {   2,  INF,  INF,  INF,   0, INF,  INF,   1 },    // Node 4: connects to 0 and 7
  { INF,  INF,  INF,  INF, INF,   0,  INF,   1 },    // Node 5: isolated
  {   1,    2,    2,  INF, INF, INF,    0, INF },    // Node 6: junction (nodes 0,1,2)
  { INF,    2,  INF,    1,   2,   1,  INF,   0 }     // Node 7: junction (nodes 1,3,4,5)
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

//-------------------------------------------------------------
//---------------Obstacle Detection Logic----------------------
//-------------------------------------------------------------

// Detect an obstacle in front of the sensor
bool detectObstacle() {
  int totalValue = 0;
  int numSamples = 3;

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


  // Check distance to obstacle
  if (adjustedValue < obstacleThreshold && adjustedValue > 1500) {
    Serial.println("Obstacle Detected!");
    return true;
  }
  return false;
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
