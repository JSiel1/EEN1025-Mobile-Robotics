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
#include "PathPlanning.h"
#include "serverCommunication.h"
#include "motorControl.h"
#include "settings.h"
#include "innovation.h"

#define obstacleSensor 1    //obbstacle detection sensor

#define TRIGGER_PIN_1 40
#define ECHO_PIN_1 41
#define TRIGGER_PIN_2 42
#define ECHO_PIN_2 45


//Line detection Sensitivity
const int whiteThreshold = 400; // Around 200 for white line. Greater means higher sensitivity

// IR sensor pins (only outermost sensors are used)
const int sensorCount = 5;
const int IR_PINS[sensorCount] = {4, 7, 6, 5, 15}; // 2 sensors on the left, 2 on the right
int sensorValues[5];

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

  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
    
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(obstacleSensor, INPUT);
  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_PINS[i], INPUT);
  }

  initDisplay();  // Call the new function for initialization

  // **Create Display Task on Core 1**
  xTaskCreatePinnedToCore(
      displayBatteryTask,  // Function
      "Battery Display",   // Task name
      4096,                // Stack size
      NULL,                // Parameters
      1,                   // Priority
      NULL,                // Task handle
      1                    // Core 1
  );

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
  //displayBattery();
  
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
//---------------Sensor Detection Logic----------------------
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


// UltraSonic Sensors
float getDistance(int triggerPin, int echoPin) {
    long duration;
    float distance;

    // Trigger the sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Measure the time for echo
    duration = pulseIn(echoPin, HIGH, 30000); // Timeout at 30ms (~5m max distance)

    // Convert time to distance
    distance = duration * 0.0343 / 2;

    // If no valid measurement, return a large number
    if (distance <= 0 || distance > 400) {
        return 999; // Return a high value to indicate no detection
    }

    return distance;
}

bool detectOuterObstacle() {
  float distance1 = getDistance(TRIGGER_PIN_1, ECHO_PIN_1);
  float distance2 = getDistance(TRIGGER_PIN_2, ECHO_PIN_2);

  // Check if either sensor detects an obstacle within the threshold
  if (distance1 <= outerObstacleThreshold || distance2 <= outerObstacleThreshold) {
    return true;
  }
    
  return false;
}
