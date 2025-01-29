#include <WiFi.h>
#include <HTTPClient.h>

#define BUFSIZE 512

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password

// Server details
const char *serverIP = "3.250.38.184"; // Server IP address
const int serverPort = 8000;          // Server port
const char *teamID = "rhtr2655";      // Replace with your team's ID

//Cloud Server Variables
bool isRunning = false;
int startingPosition = 0;  // Initial position of the robot
int currentPosition = startingPosition;   // Track the robot's current position
int nextPosition = 0;

// Wifi class
WiFiClient client;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectToWiFi();

  // Notify the server of the starting position and get next position
  nextPosition = sendPosition(startingPosition);
  
  // Check if next position valid and start mobot
  if (nextDestination != "-1") {
    Serial.print("Next Position: ");
    Serial.println(nextPosition);
    isRunning = true; // Start the line-following process
  } else {
    Serial.println("Failed to get the next Position.");
  }
}

void loop() {
  if (isRunning) {

  }
}

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
      return -1;
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


