#include <WiFi.h>

#define BUFSIZE 512

// Wi-Fi credentials
const char *ssid = "iot";                // Replace with your Wi-Fi SSID
const char *password = "manganese30sulphating"; // Replace with your Wi-Fi password

// Server details
const char *serverIP = "3.250.38.184"; // Server IP address
const int serverPort = 8000;          // Server port
const char *teamID = "rhtr2655";      // Replace with your team's ID

//Cloud Server Variables
int startingPosition = 0;  // Initial position of the robot
int currentPosition = startingPosition;   // Track the robot's current position
int nextPosition = 0;
String route = "";

// Wifi class
WiFiClient client;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectToWiFi();

  // Obtain path
  route = getRoute();

  Serial.println(route);

}

void loop() {
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
      return;
  }
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

