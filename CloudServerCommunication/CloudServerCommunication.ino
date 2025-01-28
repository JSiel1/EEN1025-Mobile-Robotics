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
bool resetReady = false; // New state to track if the robot is ready for a new destination
int startingPosition = 0;  // Initial position of the robot
int currentPosition = 0;   // Track the robot's current position
String nextDestination = ""; // Store the next destination

// Wifi class
WiFiClient client;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectToWiFi();

  // Notify the server of the starting position
  sendPosition(startingPosition);

  // Get next Position from server
  nextDestination = receiveNextPosition();
  if (nextDestination != "-1") {
    Serial.print("Next Destination: ");
    Serial.println(nextDestination);
    isRunning = true; // Start the line-following process
  } else {
    Serial.println("Failed to get the next destination.");
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
void sendPosition(int position) {
  if (client.connect(server, port)) {
    String postBody = "position=" + String(position);

    // Construct HTTP POST request
    client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
    //client.println("Host: " + String(server));
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postBody.length());
    client.println(); // Blank line to indicate end of headers
    client.println(postBody); // POST body

    // Wait for a response
    while (client.connected() || client.available()) {
      if (client.available()) {
        String response = client.readString();
        Serial.println("Server response:");
        Serial.println(response);
        break;
      }
    }

    client.stop();
  } else {
    Serial.println("Connection to server failed!");
    client.stop()
  }
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

// Function to receive the next position
int receiveNextPosition() {
  if (client.connect(server, port)) {
    // Send a GET request to retrieve the next position
    client.println("GET /api/next/" + String(teamID) + " HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Connection: close");
    client.println();

    // Read and process the response
    String response = readResponse();
    int statusCode = getStatusCode(response);
    
    client.stop();

    if (statusCode == 200) {
      String body = getResponseBody(response);
      if (!body.equals("Finished")) {
        return body.toInt();
      } else {
        Serial.println("Final destination reached.");
        return -1; // Indicate final destination
      }
    } else {
      Serial.println("Failed to retrieve next position. HTTP Status: " + String(statusCode));
      return -1; // Indicate error
    }
  } else {
    Serial.println("Connection to server failed!");
    return -1; // Indicate connection failure
  }
}

