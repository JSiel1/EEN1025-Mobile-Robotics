#include <ArduinoHttpClient.h>
#include <WiFi.h>

const char *ssid = "IOT";
const char *password = "yourpasswd";

const char *serverAddress = "3.250.38.184";  // server address
const int port = 8000;
const int teamID = 14;

//Wifi and HTTP classes
WiFiClient wifiClient;
HttpClient httpClient = HttpClient(wifiClient, serverAddress, port);

int startingPosition = 0;


void setup() {
  Serial.begin(9600);

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi.");
}

void loop() {
  httpRequest();
}

void httpRequest(){
  String url = "/api/arrived/" + String(teamID);
  String requestBody = "position=" + String(startingPosition);

  Serial.println("Making POST request...");
  httpClient.beginRequest();
  httpClient.post(url);
  httpClient.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  httpClient.sendHeader("Content-Length", requestBody.length());
  httpClient.beginBody();
  httpClient.print(requestBody);
  httpClient.endRequest();

  //read request from server
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  Serial.print("Status Code:");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

  delay(1000);
}

