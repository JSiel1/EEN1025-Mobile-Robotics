#ifndef serverCommunication_H
#define serverCommunication_H

// Buffer Size
#define BUFSIZE 512

//Route re-writing
extern String route;

//-------------------------------------------------------------
//-----------------Cloud Server Communication------------------
//-------------------------------------------------------------

// Connect to wifi
void connectToWiFi();

// Function to send current position 
void sendPosition(int position);

// Function to read Route
String getRoute();

// Function to read the HTTP response
String readResponse();

// Function to get the status code from the response
int getStatusCode(String& response);

// Function to get the body from the response
String getResponseBody(String& response);

#endif