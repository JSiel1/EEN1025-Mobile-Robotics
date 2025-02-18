#ifndef settings_H
#define settings_H

// Motor Speeds
constexpr int baseSpeed = 120;   // Base speed for the motors (0–255)
constexpr int turnSpeed = 110;   // Turning Speed for 90 degree turns 

constexpr int pivotSpeed = 90;  // Speed for outside sensor turns 
constexpr int constrainSpeed = 120; // Max motor speed

// PID parameters
constexpr float Kp = 0.175; // Proportional gain (0.7)
constexpr float Ki = 0.000001;  // Integral gain (set to 0.000015 initially)
constexpr float Kd = 0.11;  // Derivative gain   (4.25)

// Object Detection Threshold
constexpr int obstacleThreshold = 2500;  //Obstacle Sensitivity. Higher means further sensing

//Node detection settings
constexpr int forwardDelay = 80;   // Time to move across line slightly
constexpr unsigned long stopDelay = 50;     // Stopping Time at node
constexpr int rotationTime = 800;   // Time to turn 180 degrees
constexpr int turningTime = 400;    // Time to make a 90 degree turn 


// Wi-Fi credentials
constexpr char *ssid = "iot";                                  // Replace with your Wi-Fi SSID
constexpr char *password = "manganese30sulphating";           // Replace with your Wi-Fi password
//constexpr char *password = "overtechnicality7petrophilous";   // Secondary ESP32 

// Server details
constexpr char *serverIP = "3.250.38.184"; // Server IP address
constexpr int serverPort = 8000;          // Server port
constexpr char *teamID = "rhtr2655";      // Replace with your team's ID

//DRS Settings
constexpr int drsThreshold = 70;
constexpr int drsDelay = 500;

#endif