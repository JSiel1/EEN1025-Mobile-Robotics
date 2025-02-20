#ifndef settings_H
#define settings_H

// Motor Speeds
constexpr int baseSpeed = 90;   // Base speed for the motors (0–255) 90
constexpr int turnSpeed = 110;   // Turning Speed for 90 degree turns 110 
constexpr int straightSpeed = 170;

constexpr int pivotSpeed = 80;  // Speed for outside sensor turns 80
constexpr int constrainSpeed = 130; // Max motor speed 130

// PID parameters
constexpr float Kp = 0.48; // Proportional gain (3) 0.48
constexpr float Ki = 0;  // Integral gain (set to 0.000015 initially)
constexpr float Kd = 5.5;  // Derivative gain   (4.25) 5

// Separate PID values for straight paths
constexpr float straightKp = 0.008; // Lower Kp to reduce oscillation 0.01
constexpr float straightKd = 2; // Lower Kd for smoother correction

// Object Detection Threshold
inline int obstacleThreshold = 2500;  //Obstacle Sensitivity. Higher means further sensing
const int outerObstacleThreshold = 2; // Obstacle detection threshold in cm

//Node detection settings
constexpr int forwardDelay = 80;   // Time to move across line slightly
constexpr unsigned long stopDelay = 50;     // Stopping Time at node
constexpr int rotationTime = 600;   // Time to turn 180 degrees
constexpr int turningTime = 275;    // Time to make a 90 degree turn 
constexpr int straightDelay = 950;  // Boost time

constexpr int motorOffset = 2;

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