#ifndef settings_H
#define settings_H

// Motor Speeds
constexpr int baseSpeed = 90;   // Base speed for the motors (0–255) 120
constexpr int turnSpeed = 110;   // Turning Speed for 90 degree turns 
constexpr int straightSpeed = 180;

constexpr int pivotSpeed = 80;  // Speed for outside sensor turns 90
constexpr int constrainSpeed = 130; // Max motor speed 140

// PID parameters
constexpr float Kp = 0.5; // Proportional gain (3)
constexpr float Ki = 0;  // Integral gain (set to 0.000015 initially)
constexpr float Kd = 4.25;  // Derivative gain   (4.25) 17

// Separate PID values for straight paths
constexpr float straightKp = 0.05; // Lower Kp to reduce oscillation
constexpr float straightKd = 0; // Lower Kd for smoother correction

// Object Detection Threshold
constexpr int obstacleThreshold = 2500;  //Obstacle Sensitivity. Higher means further sensing

//Node detection settings
constexpr int forwardDelay = 80;   // Time to move across line slightly
constexpr unsigned long stopDelay = 50;     // Stopping Time at node
constexpr int rotationTime = 600;   // Time to turn 180 degrees
constexpr int turningTime = 350;    // Time to make a 90 degree turn 
constexpr int straightDelay = 800;  // Boost time


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