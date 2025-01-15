#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "Group-14-AP";
const char *password = "password123";

// Create server object
WebServer server(80);

// Assuming motor1PWM, motor2PWM, and PID are global variables
int motor1PWM = 150;  // Example value for left motor
int motor2PWM = 200;  // Example value for right motor
int PID = 125;        // Example integer PID value

void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Motor Monitor</title>
    <style>
      body {
        font-family: Arial, sans-serif;
        font-size: 30px;
        margin: 0;
        padding: 20px;
        display: flex;
        justify-content: center;
        flex-direction: column;
      }
      .container {
        display: flex;
        justify-content: center;
        align-items: flex-start;
        margin-top: 30px;
      }
      .bar-container {
        width: 50px;
        height: 300px;
        background: #ccc;
        margin: 10px;
        position: relative;
      }
      .bar {
        width: 100%;
        height: 0%;
        background: #007bff;
        transition: height 0.2s;
      }
      .label {
        font-size: 14px;
        position: absolute;
        bottom: -20px;
        width: 100%;
        text-align: center;
      }
      .info {
        margin-left: 20px;
        font-size: 18px;
        display: flex;
        flex-direction: row;
        justify-content: center;
      }
      .info div {
        text-align: center; 
        margin: 0 10px;
      }
    </style>
  </head>
  <body>
    <h1 style="text-align: center;">Motor and PID Status </h1>
    <div class="container">
      <div class="bar-container">
        <div id="leftBar" class="bar"></div>
        <div class="label">Left Motor</div>
      </div>
      <div class="bar-container">
        <div id="rightBar" class="bar"></div>
        <div class="label">Right Motor</div>
      </div>
      <div class="info">
        <div id="leftMotorValue">Left Motor: 0</div>
        <div id="rightMotorValue">Right Motor: 0</div>
        <div id="PIDValue">PID: 0</div>
      </div>
    </div>

    <script>
      const leftBar = document.getElementById('leftBar');
      const rightBar = document.getElementById('rightBar');
      const leftMotorValue = document.getElementById('leftMotorValue');
      const rightMotorValue = document.getElementById('rightMotorValue');
      const PIDValue = document.getElementById('PIDValue');

      function updateBars() {
        fetch('/get-values')
          .then(response => response.json())
          .then(data => {
            leftBar.style.height = `${(data.leftMotor / 255) * 100}%`;
            rightBar.style.height = `${(data.rightMotor / 255) * 100}%`;
            leftMotorValue.innerHTML = `Left Motor: ${data.leftMotor}`;
            rightMotorValue.innerHTML = `Right Motor: ${data.rightMotor}`;
            PIDValue.innerHTML = `PID: ${data.PID}`;
          })
          .catch(error => console.error('Error fetching motor values:', error));
      }

      setInterval(updateBars, 500);
    </script>
  </body>
  </html>
  )rawliteral");
}

// Serve motor speeds and PID as JSON
void handleGetValues() {
  // Map motor1PWM and motor2PWM to leftMotor and rightMotor, and send PID as integer
  String json = "{\"leftMotor\": " + String(motor1PWM) + ", \"rightMotor\": " + String(motor2PWM) + ", \"PID\": " + String(PID) + "}";
  server.send(200, "application/json", json);
}

// Simulate updating motor speeds in your existing code
void updateMotorSpeeds() {
  // Example: Update these values based on other parts of your program
  motor1PWM = random(0, 256);  // Replace with actual motor speed logic
  motor2PWM = random(0, 256); // Replace with actual motor speed logic
}

void setup() {
  Serial.begin(115200);

  // Set up Wi-Fi access point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");

  // Define server routes
  server.on("/", handleRoot);        // Serve the HTML page
  server.on("/get-values", handleGetValues);  // Serve motor values as JSON

  // Start HTTP server
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle HTTP requests
  server.handleClient();

  // Update motor speeds periodically (or from your actual logic)
  updateMotorSpeeds();

  delay(1000);

}