// Motor pins
#define motor1PWM 37  // Left motor enable (PWM)
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor enable (PWM)
#define motor2Phase 36  // Right motor phase

// IR sensor pins (only outermost sensors are used)
const uint8_t IR_PINS[] = {4, 7, 5, 15}; // 2 sensors on the left, 2 on the right
const uint8_t SENSOR_COUNT = 4;

// Additional stop sensor pin
#define STOP_SENSOR A0

// Thresholds for white and black detection
const int WHITE_THRESHOLD = 270; // Around 200 for white line
const int BLACK_THRESHOLD = 2700; // Around 2700 for black surface

// PID parameters
float Kp = 0.35; // Proportional gain
float Ki = 0.0;  // Integral gain (set to 0 initially)
float Kd = 0.2;  // Derivative gain

float Pvalue = 0;
float Ivalue = 0;
float Dvalue = 0;
int previousError = 0;

// Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 220; // Base speed for the motors (0–255)

void setup() {
  // Set motor pins as output
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  // Set the stop sensor pin as input
  pinMode(STOP_SENSOR, INPUT);

  // Start serial communication for debugging
  Serial.begin(115200);

  // Initial test to verify motor function
  //motor_drive(100, 100); // Drive both motors forward
 // delay(1000);           // Move for 2 seconds
 // motor_drive(0, 0);     // Stop motors
}

void loop() {
  // Debugging: Check stop sensor value
  int stopSensorValue = analogRead(STOP_SENSOR);
  Serial.print("Stop Sensor Value (inverted): ");
  Serial.println(4095 - stopSensorValue); // Inverted value for debugging

  // Check the additional sensor (inverted)
  if ((4095 - stopSensorValue) < 1800) { // Replace 500 with appropriate threshold
    // Turn the robot 180 degrees
    motor_drive(-baseSpeed, baseSpeed); // Rotate in place
    delay(600);                        // Adjust delay for a 180-degree turn
    motor_drive(0, 0);                  // Stop after turning
    delay(100);                         // Short pause before resuming
    return;                             // Skip the rest of the loop iteration
}


  // Perform line following
  line_following();
}

void line_following() {
  // Read and debug sensor values
  int sensorValues[SENSOR_COUNT];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorValues[i]);
  }

  // Check for node detection (3 or more sensors detecting white)
  if (detectNode(sensorValues)) {
    motor_drive(0, 0); // Stop the robot
    delay(100);        // Wait for 0.1 seconds
    motor_drive(80, 80); // Drive forward at low speed
    delay(200);          // Move slightly forward to cross the line
    motor_drive(0, 0);   // Stop again
    delay(1000);         // Wait for 1 second before resuming
    return;              // Skip the rest of the loop iteration
  }

  // Calculate the position of the line (weighted average method using outer sensors)
  int weights[] = {-2000, -1000, 1000, 2000};
  int position = 0;
  int total = 0;

  
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    position += sensorValues[i] * weights[i];
    total += sensorValues[i];
  }

  // Avoid division by zero and calculate position
  if (total != 0) {
    position /= total;
  }

  // Calculate error (target is position 0, the center)
  int error = 0 - position;

  // PID calculations
  int P = error;
  static int I = 0;
  int D = error - previousError;

  I += error;

  float PIDvalue = P*Kp + I*Ki + D*Kd;
  previousError = error;

  // Check outer sensors for sharp turns
  if (sensorValues[0] < WHITE_THRESHOLD) {
    leftSpeed = 0;
    rightSpeed = baseSpeed;
  } else if (sensorValues[3] < WHITE_THRESHOLD) {
    leftSpeed = baseSpeed;
    rightSpeed = 0;
  } else {
    leftSpeed = baseSpeed - PIDvalue;
    rightSpeed = baseSpeed + PIDvalue;
  }

  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  motor_drive(leftSpeed, rightSpeed);
}

// Detect node (3 or more sensors detecting white)
bool detectNode(int sensorValues[]) {
  int whiteCount = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] < WHITE_THRESHOLD) {
      whiteCount++;
    }
  }
  return (whiteCount >= 3); // Node detected if 3 or more sensors see white
}

// Motor drive function
void motor_drive(int left, int right) {
  if (left > 0) {
    digitalWrite(motor1Phase, LOW);
    analogWrite(motor1PWM, left);
  } else {
    digitalWrite(motor1Phase, HIGH);
    analogWrite(motor1PWM, -left);
  }

  if (right > 0) {
    digitalWrite(motor2Phase, LOW);
    analogWrite(motor2PWM, right);
  } else {
    digitalWrite(motor2Phase, HIGH);
    analogWrite(motor2PWM, -right);
  }
}

void left() {
  motor_drive(-baseSpeed, baseSpeed); // Rotate in place
  delay(300);
  motor_drive(0, 0);                  // Stop after turning
  delay(100);
  return;
}

void right() {
  motor_drive(baseSpeed, -baseSpeed); // Rotate in place
  delay(300);
  motor_drive(0, 0);                  // Stop after turning
  delay(100);
  return;
}
