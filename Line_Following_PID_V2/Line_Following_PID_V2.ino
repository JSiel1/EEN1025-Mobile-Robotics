// Motor pins
#define motor1PWM 37  // Left motor enable (PWM)
#define motor1Phase 38  // Left motor phase
#define motor2PWM 35  // Right motor enable (PWM)
#define motor2Phase 36  // Right motor phase

// IR sensor pins (only outermost sensors are used)
const uint8_t IR_PINS[] = {4, 5, 7, 15}; // 2 sensors on the left, 2 on the right
const uint8_t SENSOR_COUNT = 4;

// Thresholds for white and black detection
const int WHITE_THRESHOLD = 270; // Around 200 for white line
const int BLACK_THRESHOLD = 2700; // Around 2700 for black surface

// PID parameters
float Kp = 0.1; // Proportional gain
float Ki = 0.0;  // Integral gain (set to 0 initially)
float Kd = 0.01; // Derivative gain

float Pvalue = 0;
float Ivalue = 0;
float Dvalue = 0;
int P, I, D = 0;
int previousError = 0;
int error = 0;

//Motor Speeds
int leftSpeed = 0;
int rightSpeed = 0;
int baseSpeed = 120; // Base speed for the motors (0–255)

void setup() {
  // Set motor pins as output
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(115200);
}

void loop() {
  line_following();
}

void line_following() {
  // Read sensor values
  int sensorValues[SENSOR_COUNT];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
  }

  // Calculate the position of the line (weighted average method using outer sensors)
  // Assign weights based on sensor positions: Leftmost = -2000, Left inner = -1000, Right inner = +1000, Rightmost = +2000
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
  error = 0 - position;

  // PID calculations
  P = error;
  I += error;
  D = error - previousError;

  Pvalue = Kp * P;
  Ivalue = Ki * I;
  Dvalue = Kd * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  // Calculate motor speeds
  leftSpeed = baseSpeed - PIDvalue;
  rightSpeed = baseSpeed + PIDvalue;

  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  motor_drive(leftSpeed, rightSpeed);
}



//Change this if moving in wrong direction
void motor_drive(int left, int right) {
  // Control left motor
  if (left > 0) {
    digitalWrite(motor1Phase, HIGH); // Forward,  change to "LOW" if moving in wrong direction
    analogWrite(motor1PWM, left); // PWM control,   dont change this
  } else {
    digitalWrite(motor1Phase, LOW);  // Reverse, then this to "HIGH"
    analogWrite(motor1PWM, -left); // PWM control,    dont change this
  }

  // Control right motor
  if (right > 0) {
    digitalWrite(motor2Phase, HIGH); // Forward,  Change to "LOW" if moving in wrong direction
    analogWrite(motor2PWM, right); // PWM control,    dont change this
  } else {
    digitalWrite(motor2Phase, LOW);  // Reverse,      then this to "HIGH"
    analogWrite(motor2PWM, -right); // PWM control,     dont change this.
  }
}