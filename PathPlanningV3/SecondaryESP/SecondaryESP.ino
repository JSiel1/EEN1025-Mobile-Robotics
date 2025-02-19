#define SIGNAL_PIN 4  // Input from main ESP32 (obstacle)
#define RESUME_PIN 5  // Input from main ESP32 (resume)

#define motor1PWM   37   // Left motor enable (PWM)
#define motor1Phase 38   // Left motor phase
#define motor2PWM   35   // Right motor enable (PWM)
#define motor2Phase 36   // Right motor phase

volatile bool nodeDetected = false;
volatile bool resumeSignalReceived = false;

const int IR_PINS[] = {4, 7, 5, 15};
const int sensorCount = 4;
int weights[4]      = {-2000, -1000, 1000, 2000};
int sensorValues[4] = {0, 0, 0, 0};

int leftSpeed        = 0;
int rightSpeed       = 0;

int baseSpeed        = 120; // normal forward speed
int outerPivotSpeed  = 140; // pivot speed on outer sensor

const int MAX_SPEED  = 140; 

const int whiteThreshold = 270;

float Kp = 0.4;
float Ki = 0.00;
float Kd = 3.5;
int previousError   = 0;
int integralError   = 0;


void IRAM_ATTR handleInterrupt() {
    nodeDetected = true;  // Obstacle detected
}

void IRAM_ATTR handleResume() {
    resumeSignalReceived = true;  // Resume received
}

void setup() {
  Serial.begin(115200);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  for (int i = 0; i < sensorCount; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

    pinMode(SIGNAL_PIN, INPUT_PULLDOWN);
    pinMode(RESUME_PIN, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN), handleInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(RESUME_PIN), handleResume, RISING);
}

void loop() {
  readLineSensors();
    // Pause when obstacle is detected
  if (nodeDetected) {
    while (!resumeSignalReceived) {
      delay(1);  // Wait until resume signal is received (very short delay)
    }
    Serial.println("Resume signal received! Resuming loop...");
    nodeDetected = false;
    resumeSignalReceived = false;
  }

  // Normal operations

  followLine();

  delay(5); 
}

void readLineSensors() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
  }
}


void followLine() {

  int position = 0;
  int total    = 0;
  for (int i = 0; i < sensorCount; i++) {
    position += sensorValues[i] * weights[i];
    total    += sensorValues[i];
  }

  if (total != 0) {
    position /= total; 
  } else {
    position = 0; 
  }

  // PID error
  int error  = -position;
  int dError = error - previousError;
  previousError = error;

  integralError += error;
  float pidOut = (Kp * error) + (Ki * integralError) + (Kd * dError);


  if (sensorValues[0] < whiteThreshold) {
    leftSpeed  = 0;
    rightSpeed = outerPivotSpeed;
  }
  else if (sensorValues[3] < whiteThreshold) {
    leftSpeed  = outerPivotSpeed;
    rightSpeed = 0;
  }
  else {
 
    leftSpeed  = baseSpeed - pidOut;
    rightSpeed = baseSpeed + pidOut;
  }


  leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);


  driveMotors(leftSpeed, rightSpeed);
}


void driveMotors(int leftVal, int rightVal) {

  if (leftVal >= 0) {
    digitalWrite(motor1Phase, LOW);
  } else {
    digitalWrite(motor1Phase, HIGH);
    leftVal = -leftVal; 
  }

  if (rightVal >= 0) {
    digitalWrite(motor2Phase, LOW);
  } else {
    digitalWrite(motor2Phase, HIGH);
    rightVal = -rightVal;
  }


  analogWrite(motor1PWM, leftVal);
  analogWrite(motor2PWM, rightVal);
}