//Motor Pins
int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 35;
int motor2Phase = 36;

//Line Sensor Constants
int sensorArray[5] = {0 ,0 ,0 , 0, 0};
int analogPins[5] = {4 , 5, 6, 7, 15};      //[0] is right most sensor

//Settings
int speed = 120;
int whiteThreshold = 500;

const float Kp = 0.5;   //Lower is smoother  

int motor1Speed = 0;
int motor2Speed = 0;

void setup(){
  Serial.begin(9600);
  //Output pin Initialisation
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  //sensor Pin Initialisation
  for (int i = 0; i < 6; i++){
    pinMode(i, INPUT);
  }
}

void loop(){
  // Read sensor values
  readSensors();
  calculateTurn();

  Serial.print("Left: ");
  Serial.print(motor1Speed);
  Serial.print(" | Right: ");
  Serial.println(motor2Speed);

  driveMotor();
}

void driveMotor(){
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, motor1Speed);
  analogWrite(motor2PWM, motor2Speed);
}

void stop(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  Serial.println("Stopping");
}

void readSensors(){
  for (int i = 0; i < 5; i++){
    sensorArray[i] = analogRead(analogPins[i]);
  }
}

void calculateTurn(){
  int deviation = 0;

  if (detectNode()){
    stop();
    delay(1000);
  }

  if (sensorArray[4] < whiteThreshold) {
    deviation = -3;  // Strong left deviation
  } else if (sensorArray[3] < whiteThreshold) {
    deviation = -1;  // Mild left deviation
  } else if (sensorArray[0] < whiteThreshold) {
    deviation = 3;   // Strong right deviation
  } else if (sensorArray[1] < whiteThreshold) {
    deviation = 1;   // Mild right deviation
  }
  else {
    deviation = 0;
  }

  motor1Speed = speed - (deviation * Kp * speed);
  motor2Speed = speed + (deviation * Kp * speed);
}

bool detectNode(){
  int whiteCount = 0;
  
  if(sensorArray[0] < whiteTheshold){
    whiteCount++;
  } 
  if(sensorArray[1] < whiteThreshold){
    whiteCount++;
  } 
  if(sensorArray[3] < whiteThreshold){
    whiteCount++;
  } 
  if(sensorArray[4] < whiteThreshold){
    whiteCount++;
  }

  if (whiteCount >= 3){
    return true;
  } else{
    return false;
  }
}