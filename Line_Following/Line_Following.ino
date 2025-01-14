//Motor Pins
int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 35;
int motor2Phase = 36;

//Line Sensor Arrays
int sensorArray[5] = {0 ,0 ,0 , 0, 0};
int analogPins[5] = {4 , 5, 6, 7, 15};

//Settings
int speed = 150;
bool run = true;
int sensitivity = 500;

//PID Settings
float Kp = 1.0;   //Proportional Gain
float Ki = 0.0;   //Integral Gain
float Kd = 1.0;   //Derivative Gain

//PID Variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

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
  
}


void reverse(){
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, speed+5);
  analogWrite(motor2PWM, speed);
  Serial.println("Reverse");
}

void forward(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed+5);
  analogWrite(motor2PWM, speed);
  Serial.println("Forward");
}

void right(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, speed+60);
  Serial.println("Right");
  delay(5);
}

void left(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed+65);
  analogWrite(motor2PWM, 0);
  Serial.println("Left");
  delay(5);
}

void stop(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  Serial.println("Stopping");
}
