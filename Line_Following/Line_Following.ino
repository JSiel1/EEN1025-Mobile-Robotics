//Motor Pins
int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 35;
int motor2Phase = 36;

//Function declerations
void forward();
void reverse();
void left();
void right();
void stop();
bool lineDetection(int sensorValue);  //true if line detected


//Line Sensor Arrays
int sensorArray[5] = {0 ,0 ,0 , 0, 0};
int analogPins[5] = {4 , 5, 6, 7, 15};


//Settings
int speed = 150;
bool run = true;

int threshold = 800;

//PID Settings
float Kp = 1.0;   //Proportional Gain
float Ki = 0.0;   //Integral Gain
float Kd = 1.0;   //Derivative Gain

//PID Variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float PID = 0;



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
  //calculate PID error
  error = (sensorArray[0] * -2) + (sensorArray[1] * -1) + (sensorArray[3] * 1) + (sensorArray[4] * 2)) / (sensorArray[0] + sensorArray[1] + sensorArray[3] + sensorArray[4] + 0.001);  
  integral += error;
  derrivative += errorr - previousError;
  PID = (Kp * error) + (Ki * integral) + (Kd * derrivative);
  previousError = error;
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

bool lineDetection(){
  for (int n = 0; n < 5; n++){
    if (analogRead(analogPins[n]) < threshold){
      sensorArray[n] = 1;
    } else {
      sensorArray[n] = 0;
    }
  }
}
