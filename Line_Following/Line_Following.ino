//Motor Pins
const int motor1PWM = 37;
const int motor1Phase = 38;
const int motor2PWM = 35;
const int motor2Phase = 36;

//Function declerations
void forward();
void reverse();
void stop();
void lineDetection();  
void calculatePID();

//Line Sensor Arrays
int sensorArray[5] = {0 ,0 ,0 , 0, 0};
const int analogPins[5] = {4 , 5, 6, 7, 15};

//Settings
int speed = 120;
bool run = true;

int whiteValue = 300;       //Sensor Value over white
int darkValue = 2500;       //Sensor valye over Black

//PID Settings
float Kp = 30.0;   //Proportional Gain
float Ki = 0.0;   //Integral Gain
float Kd = Kp*10;   //Derivative Gain

//PID Variables
float error = 0;
float previousError = 0;
float proportional = 0;
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
  lineDetection();
  calculatePID();
}

void driveMotor(int turnSpeed){
  int motor1Speed = speed - turnSpeed;    //Change sign if turning wrong direction
  int motor2Speed = speed + turnSpeed;    //^^

  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, motor1Speed);
  analogWrite(motor2PWM, motor2Speed);

  Serial.print("PID Correction: \t Left Motor Speed: \t Right Motor Speed: ");
  Serial.print(PID);
  Serial.print("\t");
  Serial.print(motor1Speed);
  Serial.print("\t");
  Serial.print(motor2Speed);
}

void stop(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  Serial.println("Stopping");
}

void lineDetection(){
  for (int n = 0; n < 5; n++){
    sensorArray[n] = map(analogRead(analogPins[n]), whiteValue, darkValue, 0, 1000);    //Convert the range of sensor readings
    sensorArray[n] = constrain(sensorArray[n], 0, 1000);    //Limit the possible sensor value
  }
}

void calculatePID(){
  //calculate PID error
  error = ((sensorArray[0] * -2) + (sensorArray[1] * -1) + (sensorArray[3] * 1) + (sensorArray[4] * 2)) / (sensorArray[0] + sensorArray[1] + sensorArray[3] + sensorArray[4] + 1);  
  
  proportional = error;
  integral += error;      //Integral Error
  derivative = (error - previousError);    //Derivative Error
  PID = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  
  previousError = error;
  driveMotor(PID);
}
