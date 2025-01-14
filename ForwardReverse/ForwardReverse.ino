int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 35;
int motor2Phase = 36;

int speed = 80;
bool run = true;

void setup(){
  Serial.begin(9600);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

}

void loop(){
  forward();
  delay(2000);
  reverse();
  delay(2000);
}


void forward(){
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
  Serial.println("Forward");
  delay(1000);
}

void reverse(){
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
  Serial.println("Reverse");
  delay(1000);
}