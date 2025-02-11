const int obstacleSensor = 3;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(obstacleSensor, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  detectObstacle();
  delay(10);
}

void detectObstacle(){
  // Check obstacle sensor value
  //debug:
  int obstacleSensorValue = analogRead(obstacleSensor);
  //int obstacleSensorValue = 0;

  Serial.print("Sensor Value:");
  Serial.println(4095 - obstacleSensorValue);

  // check distance to obstacle
  if ((4095 - obstacleSensorValue) < 1200){
    Serial.print("Obstacle Detected!");
  }
  return;
}