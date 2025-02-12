const int sensorCount = 4;
int sensorValues[sensorCount];
const int IR_PINS[] = {4, 7, 5, 15}; 


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < sensorCount; i++){
    pinMode(IR_PINS[i], INPUT);
  }
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  readLineSensors();
  delay(200);

}

void readLineSensors(){
  // Read sensor values
  // put your main code here, to run repeatedly:
  for (int i = 0; i<sensorCount; i++) {
    sensorValues[i] = analogRead(IR_PINS[i]);
    Serial.print(sensorValues[i]); // This prints the actual analog reading from the sensors
    Serial.print("\t"); //tab over on screen
    if(i==3) {
      Serial.println(""); //carriage return
      delay(200); // display new set of readings every 600mS
    }
  } 

}
