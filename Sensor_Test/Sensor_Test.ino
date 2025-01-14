int sensorArray[5] = {0 ,0 ,0 , 0, 0};
int analogPins[5] = {4 , 5, 6, 7, 15};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++){
    pinMode(i, INPUT);
  }
}

void loop() {
  for(int i = 0; i<5; i++){
    sensorArray[i] = analogRead(analogPins[i]);
    Serial.print(sensorArray[i]);
    Serial.print("\t");
    if(i == 4){
      Serial.println("");
      delay(600);
    }
  }
}
