// Define RGB LED strip pins
const int redPin = 10;
const int greenPin = 11;


const int DRSPin = 12;

const int powerLevel = 255;

// Function to set RGB color
void setColor(int r, int g, int b) {
    analogWrite(redPin, r);
    analogWrite(greenPin, g);
    //analogWrite(bluePin, b);
}

// Function to generate rainbow colors
void rainbowFade(int wait) {
    for (int i = 0; i < 256; i++) {
        int r = sin((i * 3.14159 / 128) + 0) * 127 + 128;
        int g = sin((i * 3.14159 / 128) + 2.09439) * 127 + 128;
        int b = sin((i * 3.14159 / 128) + 4.18878) * 127 + 128;
        setColor(r, g, b);
        delay(wait);
    }
}

void setup() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    //pinMode(bluePin, OUTPUT);
    pinMode(DRSPin, OUTPUT);
}

void loop() {
  switchDRS(1);
  delay(1000);
  switchDRS(0);
  delay(1000);
}
void switchDRS(bool DRSPosition){
  if (DRSPosition) {
    digitalWrite(DRSPin, HIGH);
  } else {
    digitalWrite(DRSPin, LOW);
  }
}