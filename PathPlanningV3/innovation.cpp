#include "innovation.h"

static const int displayRefreshRate = 5000;  // Now only accessible within this file
static unsigned long prevMillis = 0;         // Now only accessible within this file

//LED variables
unsigned long previousMillis = 0;
int colorIndex = 0;
float colourBrightness = 0.7;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initDisplay() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
    while (1);
  }
  display.clearDisplay();
  display.display();
}

/*----------------------------------------------------------
 *              Voltage to Battery Percentage
 *--------------------------------------------------------*/
int voltageToSOC(float voltage) {
    if (voltage >= 10.8) return 100;
    if (voltage >= 10.6) return 90;
    if (voltage >= 10.4) return 80;
    if (voltage >= 10.2) return 65;
    if (voltage >= 10.0) return 50;
    if (voltage >= 9.8)  return 35;
    if (voltage >= 9.6)  return 20;
    return 0; 
}

/*----------------------------------------------------------
 *           Battery Display Task for Core 1
 *--------------------------------------------------------*/
void displayBatteryTask(void *pvParameters) {
    while (1) {
        displayBattery();  
        vTaskDelay(displayRefreshRate / portTICK_PERIOD_MS); 
    }
}

/*----------------------------------------------------------
 *              Battery Display Function
 *--------------------------------------------------------*/
void displayBattery() {
  unsigned long currMillis = millis();
  if (currMillis - prevMillis >= displayRefreshRate) {
      prevMillis = currMillis;  
       // **Read Battery Voltage**
      int rawADC = analogRead(BATTERY_PIN);
      float adcVoltage = (rawADC / ADC_MAX) * REF_VOLTAGE;
      float batteryVoltage = adcVoltage * ((R1 + R2) / R2) + 0.88;
      int batteryPercentage = voltageToSOC(batteryVoltage);
      int batteryBars = map(batteryPercentage, 0, 100, 0, 4);

      //Serial.print("Battery Voltage: ");
      //Serial.print(batteryVoltage, 2);
      //Serial.print("V | Charge: ");
      //Serial.print(batteryPercentage);
      //Serial.println("%");

      // **Update Display**
      display.clearDisplay();
      display.drawRect(10, 10, 50, 20, WHITE);
      display.fillRect(60, 15, 5, 10, WHITE);
      for (int i = 0; i < batteryBars; i++) {
          display.fillRect(12 + (i * 12), 12, 10, 16, WHITE);
      }

      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(10, 40);
      display.print("Battery: ");
      display.print(batteryVoltage, 2);
      display.print("V");

      display.setCursor(10, 52);
      display.print("Charge: ");
      display.print(batteryPercentage);
      display.print("%");

      display.display();
  }
}

//-------------------------------------------------------------
//---------------------------LED-------------------------------
//-------------------------------------------------------------

// Function to set RGB color
void setColour(int r, int g, int b) {
    analogWrite(redPin, r*colourBrightness);
    analogWrite(greenPin, g*colourBrightness);
    analogWrite(bluePin, b*colourBrightness);
}

// Function to generate rainbow colors
void rainbowFade(int wait) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= wait) {
        previousMillis = currentMillis;

        int r = sin((colorIndex * 3.14159 / 128) + 0) * 127 + 128;
        int g = sin((colorIndex * 3.14159 / 128) + 2.09439) * 127 + 128;
        int b = sin((colorIndex * 3.14159 / 128) + 4.18878) * 127 + 128;

        setColour(r, g, b);

        colorIndex++;
        if (colorIndex >= 256) colorIndex = 0; // Reset after full cycle
    }
}

//-------------------------------------------------------------
//---------------------------DRS-------------------------------
//-------------------------------------------------------------

void switchDRS(bool DRSPosition){
  if (DRSPosition) {
    digitalWrite(DRSPin, HIGH);
  } else {
    digitalWrite(DRSPin, LOW);
  }
}
