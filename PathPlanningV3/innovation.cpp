#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN       18
#define SCL_PIN       17
#define BATTERY_PIN   8


#define R1 943.0  // 1kΩ
#define R2 325.0  // 330Ω

#define ADC_MAX 4095.0  
#define REF_VOLTAGE 3.3 
#define BATTERY_MAX 10.8  
#define BATTERY_MIN 9.4   

unsigned long prevMillis = 0;  // Stores last time the display was updated
const long displayRefreshRate = 1000;        // displayRefreshRate in milliseconds (1 second)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initialiseDisplay() {    
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println("SSD1306 init failed!");
        while (1);
    }

    display.clearDisplay();
    display.display();
}

// **Function to map voltage to SOC based on lookup table**
int voltageToSOC(float voltage) {
    if (voltage >= 10.8) return 100;
    if (voltage >= 10.6) return 90;
    if (voltage >= 10.4) return 80;
    if (voltage >= 10.2) return 65;
    if (voltage >= 10.0) return 50;
    if (voltage >= 9.8)  return 35;
    if (voltage >= 9.6)  return 20;
    return 0;  // Anything below 9.4V is critically low
}

void displayBattery() {
  unsigned long currMillis = millis();
  if (currMillis - prevMillis >= displayRefreshRate) {
    prevMillis = currMillis; // Save last update time
    int rawADC = analogRead(BATTERY_PIN);
    float adcVoltage = (rawADC / ADC_MAX) * REF_VOLTAGE;
    float batteryVoltage = adcVoltage * ((R1 + R2) / R2) + 0.88;
    int batteryPercentage = voltageToSOC(batteryVoltage);
    int batteryBars = map(batteryPercentage, 0, 100, 0, 4);
      
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V | Charge: ");
    Serial.print(batteryPercentage);
    Serial.println("%");

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