#ifndef innovation_H
#define innovation_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// **OLED Settings**
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN       18
#define SCL_PIN       17
#define BATTERY_PIN   8

// **Voltage Divider**
#define R1 943.0  
#define R2 325.0  
#define ADC_MAX 4095.0  
#define REF_VOLTAGE 3.3 
#define BATTERY_MAX 10.8  
#define BATTERY_MIN 9.4   

#define redPin 10
#define greenPin 11
#define bluePin 12
#define DRSPin 9

// **Display Object**
extern Adafruit_SSD1306 display;

// Function Declarations
void initDisplay();
void displayBatteryTask(void *pvParameters);
void displayBattery();
int voltageToSOC(float voltage);

void setColour(int r, int g, int b);
void rainbowFade(int wait);
void switchDRS(bool DRSPosition);

#endif
