/* 
 *  Project: Motion-Controlled Table Lamp
 *  Description: This project uses an MPU6050 sensor to control a table lamp in the form of a cube.
 * 
 *  How it works: 
 *  Tilting the lamp in the X axis forward switches it on, and backwards off, with a smooth fade in and out effect. 
 *  Tilting the lamp in the Y axis **while holding**  to the right, it increases the brightness, and to the left, it decreases the brightness.
 *  
 *  It also includes a basic filter in the readings of the MPU that helps smoothing the values so the functions do not start by minor noises in the readings and micromovements.
 */

#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <EEPROM.h>

#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0
#define RAD_TO_DEG 57.295779

// LED Matrix
#define NUM_LEDS 64
#define LED_PIN 4

#define FADE_SPEED 7  // Adjust this value to change the speed of the fading transition
#define TILT_THRESHOLD 4  // Adjust this value to change the tilt threshold
#define MAX_BRIGHTNESS 255  // Adjust this value to change the maximum brightness
#define MIN_BRIGHTNESS 0  // Adjust this value to change the minimum brightness
#define BRIGHTNESS_INCREMENT 1  // Adjust this value to change the brightness increment
#define BRIGHTNESS_THRESHOLD 50 // Adjust this value to change the brightness treshold when brightness is close to 0 to create a smooth transition and avoid the LEDs to change too suddenly

#define DEBUG 0 // Set to 1 to enable debug in the code, 0 to disable
#if DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
#endif

float voltage;
int bat_percentage;
int analogInPin = A0; // Analog input pin!
int sensorValue;

// Pins for charging and standby
const int CHARGING_PIN = D7;
const int STANDBY_PIN = D6;

// Check Battery voltage using multimeter & add/subtract the value
float calibration = 0.36;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

bool isCharging;
bool isStandby;

CRGB leds[NUM_LEDS];

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];
float OffsetAcc[2] = {0, 0};
float OffsetGy[2] = {0, 0};

String values;
long previous_time;
float dt;

unsigned long previousFadeMillis;
int brightness = 0;  // Start with the minimum brightness
bool isFadingIn = false;
bool isFadingOut = false;
bool isIncreasingBrightness = false;
bool isDecreasingBrightness = false;
int savedBrightness = MAX_BRIGHTNESS; // Saves user defined brightness. It starts in MAX_BRIGHTNESS by default

int prevSavedBrightness = -1;  // start with an invalid brightness value -- This is a variable to check if the brightness value has changed to save the new one in the EEPROM

unsigned long previousFadeWhiteMillis = 0;  // Keep track of the last time fadeToWhite() was updated
unsigned long previousFadeBlackMillis = 0;  // Keep track of the last time fadeToBlack() was updated

// Filter constants to smooth the readings of the MPU 6050
static float previousAngleX = 0;
static float previousAngleY = 0;
float alpha = 0.75; // factor between 0 and 1, which determines the weight given to the current value. A smaller alpha gives more weight to older values, while a larger alpha makes the filter respond more to recent changes

// Functions declaration for the VS Code IDE (platformio) -- Not needed for Arduino IDE
void decreaseBrightness();
void increaseBrightness();
void fadeToBlack();
void fadeToWhite();
void readRawData();
void calculateAngles();

// Test SIMPLE
int STATE = 0; // 0 OFF, 1 ON
int POSITION_STATE = 0;
bool needToFadeIn = false;
bool needToFadeOut = false;

void setup()
{
  EEPROM.begin(512);
  EEPROM.get(0, savedBrightness); // Retrieves the brightness value from EEPROM

  Wire.begin(2,0);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setCorrection(Candle); // Setup the LED Matrix to warm white
  FastLED.setMaxPowerInMilliWatts(5000); // Sets current limit

  // TP4056 battery management
  pinMode(analogInPin, INPUT);
  pinMode(CHARGING_PIN, INPUT); // Using internal pull-up
  pinMode(STANDBY_PIN, INPUT); // Using internal pull-up

  // MPU Calibration at start so no matter if the user turns on the lamp in a tilted table, it will be start in zero on all axis.
  for(int i = 0; i < 400; i++)
  {
    readRawData();
    OffsetAcc[0] += AcX;
    OffsetAcc[1] += AcY;
    OffsetGy[0] += GyX;
    OffsetGy[1] += GyY;
    delay(5);
  }

  // Calculate average
  OffsetAcc[0] /= 400.0;
  OffsetAcc[1] /= 400.0;
  OffsetGy[0] /= 400.0;
  OffsetGy[1] /= 400.0;

  // Ensure the LED matrix is off when turning on the device
  FastLED.clear();
  FastLED.show();
}

void loop()
{
  readRawData();

  // Apply offset from the calibration
  AcX -= OffsetAcc[0];
  AcY -= OffsetAcc[1];
  GyX -= OffsetGy[0];
  GyY -= OffsetGy[1];

  // Calculate angles from the MPU
  calculateAngles();

  // Print the angles values to debug
  values = "Axis X: " + String(Angle[0], 1) + " - Axis Y: " + String(Angle[1], 1);
  DEBUG_PRINT("Brightness:" + String(brightness) + " - "+ values);

  // START ------- TP4056 battery management -------
  
  sensorValue = analogRead(analogInPin);
  // Multiply by two as voltage divider network is 100K & 100K Resistor
  voltage = (((sensorValue * 3.3) / 1024) * 2 - calibration);
  // 2.8V as Battery Cut off Voltage & 4.2V as Maximum Voltage
  bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100);

  if (bat_percentage >= 100) {bat_percentage = 100;}
  if (bat_percentage <= 0) {bat_percentage = 1;}

  // Check charging status
  if(digitalRead(CHARGING_PIN) == 1){
    isCharging = true;
  } else if (digitalRead(CHARGING_PIN) == 0) {
    isCharging = false;
  }

  // END -------  TP4056 battery management -------

  if(isCharging == true) {
    DEBUG_PRINT("CHARGER CONNECTED");
    
    bool isTilted = Angle[0] >= TILT_THRESHOLD;

    if (isTilted && STATE == 0 && POSITION_STATE == 0) {
      STATE = 1; // Change to ON
      POSITION_STATE = 1;
      DEBUG_PRINT("State changed to ON");
    } else if (isTilted && STATE == 1 && POSITION_STATE == 0) {
      STATE = 0; // Change to OFF
      POSITION_STATE = 1;
      DEBUG_PRINT("State changed to OFF");
    }

    if (!isTilted && POSITION_STATE == 1) {
      POSITION_STATE = 0;
      DEBUG_PRINT("MOVED TO THE CENTER ------------------------------");
    }

    if (STATE == 1) {
      fadeToWhite();
    } else if (STATE == 0) {
      fadeToBlack();
    }


  } else if (isCharging == false) {
    DEBUG_PRINT(" CHARGER DISCONNECTED ");
  }
  
  FastLED.show();

}


void readRawData()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,4,true);
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
}

void calculateAngles()
{
  Acc[0] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[1] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;

  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;

  dt = (millis() - previous_time) / 1000.0;
  previous_time = millis();

  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

  Angle[0] = (1 - alpha) * previousAngleX + alpha * Angle[0];
  Angle[1] = (1 - alpha) * previousAngleY + alpha * Angle[1];

  previousAngleX = Angle[0];
  previousAngleY = Angle[1];
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void fadeToWhite() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousFadeWhiteMillis >= FADE_SPEED) {
    previousFadeWhiteMillis = currentMillis;
    if(brightness < MAX_BRIGHTNESS) {
      brightness++;
      fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
      FastLED.show();
    }
  }
}

void fadeToBlack() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousFadeBlackMillis >= FADE_SPEED) {
    previousFadeBlackMillis = currentMillis;
    if(brightness > MIN_BRIGHTNESS) {
      brightness--;
      fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
      FastLED.show();
    }
  }
}