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
#define DEBUG 1 // Set to 1 to enable debug in the code, 0 to disable
#if DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
#endif

#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <I2Cdev.h>     
#include <MPU6050.h>
#include <MadgwickAHRS.h>

// MPU 6050 config
Madgwick filter;
unsigned long lastUpdateTime = 0;
#define G_R 131.0
float offsetX = 0.0, offsetY = 0.0;
float angleX = 0;
float angleY = 0;

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

// LED Matrix
#define NUM_LEDS 64
#define LED_PIN 4

#define FADE_SPEED 7  // Adjust this value to change the speed of the fading transition
#define TILT_THRESHOLD 6  // Adjust this value to change the tilt threshold
#define MAX_BRIGHTNESS 255  // Adjust this value to change the maximum brightness
#define MIN_BRIGHTNESS 0  // Adjust this value to change the minimum brightness
#define BRIGHTNESS_INCREMENT 1  // Adjust this value to change the brightness increment
#define BRIGHTNESS_THRESHOLD 50 // Adjust this value to change the brightness treshold when brightness is close to 0 to create a smooth transition and avoid the LEDs to change too suddenly

float voltage;
int bat_percentage;
int analogInPin = A0; // Analog input pin!
int sensorValue;

// Pins for charging and standby :)
const int CHARGING_PIN = D7;
const int STANDBY_PIN = D6;

// Check Battery voltage using multimeter & add/subtract the value
float calibration = 0.36;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

bool isCharging;
bool isStandby;

CRGB leds[NUM_LEDS];

String values;
long previous_time;
float dt;

unsigned long previousFadeMillis;
int brightness = 0;  // Start with the minimum brightness
bool isFadingIn = false;
bool isFadingOut = false;
bool isIncreasingBrightness = false;
bool isDecreasingBrightness = false;
int savedBrightness = 50; // Saves user defined brightness. It starts in MAX_BRIGHTNESS by default

int prevSavedBrightness = -1;  // start with an invalid brightness value -- This is a variable to check if the brightness value has changed to save the new one in the EEPROM

unsigned long previousFadeWhiteMillis = 0;  // Keep track of the last time fadeToWhite() was updated
unsigned long previousFadeBlackMillis = 0;  // Keep track of the last time fadeToBlack() was updated

// Functions declaration for the VS Code IDE (platformio) -- Not needed for Arduino IDE
void decreaseBrightness();
void increaseBrightness();
void fadeToBlack();
void fadeToWhite();
void readRawData();
void calculateAngles();
void saveLastAngle();

// Test SIMPLE
int STATE = 0; // 0 OFF, 1 ON
int POSITION_STATE = 0;
bool needToFadeIn = false;
bool needToFadeToBlack = false;
bool isTiltedX = false;
bool isTiltedY = false;
unsigned long tiltStartTimeX = 0;
unsigned long tiltStartTimeY = 0;
unsigned long previousIncreaseMillis = 0;  // Keep track of the last time increaseBrightness() was updated
String lastAngle; // Will store in which direction it was last turned on
unsigned long lastTiltTime = 0;
unsigned long debounceDelay = 1000; // Adjust this value based on your needs
bool isBrightnessAdjusted = false; // Add this line at the top with the other global variables
void calibrate(int calibrationCount);
bool comesFromDisconnected = false; // Variable to see if the device has just been re-connected after being unplugged from the charger

void setup()
{
  //EEPROM.begin(512);
  //EEPROM.get(0, savedBrightness); // Retrieves the brightness value from EEPROM

  Serial.begin(115200);
  Wire.begin(2,0);
  mpu.initialize();
  filter.begin(50);  // Set sample frequency to 50Hz

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setCorrection(Candle); // Setup the LED Matrix to warm white
  FastLED.setMaxPowerInMilliWatts(5000); // Sets current limit

  // TP4056 battery management
  pinMode(analogInPin, INPUT);
  pinMode(CHARGING_PIN, INPUT); // Using internal pull-up
  pinMode(STANDBY_PIN, INPUT); // Using internal pull-up

  calibrate(400); // Runs a calibration rutine when turning on to reset everything to zero

  // Ensure the LED matrix is off when turning on the device
  FastLED.clear();
  FastLED.show();
}

void loop()
{
  // Calculate angles from the MPU
  calculateAngles();

  // Print the angles values to debug
  values = "Axis X: " + String(angleX, 1) + " - Axis Y: " + String(angleY, 1);
  //DEBUG_PRINT("Brightness:" + String(brightness) + " - "+ values);
  //DEBUG_PRINT("Brightness:" + String(brightness));

  // TP4056 battery management
  sensorValue = analogRead(analogInPin);
  // Multiply by two as voltage divider network is 100K & 100K Resistor
  voltage = (((sensorValue * 3.3) / 1024) * 2 - calibration);
  // 2.8V as Battery Cut off Voltage & 4.2V as Maximum Voltage
  bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100);

  if (bat_percentage >= 100) {bat_percentage = 100;}
  if (bat_percentage <= 0) {bat_percentage = 1;}

  // Check charging status
  if(digitalRead(CHARGING_PIN) == 1){ // Is charging
    //DEBUG_PRINT("CHARGER CONNECTED");

    if(comesFromDisconnected == true) {
      calibrate(150); // Runs a quick calibration rutine after coming from disconnected to ensure good responsiveness in case it was moved to a tilted surface compared to the last one
      comesFromDisconnected = false;
      needToFadeToBlack = true; // Fades to black after calibrating to understand when it finishes
    }

    // Check if X axis is tilted
    if(angleX >= TILT_THRESHOLD || angleX <= -TILT_THRESHOLD) {
      if (!isTiltedX) {
        tiltStartTimeX = millis();
        isTiltedX = true;
        isBrightnessAdjusted = false; // Reset the flag when the cube is tilted
        DEBUG_PRINT("X is tilted");
      }
      
      if (millis() - tiltStartTimeX >= 1000 && STATE == 1) {
        DEBUG_PRINT("X is tilted for 1 second ///////////////////////////////////////");

        if (lastAngle == "X+" && angleX <= -TILT_THRESHOLD) { // Checks if it was tilted to the opposite side of the last ON tilt
          decreaseBrightness();
          isDecreasingBrightness = true;
          isBrightnessAdjusted = true; // Set the flag when brightness is adjusted
        } else if (lastAngle == "X-" && angleX >= TILT_THRESHOLD) { // Checks if it was tilted to the opposite side of the last ON tilt
          decreaseBrightness();
          isDecreasingBrightness = true;
          isBrightnessAdjusted = true; // Set the flag when brightness is adjusted
        } else {
          increaseBrightness();
          saveLastAngle();
          isIncreasingBrightness = true;
        }

      }
    } else {
      isTiltedX = false;
    }

    // Check if Y axis is tilted
    if(angleY >= TILT_THRESHOLD || angleY <= -TILT_THRESHOLD) {
      if (!isTiltedY) {
        tiltStartTimeY = millis();
        isTiltedY = true;
        isBrightnessAdjusted = false; // Reset the flag when the cube is tilted
        DEBUG_PRINT("Y is tilted");
      }
      
      if (millis() - tiltStartTimeY >= 1000) {
        DEBUG_PRINT("Y is tilted for 1 second ///////////////////////////////////////");
        
        if (lastAngle == "Y+" && angleY <= -TILT_THRESHOLD) { // Checks if it was tilted to the opposite side of the last ON tilt
          decreaseBrightness();
          isDecreasingBrightness = true;
          isBrightnessAdjusted = true; // Set the flag when brightness is adjusted
        } else if (lastAngle == "Y-" && angleY >= TILT_THRESHOLD) { // Checks if it was tilted to the opposite side of the last ON tilt
          decreaseBrightness();
          isDecreasingBrightness = true;
          isBrightnessAdjusted = true; // Set the flag when brightness is adjusted
        } else {
          increaseBrightness();
          saveLastAngle();
          isIncreasingBrightness = true;
        }

      }
    } else {
      isTiltedY = false;
    }

    if ((isTiltedX || isTiltedY) && STATE == 0 && POSITION_STATE == 0) {
      STATE = 1; // Change to ON
      DEBUG_PRINT("State changed to ON");
      POSITION_STATE = 1;
      saveLastAngle(); // Save last direction it was turned on
    } else if ((isTiltedX || isTiltedY) && STATE == 1 && POSITION_STATE == 0) {
        needToFadeToBlack = true;
        POSITION_STATE = 1;
      }

    if ((angleX <= 3 && angleX >= -3) && (angleY <= 3 && angleY >= -3) && POSITION_STATE == 1) {
      if (needToFadeToBlack == true && isIncreasingBrightness == false && !isBrightnessAdjusted)
      {
        STATE = 0;
        DEBUG_PRINT("State changed to OFF");
        needToFadeToBlack = false;
      }
      
      POSITION_STATE = 0;
      tiltStartTimeX = 0;  // Reset the counter for X axis
      tiltStartTimeY = 0;  // Reset the counter for Y axis
      isIncreasingBrightness = false; 
      isDecreasingBrightness = false;
      DEBUG_PRINT("MOVED TO THE CENTER ------------------------------");
    }

    if (STATE == 1 && isDecreasingBrightness == false) {
      fadeToWhite();
    } else if (STATE == 0 && isDecreasingBrightness == false && isIncreasingBrightness == false && !isBrightnessAdjusted) {
      fadeToBlack();
    }

  } else if (digitalRead(CHARGING_PIN) == 0) { // Is disconnected from the charger
    DEBUG_PRINT(" CHARGER DISCONNECTED ");
    if(bat_percentage > 20) { // It will be turned on only if the battery percentage is avobe 20%, then it will automatically turn off to avoid the leds turning orange due to low voltage
      POSITION_STATE = 1;
      fadeToWhite();
    } else {
      fadeToBlack;
    }
    
    comesFromDisconnected = true;
  }
  
  FastLED.show();

}

void calculateAngles() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= 10) {  // Fade speed
    lastUpdateTime = currentTime;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Update the Madgwick filter
    filter.updateIMU(gx / G_R, gy / G_R, gz / G_R, ax, ay, az);

    // Get the Euler angles
    angleX = filter.getRoll() - offsetX;
    angleY = filter.getPitch() - offsetY;
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calibrate(int calibrationCount){ // Calibration routine (calibrationCount == readings to take for calibration)
  for (int i = 0; i < calibrationCount; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    filter.updateIMU(gx / G_R, gy / G_R, gz / G_R, ax, ay, az);
    offsetX += filter.getRoll();
    offsetY += filter.getPitch();
    delay(1);  // Wait a bit before the next reading
  }
  offsetX /= calibrationCount;  // Average to get the offset
  offsetY /= calibrationCount;
}

void fadeToWhite() {
  //DEBUG_PRINT(" fadeToWhite ");
  unsigned long currentMillis = millis();
  if(currentMillis - previousFadeWhiteMillis >= 20) { // Fade speed (the higher the slower)
    previousFadeWhiteMillis = currentMillis;
    if(brightness < savedBrightness && isDecreasingBrightness == false) {
      if (savedBrightness < 40) {
        brightness ++; // Increases the brightness very slow when savedBrightness is low.
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
      } else {
        brightness += 3;
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
      }
      
    }
  }
}

void fadeToBlack() {
  unsigned long currentMillis = millis();
  if(savedBrightness < 40) {
    if(currentMillis - previousFadeBlackMillis >= 20) { // Fade speed (the higher the slower)
      previousFadeBlackMillis = currentMillis;
      if(brightness > 0) {
          brightness--;
          fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
          FastLED.show();
      }
    }
  } else {
    if(currentMillis - previousFadeBlackMillis >= 4) { // Fade speed (the higher the slower)
      previousFadeBlackMillis = currentMillis;
      if(brightness > 0) {
        if (brightness > 75) { // Increase the speed when values are above 75 since eyes won't see much of a change
          brightness -= 10;
          fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
          FastLED.show();
        } else {
          brightness--;
          fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
          FastLED.show();
        }
      }
    }
  }
  
}

void increaseBrightness() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousIncreaseMillis >= 7) {
    previousIncreaseMillis = currentMillis;
    if(brightness < 255) {
      brightness++;
      fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
      FastLED.show();
      savedBrightness = brightness;
    }
  }
}

void decreaseBrightness() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousIncreaseMillis >= 10) { // Fade speed (the higher the slower)
    previousIncreaseMillis = currentMillis;
    if(brightness > 15) {
      if (brightness > 150) { // Increase the speed when values are above 150 since eyes won't see much of a change
        brightness -= 3;
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
      } else if (brightness > 75) { // Increase the speed when values are above 75 since eyes won't see much of a change
        brightness -= 2;
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
      } else {
        brightness--;
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
      }
      savedBrightness = brightness;
    }
  }
}

void saveLastAngle() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTiltTime > debounceDelay) {
    if (abs(angleY) <= TILT_THRESHOLD && angleX >= TILT_THRESHOLD) {
      lastAngle = "X+";
    } else if (abs(angleY) <= TILT_THRESHOLD && angleX <= -TILT_THRESHOLD) {
      lastAngle = "X-";
    } else if (abs(angleX) <= TILT_THRESHOLD && angleY >= TILT_THRESHOLD) {
      lastAngle = "Y+";
    } else if (abs(angleX) <= TILT_THRESHOLD && angleY <= -TILT_THRESHOLD) {
      lastAngle = "Y-";
    }
    lastTiltTime = currentMillis;
    DEBUG_PRINT(" LAST ANGLE WAS " + lastAngle);
  }
}