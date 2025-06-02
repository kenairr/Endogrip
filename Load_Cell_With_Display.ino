// Display
#include <TM1637.h>
int CLK = 4;
int DIO = 5;
TM1637 tm(CLK, DIO);

bool abortTest = false;

// Load Cell
#include "HX711.h"
#define DOUT  3
#define LOAD_CLK  2
const float calibration_factor = -117940.0;
HX711 scale;

// MPU6050
#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpuForearm(Wire);
MPU6050 mpuHand(Wire);

unsigned long startTime;

// Moving Average to Calculate Offset of MPU6050
// Smooth out noisy sensor readings by averaging them, calculate measurement relative to baseline position
#define NUM_SAMPLES 10
//store the last 10 readings 
float forearmBuffer[NUM_SAMPLES] = {0};
float handBuffer[NUM_SAMPLES] = {0};
int bufferIndex = 0;

float getAverage(float* buffer) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += buffer[i]; //index 
  }
  return sum / NUM_SAMPLES;
}

// Offsets
float forearmZero = 0;
float handZero = 0;
bool zeroSet = false; //once enough data collected, set to True 

void setup() {
  // Display
  tm.init();
  tm.set(2);

  // Serial
  Serial.begin(9600);

  // Load Cell
  scale.begin(DOUT, LOAD_CLK);
  scale.set_scale(calibration_factor);
  scale.tare();

  // I2C + MPU setup
  Wire.begin();

  // Forearm MPU (0x68)
  // If AD0 is connected to GND - 0x68 
  Wire.beginTransmission(0x68); //start communication with device at 0x68 
  Wire.write(0x6B); Wire.write(0); //wake up sensor
  Wire.endTransmission(true);
  mpuForearm.begin();
  mpuForearm.calcOffsets(true, true);

  // Hand MPU (0x69)
  // If AD0 is connected to VCC - 0x69
  Wire.beginTransmission(0x69); //start communication with device at 0x69
  Wire.write(0x6B); Wire.write(0); //wake up sensor 
  Wire.endTransmission(true);
  mpuHand.setAddress(0x69);
  mpuHand.begin();
  mpuHand.calcOffsets(true, true);

  Serial.println("Place wrist in baseline position.");
  Serial.println("Press any key to set zero.");
  startTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - startTime) / 1000.0; //in seconds

  if (elapsedTime >= 80.00) { //80 for Fixed Load Test 
    while (true); // Stop after 30 seconds for MVC Test
  }

  if (Serial.available() > 0) {
  char input = Serial.read();
  if (input == '1') {
    Serial.println("Test aborted by user.");
    abortTest = true;
  }
}

  if (abortTest) {
    while (true); // Stop after abort
  }

  //read new data from MPU6050
  mpuForearm.update();
  mpuHand.update();

  // gets current raw pitch angle 
  float rawForearm = mpuForearm.getAngleX();
  float rawHand = mpuHand.getAngleX();

  if (Serial.available() && !zeroSet) { //input from serial monitor starts offset calibration
    float sumForearm = 0;
    float sumHand = 0;

    //collect 10 readings pausing 100ms between each, average for zeroing
    for (int i = 0; i < NUM_SAMPLES; i++) {
      mpuForearm.update();
      mpuHand.update();
      sumForearm += mpuForearm.getAngleX();
      sumHand += mpuHand.getAngleX();
      delay(100);
    }

    // calculate average baseline angle, stops calibration
    forearmZero = sumForearm / NUM_SAMPLES;
    handZero = sumHand / NUM_SAMPLES;
    zeroSet = true;
  }

  //run after calibration
  if (zeroSet) {
    // subtract offset to get correct angle 
    float correctedForearm = rawForearm - forearmZero;
    float correctedHand = rawHand - handZero;

    forearmBuffer[bufferIndex] = correctedForearm;
    handBuffer[bufferIndex] = correctedHand;
    bufferIndex = (bufferIndex + 1) % NUM_SAMPLES; 

    // moving average to smooth out noise
    float smoothForearm = getAverage(forearmBuffer);
    float smoothHand = getAverage(handBuffer);
    // calcuate wrist angle (forearm angle - hand angle)
    float wristAngle = smoothHand - smoothForearm;

    float weight = scale.get_units();

    Serial.print(elapsedTime, 2);
    Serial.print(" | ");
    Serial.print(weight, 3);
    Serial.print(" | ");
    Serial.print(wristAngle, 2);
    Serial.println("°");

    // Display
    float weight_display = abs(weight);
    // convert to integer to show on display
    int weightInGrams = int(weight_display * 100);
    // only show ones and tenths digits 
    int ones = weightInGrams / 100;
    int tenths = (weightInGrams / 10) % 10;

    tm.display(0, ones);
    tm.point(1);
    tm.display(2, tenths);
  }

  delay(5);
}

