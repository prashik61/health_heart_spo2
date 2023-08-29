#include <Wire.h>
#include "MAX30105.h"
#include "spo2_custom_algorithm.h"

// Create an instance of the MAX30105 sensor
MAX30105 customSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t customIRBuffer[100];
uint16_t customRedBuffer[100];
#else
uint32_t customIRBuffer[100];
uint32_t customRedBuffer[100];
#endif

int32_t customBufferLength;
int32_t customSpO2;
int8_t validCustomSpO2;
int32_t customHeartRate;
int8_t validCustomHeartRate;

byte customPulseLED = 11;
byte customReadLED = 13;

void customSetup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes for LEDs
  pinMode(customPulseLED, OUTPUT);
  pinMode(customReadLED, OUTPUT);

  // Check if the custom sensor is detected
  if (!customSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println(F("MAX30105 not detected. Check wiring/power."));
    while (1);
  }

  // Prompt the user to attach the sensor
  Serial.println(F("Attach sensor to finger using band. (Press any key to start."));
  while (Serial.available() == 0);// after serial start press any key or if dont want comment this line 
  Serial.read(); // omment this line also

  // Sensor configuration parameters
  byte customBrightness = 60;
  byte customSampleAverage = 4;
  byte customLedMode = 2;
  byte customSampleRate = 100;
  int customPulseWidth = 411;
  int customAdcRange = 4096;

  // Configure the custom sensor
  customSensor.setup(customBrightness, customSampleAverage, customLedMode, customSampleRate, customPulseWidth, customAdcRange);
}

void customLoop()
{
  customBufferLength = 100;

  // Read the initial 100 samples to determine signal range
  for (byte i = 0; i < customBufferLength; i++)
  {
    while (customSensor.available() == false)
      customSensor.check();

    customRedBuffer[i] = customSensor.getRed();
    customIRBuffer[i] = customSensor.getIR();
    customSensor.nextSample();

    Serial.print(F("cr="));
    Serial.print(customRedBuffer[i], DEC);
    Serial.print(F(", ci="));
    Serial.println(customIRBuffer[i], DEC);
  }

  // Calculate heart rate and SpO2 after the first 100 samples
  calculateCustomHRAndSpO2(customIRBuffer, customBufferLength, customRedBuffer, &customSpO2, &validCustomSpO2, &customHeartRate, &validCustomHeartRate);

  while (1)
  {
    // Shift data in memory to make room for new samples
    for (byte i = 25; i < 100; i++)
    {
      customRedBuffer[i - 25] = customRedBuffer[i];
      customIRBuffer[i - 25] = customIRBuffer[i];
    }

    // Take 25 new samples before calculating heart rate
    for (byte i = 75; i < 100; i++)
    {
      while (customSensor.available() == false)
        customSensor.check();

      digitalWrite(customReadLED, !digitalRead(customReadLED));

      customRedBuffer[i] = customSensor.getRed();
      customIRBuffer[i] = customSensor.getIR();
      customSensor.nextSample();

      // Print samples and calculation results to the terminal
      Serial.print(F("cr="));
      Serial.print(customRedBuffer[i], DEC);
      Serial.print(F(", ci="));
      Serial.print(customIRBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(customHeartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validCustomHeartRate, DEC);

      Serial.print(F(", SpO2="));
      Serial.print(customSpO2, DEC);

      Serial.print(F(", SpO2Valid="));
      Serial.println(validCustomSpO2, DEC);
    }

    // Recalculate heart rate and SpO2 using the new samples
    calculateCustomHRAndSpO2(customIRBuffer, customBufferLength, customRedBuffer, &customSpO2, &validCustomSpO2, &customHeartRate, &validCustomHeartRate);
  }
}

void setup()
{
  customSetup();
}

void loop()
{
  customLoop();
}
