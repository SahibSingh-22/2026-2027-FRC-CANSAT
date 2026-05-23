#include <Arduino.h>
#include <Wire.h>

#include "constants.h"

#include "sensors.h"
#include "bmp280.h"
#include "mpu6050.h"

#include "camera.h"
#include "sd_logger.h"



#define I2C_SDA 1  // I2C data
#define I2C_SCL 2    // I2C clock
#define LED_ONBOARD 33 // LED active LOW

// Flight states
enum FlightState
{
  PREDROP,
  RELEASED,
  DESCENT,
  LANDED
};

const char *stateToString(FlightState s)
{
  switch (s)
  {
  case PREDROP:
    return "PREDROP";
  case RELEASED:
    return "RELEASED";
  case DESCENT:
    return "DESCENT";
  case LANDED:
    return "LANDED";
  default:
    return "UNKNOWN";
  }
}

// State variables
FlightState state = PREDROP;

float previousAltitude = 0.0f;
float groundAltitude = 0.0f;
telemetryData sensorData;

int landedCounter = 0;
int dropCounter = 0;

unsigned long now = 0;

unsigned long lastSampleTime = 0;

// EMA filter
bool emaReady = false;

// Function declarations
void checkRelease();
void checkLanding();
void ledBlink(int times, int onMs, int offMs);
void errorSignal(const char *message);

// Setup
void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, HIGH); // LED off

  Serial.println("Flight Computer Ready");

  Wire.begin(I2C_SDA, I2C_SCL);

  sd_init();
  initCamera();
  
  if (!initSensors())
  {
    // errorSignal("Sensor init failed!");
    Serial.println("Sensor init failed!");
  }
  calibrateGroundAltitude();

  ledBlink(3, 150, 150);
  

  

  startRecording();
}

// Loop
void loop()
{
  now = millis();

  if (recording)
  {
    captureFrame();

    // Stop after 10 seconds
    if (now >= 10000)
    {
      stopRecording();
      recording = false;
    }
  }
  

  
  
  // Sample sensors every 100ms
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS)
  {
    lastSampleTime = now;

    previousAltitude = sensorData.altitude;
    sensorData = getSensorData();
    sd_log(sensorData);

    switch (state)
    {

    case PREDROP:
      checkRelease();
      break;

    case RELEASED:
      if (!recording)
      {
        recording = true;

      }
      state = DESCENT;
      break;

    case DESCENT:
      checkLanding();
      break;

    case LANDED:
      digitalWrite(LED_ONBOARD, (now / 500) % 2 == 0 ? LOW : HIGH); // blink
      break;
    }
  }
  Serial.print("Temperature: ");
  Serial.print(sensorData.temp);
  Serial.print("Pressure: ");
  Serial.print(sensorData.pressure);
  Serial.print("Altitude: ");
  Serial.println(sensorData.altitude);
}





void checkRelease()
{
  float totalAccel = sqrt(sensorData.accelData.accelX * sensorData.accelData.accelX + sensorData.accelData.accelY * sensorData.accelData.accelY + sensorData.accelData.accelZ * sensorData.accelData.accelZ);
  float altitudeDrop = previousAltitude - sensorData.altitude;

  if (totalAccel < 3.0 && altitudeDrop > 0.5)
  {
    dropCounter += 1;
  }
  else
  {
    dropCounter = 0;
  }



}

// Detect landing
void checkLanding()
{
  float totalAccel = sqrt(sensorData.accelData.accelX * sensorData.accelData.accelX + sensorData.accelData.accelY * sensorData.accelData.accelY + sensorData.accelData.accelZ * sensorData.accelData.accelZ);
  float altitudeChange = abs(previousAltitude - sensorData.altitude);
  if (totalAccel > LANDING_ACCEL_THRESHOLD && sensorData.altitude < LANDING_ALT_THRESHOLD && altitudeChange < LANDING_ALTCHANGE_THRESHOLD)
  {
    landedCounter += 1;
  }
  else
  {
    landedCounter = 0;
  }
  if (landedCounter >= 20)
  {
    state = LANDED;
  }
}

// LED blink helper
void ledBlink(int times, int onMs, int offMs)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_ONBOARD, LOW);
    delay(onMs);
    digitalWrite(LED_ONBOARD, HIGH);
    delay(offMs);
  }
}

// Error loop
void errorSignal(const char *message)
{
  Serial.println(message);

  while (true)
  {
    digitalWrite(LED_ONBOARD, LOW);
    delay(200);
    digitalWrite(LED_ONBOARD, HIGH);
    delay(200);
  }
}