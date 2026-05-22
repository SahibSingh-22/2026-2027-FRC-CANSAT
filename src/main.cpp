#include <Arduino.h>

#include <Wire.h>
#include "bmp280.h"
#include "mpu6050.h"
#include "camera.h"
#include "sd_logger.h"

#define I2C_SDA 47     // I2C data
#define I2C_SCL 21     // I2C clock
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

float altitude = 0.0f;
float previousAltitude = 0.0f;
float groundAltitude = 0.0f;
float temp = 0.0f;
float pressure = 0.0f;
sensorAccel accelData;
sensorGyro gyroData;

unsigned long now = 0;
unsigned long recordingStart = 0;
unsigned long recordingStop = 0;
unsigned long lastSampleTime = 0;

// Timing
const unsigned long SAMPLE_INTERVAL_MS = 100; // 10 Hz

// EMA filter
const float EMA_ALPHA = 0.7f;
bool emaReady = false;

// Thresholds
const float LANDING_ALT_THRESHOLD = 2.0f;
const float LANDING_ACCEL_MAX = 11.0f;

// counters
int dropCounter = 0;
int landedCounter = 0;

// Function declarations
bool initSensors();
void readSensors();
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

  if (!initSensors())
  {
    errorSignal("Sensor init failed!");
  }

  groundAltitude = calibrateGroundAltitude();

  ledBlink(3, 150, 150);

  initCamera();
  initSD();

  startRecording();
}

// Loop
void loop()
{
  now = millis();
  if (!recording && now - recordingStop >= 10000)
  {
    recording = true;
    recordingStart = now;
  }

  if (recording)
  {
    captureFrame();

    // Stop after 10 seconds
    if (now - recordingStart >= 10000)
    {
      stopRecording();
      recordingStop = now;
      recording = false;
    }
  }

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS)
  {
    lastSampleTime = now;

    readSensors();

    switch (state)
    {

    case PREDROP:
      checkRelease();
      break;

    case RELEASED:
      if (!recording)
      {
        recording = true;
        recordingStart = now;
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
}

// Sensor init
bool initSensors()
{
  if (!initMPU())
  {
    return false;
  }
  if (!initBMP())
  {
    return false;
  }
  return true;
}

// Read sensors
void readSensors()
{
  previousAltitude = altitude;

  accelData = getAccel();
  gyroData = getGyro();

  pressure = getPressure();
  altitude = getAltitude(previousAltitude, groundAltitude, emaReady);
  temp = getTemp();

  sd_log(temp, pressure, altitude, accelData, gyroData);
}

void checkRelease()
{
  float totalAccel = sqrt(accelData.accelX * accelData.accelX + accelData.accelY * accelData.accelY + accelData.accelZ * accelData.accelZ);
  float altitudeDrop = previousAltitude - altitude;

  if (totalAccel < 3.0 && altitudeDrop > 0.5)
  {
    dropCounter += 1;
  }
  else
  {
    dropCounter = 0;
  }

  if (dropCounter >= 3)
  {
    state = RELEASED;
    Serial.println("Release detected");
  }
}

// Detect landing
void checkLanding()
{
  float totalAccel = sqrt(accelData.accelX * accelData.accelX + accelData.accelY * accelData.accelY + accelData.accelZ * accelData.accelZ);
  float altitudeChange = abs(previousAltitude - altitude);
  if (totalAccel > 7.0 && altitude < LANDING_ALT_THRESHOLD && totalAccel < 11.5 && altitudeChange < 0.5)
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