#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "camera.h"
#include "sd_logger.cpp"

#define I2C_SDA 41     // I2C data
#define I2C_SCL 42     // I2C clock
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

// Sensors
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// State variables
FlightState state = PREDROP;

float altitude = 0.0f;
float previousAltitude = 0.0f;
float groundAltitude = 0.0f;
float temp = 0.0f;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

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
void calibrateGroundAltitude();
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

  calibrateGroundAltitude();

  ledBlink(3, 150, 150);

  initCamera();
  initSD();
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
  if (!bmp.begin(0x77))
  {
    Serial.println("BMP280 fail");
    return false;
  }

  if (!mpu.begin())
  {
    Serial.println("MPU6050 fail");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

// Read sensors
void readSensors()
{
  previousAltitude = altitude;
  float pressure = bmp.readPressure();
  temp = bmp.readTemperature();

  float rawAltitude = 44330.0f * (1.0f - pow(pressure / 101325.0f, 0.1903f)) - groundAltitude;

  if (!emaReady)
  {
    altitude = rawAltitude;
    emaReady = true;
  }
  else
  {
    altitude = EMA_ALPHA * altitude + (1.0f - EMA_ALPHA) * rawAltitude;
  }

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  sd_log(temp, pressure, altitude);
}

void checkRelease()
{
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
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

void calibrateGroundAltitude()
{
  float sum = 0;

  for (int i = 0; i < 20; i++)
  {
    float pressure = bmp.readPressure();

    sum += 44330.0f *
           (1.0f - pow(pressure / 101325.0f, 0.1903f));

    delay(50);
  }

  groundAltitude = sum / 20;
}

// Detect landing
void checkLanding()
{
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
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