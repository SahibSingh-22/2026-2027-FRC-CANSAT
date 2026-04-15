#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

#define I2C_SDA 13     // I2C data
#define I2C_SCL 15     // I2C clock
#define LED_ONBOARD 33 // LED active LOW

// Flight states
enum FlightState
{
  PRELAUNCH,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

const char *stateToString(FlightState s)
{
  switch (s)
  {
  case PRELAUNCH:
    return "PRELAUNCH";
  case ASCENT:
    return "ASCENT";
  case APOGEE:
    return "APOGEE";
  case DESCENT:
    return "DESCENT";
  case LANDED:
    return "LANDED";
  default:
    return "UNKNOWN";
  }
}

// Sensors
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

// State variables
FlightState state = PRELAUNCH;

float altitude = 0.0f;
float maxAltitude = 0.0f;
float groundAltitude = 0.0f;
float temp = 0.0f;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

unsigned long now = 0;
unsigned long lastSampleTime = 0;

// Timing
const unsigned long SAMPLE_INTERVAL_MS = 100; // 10 Hz

// EMA filter
const float EMA_ALPHA = 0.9f;
bool emaReady = false;

// Thresholds
const float LAUNCH_ACCEL_THRESHOLD = 15.0f;
const float LAUNCH_ALT_THRESHOLD = 2.0f;
const float APOGEE_DROP_THRESHOLD = 5.0f;
const float LANDING_ALT_THRESHOLD = 2.0f;
const float LANDING_ACCEL_MAX = 11.0f;

// Launch detection counter
int accelCount = 0; // counts consecutive accel spikes

// Function declarations
bool initSensors();
void calibrateGroundAltitude();
void readSensors();
void checkLaunch();
void checkApogee();
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
}

// Loop
void loop()
{
  now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS)
  {
    lastSampleTime = now;

    readSensors();

    switch (state)
    {

    case PRELAUNCH:
      checkLaunch();
      break;

    case ASCENT:
      checkApogee();
      break;

    case APOGEE:
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
  if (!bmp.begin())
  {
    Serial.println("BMP180 fail");
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

// Calibrate ground altitude
void calibrateGroundAltitude()
{
  float sum = 0.0f;

  for (int i = 0; i < 20; i++)
  {
    float pressure = bmp.readPressure();
    sum += 44330.0f * (1.0f - pow(pressure / 101325.0f, 0.1903f));
    delay(50);
  }

  groundAltitude = sum / 20;
  altitude = 0.0f;
  emaReady = true;
}

// Read sensors
void readSensors()
{
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
}

// Detect launch (tuned)
void checkLaunch()
{
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  if (totalAccel > LAUNCH_ACCEL_THRESHOLD)
  {
    accelCount++; // count sustained accel
  }
  else
  {
    accelCount = 0; // reset
  }

  if (accelCount > 5 && altitude > LAUNCH_ALT_THRESHOLD)
  { // sustained accel + altitude
    state = ASCENT;
    maxAltitude = altitude;
    digitalWrite(LED_ONBOARD, LOW); // LED on
    Serial.println("LAUNCH DETECTED");
  }
}

// Detect apogee
void checkApogee()
{
  if (altitude > maxAltitude)
  {
    maxAltitude = altitude;
  }

  if (altitude < (maxAltitude - APOGEE_DROP_THRESHOLD))
  {
    state = APOGEE;
    Serial.println("APOGEE");
  }
}

// Detect landing
void checkLanding()
{
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  if (altitude < LANDING_ALT_THRESHOLD && totalAccel < LANDING_ACCEL_MAX)
  {
    state = LANDED;
    Serial.println("LANDED");
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