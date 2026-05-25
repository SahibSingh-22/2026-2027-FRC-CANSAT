#include <Arduino.h>
#include <Wire.h>

#include "constants.h"

#include "sensors.h"
#include "bmp280.h"
#include "mpu6050.h"

#include "camera.h"
#include "sd_logger.h"
#include "constants.h"

// Flight states
enum FlightState
{
  PREDROP,
  DESCENT,
  LANDED
};

const char *stateToString(FlightState s)
{
  switch (s)
  {
  case PREDROP:
    return "PREDROP";
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

telemetryData sensorData;
QueueHandle_t telemetryQueue; // create the queue
SemaphoreHandle_t serialMonitorMutex;
SemaphoreHandle_t sdMutex;

int landedCounter = 0;
int dropCounter = 0;

float previousAltitude = 0.0f;

unsigned long now = 0;
unsigned long lastSampleTime = 0;

// Function declarations
void checkRelease();
void checkLanding();
void errorSignal(const char *message);

// function task which will be run on core 0
void storageAndCameraTask(void *parameter)
{                   // this parameter won't be used in our code but it is important for tasks but it can be used to pass in and make multiple tasks from the same function
  // things to do before we go into loop for core 0
  sd_init();
  initCamera();
  startRecording(); 

  telemetryData receivedData; // data which is getting received from the queue
  while (true)
  {
    captureFrame();
    while (xQueueReceive(telemetryQueue, &receivedData, 0) == pdPASS)
    { // keep pulling data from queue until there is none left before going to captureFram() this is due to captureFrame() taking longer than other tasks so there will be a few items in queue by the time it comes back around
      sd_log(receivedData);
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // IMPORTANT we need to give the core a break to do background tasks, this is baked into loop() but not into FreeRTOS tasks
  }
}

// Setup
void setup()
{
  Serial.begin(115200);
  delay(1000);

  serialMonitorMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();

  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, HIGH); // LED off

  Wire.begin(I2C_SDA, I2C_SCL);
  
  if (!initSensors())
  {
    // errorSignal("Sensor init failed!");
    Serial.println("Sensor init failed!");
  }
  calibrateGroundAltitude();

  telemetryQueue = xQueueCreate(20, sizeof(telemetryData));                                      // give number of items that will get stored in queue and size of the type
  xTaskCreatePinnedToCore(storageAndCameraTask, "storageAndCameraTask", 16384, NULL, 1, NULL, 0); // create task, pass in function, label, memory allocated for this task, priorty and the core number which will run task

  sensorData = getSensorData();
}

// Loop
void loop()
{

  now = millis();

  
  
  // Sample sensors every 100ms
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS)
  {
    lastSampleTime = now;

    previousAltitude = sensorData.altitude;

    sensorData = getSensorData();
    xQueueSend(telemetryQueue, &sensorData, 0); // Send sensorData to TelemetryQueue

    switch (state)
    {

    case PREDROP:
      checkRelease();
      break;

    case DESCENT:
      checkLanding();
      break;

    case LANDED:
      digitalWrite(LED_ONBOARD, (now / 500) % 2 == 0 ? LOW : HIGH);
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

  if (dropCounter > 5)
  {
    state = DESCENT;
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
    SAMPLE_INTERVAL_MS = 300; // slow down data recording after landing
  }
}
// Error loop
void errorSignal(const char *message)
{
  if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
  {
    Serial.println(message);
    xSemaphoreGive(serialMonitorMutex);
  }

  while (true)
  {
    digitalWrite(LED_ONBOARD, LOW);
    delay(200);
    digitalWrite(LED_ONBOARD, HIGH);
    delay(200);
  }
}