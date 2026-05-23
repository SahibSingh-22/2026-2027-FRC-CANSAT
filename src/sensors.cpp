#include "sensors.h"
#include <Arduino.h>

float previousFilteredAltitude = 0.0f;
float groundAltitude = 0.0f;
bool emaReady = false;

bool initSensors()
{
    if (!initBMP())
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("BMP init fail!");
            xSemaphoreGive(serialMonitorMutex);
        }
        return false;
    }
    if (!initMPU())
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("MPU init fail!");
            xSemaphoreGive(serialMonitorMutex);
        }
        return false;
    }
    if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
    {
        Serial.println("Sensors init success!");
        xSemaphoreGive(serialMonitorMutex);
    }
    return true;
}

telemetryData getSensorData()
{
    telemetryData data;

    data.accelData = getAccel();
    data.gyroData = getGyro();
    data.altitude = getAltitude(previousFilteredAltitude, groundAltitude, emaReady);
    previousFilteredAltitude = data.altitude;
    data.temp = getTemp();
    data.pressure = getPressure();

    return data;
}

void calibrateGroundAltitude()
{
    float sum = 0;

    for (int i = 0; i < 20; i++)
    {
        float pressure = getPressure();

        sum += 44330.0f *
               (1.0f - pow(pressure / 101325.0f, 0.1903f));

        delay(50);
    }

    groundAltitude = sum / 20;
}