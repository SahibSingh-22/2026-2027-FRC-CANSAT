#pragma once

#include <Arduino.h>
#include "mpu6050.h"
#include "bmp280.h"

extern SemaphoreHandle_t serialMonitorMutex;

struct telemetryData
{
    sensorAccel accelData;
    sensorGyro gyroData;
    float temp;
    float altitude;
    float pressure;
};

bool initSensors();
telemetryData getSensorData();
void calibrateGroundAltitude();