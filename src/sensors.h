#pragma once

#include "mpu6050.h"
#include "bmp280.h"

extern float previousAltitude;
extern float groundAltitude;
extern bool emaReady;

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