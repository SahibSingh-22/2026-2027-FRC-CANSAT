#pragma once

struct sensorAccel
{
    float accelX;
    float accelY;
    float accelZ;
};

struct sensorGyro
{
    float gyroX;
    float gyroY;
    float gyroZ;
};

bool initMPU();
sensorAccel getAccel();
sensorGyro getGyro();
