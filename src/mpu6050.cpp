#include <Arduino.h>
#include "mpu6050.h"
#include <Adafruit_sensor.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

sensorAccel getAccel()
{
    sensorAccel data;

    sensors_event_t a;
    mpu.getEvent(&a, nullptr, nullptr);

    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;

    return data;
}

sensorGyro getGyro()
{
    sensorGyro data;

    sensors_event_t g;
    mpu.getEvent(nullptr, &g, nullptr);

    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;

    data.gyroX = data.gyroX * 9.549296;
    data.gyroY = data.gyroY * 9.549296;
    data.gyroZ = data.gyroZ * 9.549296;

    return data;
}

bool initMPU()
{
    if (!mpu.begin())
    {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
}
