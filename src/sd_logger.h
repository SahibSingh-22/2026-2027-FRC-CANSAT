#pragma once

#include <Arduino.h>
#include "mpu6050.h"

// constants
#define SD_CS_PIN 5

#define LOG_FILENAME "/flight_data.csv"

// functions
bool sd_init();

void sd_log(float temperature, float pressure, float altitude, sensorAccel accelData, sensorGyro gyroData);

void sd_flush();
