#pragma once

#include <Arduino.h>
#include "sensors.h"

// constants
#define SD_CS_PIN 5

#define LOG_FILENAME "/flight_data.csv"

// functions
bool sd_init();

void sd_log(telemetryData sensorData);

void sd_flush();
