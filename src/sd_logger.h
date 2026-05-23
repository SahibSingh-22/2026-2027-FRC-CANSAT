#pragma once

#include <Arduino.h>
#include "sensors.h"

extern SemaphoreHandle_t serialMonitorMutex;

#define SD_MMC_CMD 47
#define SD_MMC_CLK 39
#define SD_MMC_D0 40
#define SD_MMC_D1 41
#define SD_MMC_D2 42
#define SD_MMC_D3 38

#define LOG_FILENAME "/flight_data.csv"

// functions
bool sd_init();

void sd_log(telemetryData sensorData);
