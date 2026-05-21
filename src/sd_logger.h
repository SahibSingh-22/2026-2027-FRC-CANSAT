#pragma once

#include <Arduino.h>

//constants
#define SD_CS_PIN 5

#define LOG_FILENAME "/flight_data.csv"

//functions
bool sd_init();

void sd_log(float temperature, float pressure, float altitude);

void sd_flush();
