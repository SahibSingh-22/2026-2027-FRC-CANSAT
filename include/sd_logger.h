#pragma once

#include <Arduino.h>

#include "sensors.h"

// functions
void sd_init();

void sd_log(telemetryData sensorData);
