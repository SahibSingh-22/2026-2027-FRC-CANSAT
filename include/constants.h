// config.h
#pragma once


extern SemaphoreHandle_t serialMonitorMutex;
extern SemaphoreHandle_t sdMutex;

// ==========================================
// PIN DEFINITIONS
// ==========================================
constexpr int I2C_SDA = 47;
constexpr int I2C_SCL = 21;
constexpr int LED_ONBOARD = 33;

// ==========================================
// TIMING & SAMPLING
// ==========================================
int unsigned long SAMPLE_INTERVAL_MS = 100;

// ==========================================
// FLIGHT THRESHOLDS
// ==========================================
constexpr float RELEASE_ACCEL_MAX = 3.0f;
constexpr float RELEASE_ALT_DROP = 0.5f;
constexpr int RELEASE_COUNT_REQUIRED = 3;

constexpr float LANDING_ALT_THRESHOLD = 2.0f;
constexpr float LANDING_ACCEL_THRESHOLD = 7.0f;
constexpr float LANDING_ALTCHANGE_THRESHOLD = 0.5f;
constexpr int LANDING_COUNT_REQUIRED = 20;