#include <Arduino.h>
#include "bmp280.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

float getTemp()
{
    float temp = bmp.readTemperature();
    return temp;
}

float getPressure()
{
    float pressure = bmp.readPressure();
    return pressure;
}

float getAltitude(float previousAltitude, float groundAltitude, bool &emaReady)
{
    float EMA_ALPHA = 0.7;
    float altitude;
    float pressure = getPressure();
    float rawAltitude = 44330.0f * (1.0f - pow(pressure / 101325.0f, 0.1903f)) - groundAltitude;

    if (!emaReady)
    {
        altitude = rawAltitude;
        emaReady = true;
    }
    else
    {
        altitude = EMA_ALPHA * previousAltitude + (1.0f - EMA_ALPHA) * rawAltitude;
    }

    return altitude;
}

bool initBMP()
{
    if (!bmp.begin(0x77))
    {
        return false;
    }
    Serial.println("BMP280 init success!");
    return true;
}