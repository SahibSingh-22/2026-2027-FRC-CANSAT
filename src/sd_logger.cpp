#include "sd_logger.h"
#include "FS.h"
#include "SD_MMC.h"
#include "mpu6050.h"
#include <Arduino.h>

static uint32_t rowCount = 0;
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0 40

#define LOG_FILENAME "/flight_data.csv"


void sd_init()
{
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (!SD_MMC.begin("/sdcard", true))
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("SD Card Mount Failed");
            xSemaphoreGive(serialMonitorMutex);
        }
        return;
    }
    else
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("SD Card Mount Success");
            xSemaphoreGive(serialMonitorMutex);
        }
        return;
    }
}

void sd_log(telemetryData sensorData)
{
    // Changed "SD" to "SD_MMC" and "FILE_WRITE" to "FILE_APPEND" so data adds to the end
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE)
    {
            File logFile = SD_MMC.open(LOG_FILENAME, FILE_APPEND);

        if (!logFile)
        {
            
            if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
            {
                Serial.println("[SDMMC] WARNING: Could not open file to write row.");
                xSemaphoreGive(serialMonitorMutex);
            }
            return;
        }

        rowCount++;
        logFile.print(rowCount);
        logFile.print(",");
        
        // Note: millis() returns an integer, so the second parameter (decimals) is removed
        logFile.print(millis()); 
        logFile.print(",");
        
        logFile.print(sensorData.temp, 3);
        logFile.print(",");
        logFile.print(sensorData.pressure, 3);
        logFile.print(",");
        logFile.print(sensorData.altitude, 3);
        logFile.print(",");
        logFile.print(sensorData.accelData.accelX, 3);
        logFile.print(",");
        logFile.print(sensorData.accelData.accelY, 3);
        logFile.print(",");
        logFile.print(sensorData.accelData.accelZ, 3);
        logFile.print(",");
        logFile.print(sensorData.gyroData.gyroX, 3);
        logFile.print(",");
        logFile.print(sensorData.gyroData.gyroY, 3);
        logFile.print(",");
        logFile.println(sensorData.gyroData.gyroZ, 3); // Added decimal precision here to match the others

        logFile.close();

        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.print("[SDMMC] Row written: ");
            Serial.println(rowCount);
            xSemaphoreGive(serialMonitorMutex);
        }
            xSemaphoreGive(sdMutex);
    }

}

