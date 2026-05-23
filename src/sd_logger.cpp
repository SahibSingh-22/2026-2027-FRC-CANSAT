#include "sd_logger.h"
#include <FS.h>
#include <SD_MMC.h>

static uint32_t rowCount = 0;
bool sd_init()
{
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0, SD_MMC_D1, SD_MMC_D2, SD_MMC_D3);
    if (!SD_MMC.begin("/sdcard", false))
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("SD Card Mount Failed");
            xSemaphoreGive(serialMonitorMutex);
        }
        return false;
    }
    else
    {
        return true;
    }
}

void sd_log(telemetryData sensorData)
{
    // idk the what exactly we are measuring so i put random ones down feel free to change them
    File logFile = SD_MMC.open(LOG_FILENAME, FILE_APPEND);

    if (!logFile)
    {
        if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("Could not open file to write row.");
            xSemaphoreGive(serialMonitorMutex);
        }
        return;
    }

    rowCount++;
    logFile.print(rowCount);
    logFile.print(",");
    logFile.print(millis(), 2);
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
    logFile.println(sensorData.gyroData.gyroZ);

    logFile.close();
    if (xSemaphoreTake(serialMonitorMutex, portMAX_DELAY) == pdTRUE)
    {
        Serial.print("[SD] Row written: ");
        Serial.println(rowCount);
        xSemaphoreGive(serialMonitorMutex);
    }
}