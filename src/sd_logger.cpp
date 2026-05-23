#include "sd_logger.h"
#include <FS.h>
#include <SD_MMC.h>

#define SD_MMC_CMD        38
#define SD_MMC_CLK        39
#define SD_MMC_D0         40


#define LOG_FILENAME "/flight_data.csv"

static uint32_t rowCount = 0;
void sd_init() {
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (!SD_MMC.begin("/sdcard",true)){
        Serial.println("SD Card Mount Failed");
        return;
    }
    else{
        Serial.println("SD Card Mount Success");
        return;
    }
}

void sd_log(telemetryData sensorData) {
    //idk the what exactly we are measuring so i put random ones down feel free to change them
    File logFile = SD_MMC.open(LOG_FILENAME, FILE_APPEND);

    if (!logFile)
    {
        Serial.println("Could not open file to write row.");
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

    Serial.print("[SD] Row written: ");
    Serial.println(rowCount);
}