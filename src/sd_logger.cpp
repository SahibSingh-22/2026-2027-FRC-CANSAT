#include "sd_logger.h"
#include <FS.h>
#include <SD_MMC.h>

static uint32_t rowCount = 0;
bool sd_init() {
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0, SD_MMC_D1, SD_MMC_D2, SD_MMC_D3);
    if (!SD_MMC.begin("/sdcard",false)){
        Serial.println("SD Card Mount Failed");
        return false;
    }
    else{
        return true;
    }
}

void sd_log(float temperature, float pressure, float altitude) {
    //idk the what exactly we are measuring so i put random ones down feel free to change them
    File logFile = SD_MMC.open(LOG_FILENAME, FILE_APPEND);

    if (!logFile) {
        Serial.println("Could not open file to write row.");
        return;
    }

    rowCount++;
    logFile.print(rowCount);
    logFile.print(",");
    logFile.print(temperature, 2);
    logFile.print(",");
    logFile.print(pressure, 2);
    logFile.print(",");
    logFile.println(altitude, 2);

    logFile.close();

    Serial.print("[SD] Row written: ");
    Serial.println(rowCount);
}