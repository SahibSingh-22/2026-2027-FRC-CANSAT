#include "sd_logger.h"
#include <SD.h>
#include <SPI.h>

static uint32_t rowCount = 0;
bool sd_init() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] ERROR: Card not found or failed to mount.");
        Serial.println("[SD] Check wiring and that CS pin matches SD_CS_PIN.");
        return false;
    }

    File logFile = SD.open(LOG_FILENAME, FILE_WRITE);

    if (!logFile) {
        Serial.println("[SD] ERROR: Could not open log file for writing.");
        return false;
    }

    logFile.println("row,temperature_c,pressure_pa,altitude_m");
    logFile.close();

    Serial.println("[SD] Initialised. Logging to: " LOG_FILENAME);
    return true;
}

void sd_log(float temperature, float pressure, float altitude) {
    //idk the what exactly we are measuring so i put random ones down feel free to change them
    File logFile = SD.open(LOG_FILENAME, FILE_WRITE);

    if (!logFile) {
        Serial.println("[SD] WARNING: Could not open file to write row.");
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
void sd_flush() {
    File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) logFile.close();
}
