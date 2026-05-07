#include "Arduino.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "camera.h"

// Freenove ESP32-S3 WROOM camera pin definitions
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

#define SD_MMC_CMD        38
#define SD_MMC_CLK        39
#define SD_MMC_D0         40

File videoFile;
bool recording = false;
int frameCount = 0;
String filePath;

void initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 12;              // 0-63, lower = better quality
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        while (true) delay(1000);
    }
    Serial.println("Camera initialized.");
}

void initSD() {
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (!SD_MMC.begin("/sdcard", true)) { // true = 1-bit mode
        Serial.println("SD card mount failed!");
        while (true) delay(1000);
    }
    Serial.println("SD card initialized.");
}

void startRecording() {
    // Find a unique filename
    int index = 0;
    do {
        filePath = "/CansatRecording" + String(index++) + ".mjpeg";
    } while (SD_MMC.exists(filePath));

    videoFile = SD_MMC.open(filePath, FILE_WRITE);
    if (!videoFile) {
        Serial.println("Failed to open file for writing!");
        return;
    }
    recording = true;
    frameCount = 0;
    Serial.printf("Recording started: %s\n", filePath.c_str());
}

void stopRecording() {
    if (recording) {
        videoFile.close();
        recording = false;
        Serial.printf("Recording stopped. Frames saved: %d\n", frameCount);
        Serial.printf("File saved to: %s\n", filePath.c_str());
    }
}

void captureFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame capture failed");
        return;
    }

    // Write frame size then frame data (simple MJPEG container)
    uint32_t size = fb->len;
    videoFile.write((uint8_t*)&size, sizeof(size));
    videoFile.write(fb->buf, fb->len);
    frameCount++;

    esp_camera_fb_return(fb);
}

