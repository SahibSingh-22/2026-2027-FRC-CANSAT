#include "Arduino.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "camera.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

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
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return;  // don't hang, let caller decide
    }
    Serial.println("Camera initialized.");
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

