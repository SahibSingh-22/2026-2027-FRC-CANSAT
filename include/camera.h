#pragma once
#include "FS.h"

// Function declarations

extern SemaphoreHandle_t serialMonitorMutex;

void initCamera();
void startRecording();
void stopRecording();
void captureFrame();

// Shared variables
extern File videoFile;
extern bool recording;
extern int frameCount;
extern String filePath;