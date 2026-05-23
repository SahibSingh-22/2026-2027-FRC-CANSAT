#pragma once
#include "FS.h"

// Function declarations
void initCamera();
void startRecording();
void stopRecording();
void captureFrame();

// Shared variables
extern File videoFile;
extern bool recording;
extern int frameCount;
extern String filePath;