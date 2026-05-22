#pragma once

bool initBMP();
float getTemp();
float getAltitude(float previousAltitude, float groundAltitude, bool &emaReady);
float getPressure();