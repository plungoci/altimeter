#pragma once

#include <Arduino.h>

float pressureToAltitudeRelative(float pressurePa, float pressureRefPa);

class VarioMath {
 public:
  explicit VarioMath(float emaAlpha);

  void reset(float altitudeM, uint32_t timeUs);
  float updateAltitude(float rawAltitudeM);
  float computeVario(float filteredAltitudeM, uint32_t nowUs);

 private:
  float alpha_;
  bool initialized_;
  float filteredAltitudeM_;
  float prevAltitudeM_;
  uint32_t prevTimeUs_;
};
