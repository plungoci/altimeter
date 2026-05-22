#include "VarioMath.h"

#include <math.h>

float pressureToAltitudeRelative(float pressurePa, float pressureRefPa) {
  if (pressurePa <= 0.0f || pressureRefPa <= 0.0f) return 0.0f;
  // Barometric formula relative to reference pressure.
  return 44330.0f * (1.0f - powf(pressurePa / pressureRefPa, 0.19029495718f));
}

VarioMath::VarioMath(float emaAlpha)
    : alpha_(constrain(emaAlpha, 0.01f, 1.0f)),
      initialized_(false),
      filteredAltitudeM_(0.0f),
      prevAltitudeM_(0.0f),
      prevTimeUs_(0) {}

void VarioMath::reset(float altitudeM, uint32_t timeUs) {
  filteredAltitudeM_ = altitudeM;
  prevAltitudeM_ = altitudeM;
  prevTimeUs_ = timeUs;
  initialized_ = true;
}

float VarioMath::updateAltitude(float rawAltitudeM) {
  if (!initialized_) {
    filteredAltitudeM_ = rawAltitudeM;
    prevAltitudeM_ = rawAltitudeM;
    initialized_ = true;
    return filteredAltitudeM_;
  }

  filteredAltitudeM_ = alpha_ * rawAltitudeM + (1.0f - alpha_) * filteredAltitudeM_;
  return filteredAltitudeM_;
}

float VarioMath::computeVario(float filteredAltitudeM, uint32_t nowUs) {
  if (!initialized_) {
    reset(filteredAltitudeM, nowUs);
    return 0.0f;
  }

  const uint32_t dtUs = nowUs - prevTimeUs_;
  if (dtUs < 5000) {  // protect from unrealistically short intervals
    return 0.0f;
  }

  const float dt = dtUs * 1e-6f;
  const float vario = (filteredAltitudeM - prevAltitudeM_) / dt;

  prevAltitudeM_ = filteredAltitudeM;
  prevTimeUs_ = nowUs;
  return vario;
}
