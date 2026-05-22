#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MS5611.h>

#include "VarioMath.h"

/*
  DIY Paragliding Variometer + Altimeter (ESP32)

  Wiring (I2C):
    ESP32 GPIO21 (SDA) -> MS5611 SDA, SSD1306 SDA
    ESP32 GPIO22 (SCL) -> MS5611 SCL, SSD1306 SCL
    ESP32 3V3          -> MS5611 VCC, SSD1306 VCC
    ESP32 GND          -> MS5611 GND, SSD1306 GND

  Buzzer:
    ESP32 GPIO25 -> Passive piezo buzzer (+)
    ESP32 GND    -> Passive piezo buzzer (-)
*/

namespace Config {
constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;
constexpr uint8_t PIN_BUZZER = 25;

constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr int8_t OLED_RESET = -1;
constexpr uint8_t OLED_ADDR = 0x3C;

constexpr uint32_t SENSOR_HZ = 80;   // 50-100 Hz target
constexpr uint32_t AUDIO_HZ = 80;    // 50-100 Hz target
constexpr uint32_t DISPLAY_HZ = 8;   // 5-10 Hz target
constexpr uint32_t DEBUG_HZ = 5;

constexpr float EMA_ALPHA = 0.12f;   // altitude smoothing factor

// Vario audio behavior thresholds (m/s)
constexpr float CLIMB_START_MS = 0.1f;
constexpr float SILENCE_LOW_MS = -0.2f;
constexpr float SINK_CONT_MS = -2.0f;

// Audio mapping
constexpr int CLIMB_FREQ_MIN = 900;
constexpr int CLIMB_FREQ_MAX = 2400;
constexpr int SINK_FREQ = 280;
constexpr uint32_t BEEP_ON_MIN_MS = 25;
constexpr uint32_t BEEP_ON_MAX_MS = 80;
constexpr uint32_t BEEP_GAP_MIN_MS = 35;
constexpr uint32_t BEEP_GAP_MAX_MS = 220;

constexpr uint8_t BUZZER_LEDC_CHANNEL = 0;
constexpr uint8_t BUZZER_LEDC_RES_BITS = 8;
constexpr uint32_t BUZZER_LEDC_BASE_FREQ = 1000;
}  // namespace Config

Adafruit_SSD1306 display(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, Config::OLED_RESET);
MS5611 ms5611;
VarioMath varioMath(Config::EMA_ALPHA);

struct FlightState {
  float pressurePa = 101325.0f;
  float temperatureC = 20.0f;
  float altitudeM = 0.0f;
  float varioMs = 0.0f;
  float refPressurePa = 0.0f;
  bool calibrated = false;
} state;

struct AudioState {
  bool toneOn = false;
  bool beepActive = false;
  uint32_t phaseStartMs = 0;
} audio;

uint32_t lastSensorUs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastAudioUs = 0;
uint32_t lastDebugMs = 0;

constexpr uint32_t sensorPeriodUs() { return 1000000UL / Config::SENSOR_HZ; }
constexpr uint32_t audioPeriodUs() { return 1000000UL / Config::AUDIO_HZ; }
constexpr uint32_t displayPeriodMs() { return 1000UL / Config::DISPLAY_HZ; }
constexpr uint32_t debugPeriodMs() { return 1000UL / Config::DEBUG_HZ; }

void setTone(bool enable, int freqHz = 0) {
  if (enable) {
    ledcWriteTone(Config::BUZZER_LEDC_CHANNEL, freqHz);
    ledcWrite(Config::BUZZER_LEDC_CHANNEL, 128);
    audio.toneOn = true;
  } else {
    ledcWrite(Config::BUZZER_LEDC_CHANNEL, 0);
    ledcWrite(Config::BUZZER_LEDC_CHANNEL, 0);
    audio.toneOn = false;
  }
}

void drawSplashScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 22);
  display.println("DIY Paragliding");
  display.setCursor(36, 36);
  display.println("Vario");
  display.display();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("ALT %4.0f m", state.altitudeM);

  display.setCursor(0, 16);
  display.printf("VAR %+4.1f m/s", state.varioMs);

  display.setCursor(0, 32);
  display.printf("TEMP %4.1f C", state.temperatureC);

  display.setCursor(0, 48);
  display.printf("P %7.0f Pa", state.pressurePa);

  display.display();
}

void updateAudio(uint32_t nowMs) {
  const float v = state.varioMs;

  if (v < Config::SINK_CONT_MS) {
    if (!audio.toneOn) {
      setTone(true, Config::SINK_FREQ);
    }
    audio.beepActive = true;
    return;
  }

  if (v >= Config::CLIMB_START_MS) {
    const float climbNorm = constrain((v - Config::CLIMB_START_MS) / 4.0f, 0.0f, 1.0f);

    const int freq = static_cast<int>(Config::CLIMB_FREQ_MIN +
                                      climbNorm * (Config::CLIMB_FREQ_MAX - Config::CLIMB_FREQ_MIN));

    const uint32_t onMs = static_cast<uint32_t>(Config::BEEP_ON_MAX_MS -
                                                climbNorm * (Config::BEEP_ON_MAX_MS - Config::BEEP_ON_MIN_MS));
    const uint32_t offMs = static_cast<uint32_t>(Config::BEEP_GAP_MAX_MS -
                                                 climbNorm * (Config::BEEP_GAP_MAX_MS - Config::BEEP_GAP_MIN_MS));

    if (!audio.beepActive) {
      audio.beepActive = true;
      audio.phaseStartMs = nowMs;
      setTone(true, freq);
      return;
    }

    const uint32_t elapsed = nowMs - audio.phaseStartMs;
    if (audio.toneOn && elapsed >= onMs) {
      setTone(false);
      audio.phaseStartMs = nowMs;
    } else if (!audio.toneOn && elapsed >= offMs) {
      setTone(true, freq);
      audio.phaseStartMs = nowMs;
    }
    return;
  }

  // Between -0.2 and +0.1 m/s: silent (and also gentle sink region).
  if (v >= Config::SILENCE_LOW_MS && v < Config::CLIMB_START_MS) {
    setTone(false);
    audio.beepActive = false;
    return;
  }

  // Mild sink: keep silent for a clean, usable acoustic profile.
  setTone(false);
  audio.beepActive = false;
}

void sampleBaroAndCompute() {
  const uint32_t nowUs = micros();

  if (ms5611.read() != 0) {
    // Non-zero means read failed in common MS5611 libs. Keep previous values.
    return;
  }

  state.pressurePa = ms5611.getPressure() * 100.0f;  // mbar->Pa in many libs
  state.temperatureC = ms5611.getTemperature();

  if (!state.calibrated) {
    state.refPressurePa = state.pressurePa;
    varioMath.reset(0.0f, nowUs);
    state.calibrated = true;
    return;
  }

  const float relativeAltitudeM = pressureToAltitudeRelative(state.pressurePa, state.refPressurePa);
  state.altitudeM = varioMath.updateAltitude(relativeAltitudeM);
  state.varioMs = varioMath.computeVario(state.altitudeM, nowUs);
}

void printDebug() {
  Serial.printf("P=%.0fPa T=%.2fC ALT=%.2fm VAR=%.2fm/s\n",
                state.pressurePa,
                state.temperatureC,
                state.altitudeM,
                state.varioMs);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nDIY Paragliding Vario booting...");

  Wire.begin(Config::PIN_SDA, Config::PIN_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR)) {
    Serial.println("OLED init failed");
  }

  drawSplashScreen();

  if (!ms5611.begin()) {
    Serial.println("MS5611 init failed");
  } else {
    Serial.println("MS5611 initialized");
  }

  ledcSetup(Config::BUZZER_LEDC_CHANNEL, Config::BUZZER_LEDC_BASE_FREQ, Config::BUZZER_LEDC_RES_BITS);
  ledcAttachPin(Config::PIN_BUZZER, Config::BUZZER_LEDC_CHANNEL);
  setTone(false);

  // Startup calibration: capture initial pressure as zero-altitude reference.
  const uint32_t calibStart = millis();
  while (millis() - calibStart < 1200) {
    sampleBaroAndCompute();
  }

  Serial.println("Startup calibration completed.");

  lastSensorUs = micros();
  lastAudioUs = lastSensorUs;
  lastDisplayMs = millis();
  lastDebugMs = lastDisplayMs;
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  if (static_cast<uint32_t>(nowUs - lastSensorUs) >= sensorPeriodUs()) {
    lastSensorUs += sensorPeriodUs();
    sampleBaroAndCompute();
  }

  if (static_cast<uint32_t>(nowUs - lastAudioUs) >= audioPeriodUs()) {
    lastAudioUs += audioPeriodUs();
    updateAudio(nowMs);
  }

  if (nowMs - lastDisplayMs >= displayPeriodMs()) {
    lastDisplayMs += displayPeriodMs();
    updateDisplay();
  }

  if (nowMs - lastDebugMs >= debugPeriodMs()) {
    lastDebugMs += debugPeriodMs();
    printDebug();
  }
}
