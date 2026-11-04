#include <Arduino.h>
#include <math.h>
#include "loopback_test.h"

namespace {
constexpr uint8_t DAC_PIN = 25;       // Classic ESP32 DAC1
constexpr uint8_t ADC_PIN = 34;       // ADC1_CH6, input-only
constexpr uint8_t READS_PER_STEP = 32;

bool sweepRequested = false;
uint32_t lastReportMs = 0;

struct SampleStats {
  float mean;
  float sigma;
  uint16_t minimum;
  uint16_t maximum;
};

SampleStats readAveraged(uint8_t count) {
  uint32_t sum = 0;
  uint32_t sumSquares = 0;
  uint16_t minValue = 4095;
  uint16_t maxValue = 0;

  // One discarded conversion helps after changing the DAC output.
  (void)analogRead(ADC_PIN);

  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t value = static_cast<uint16_t>(analogRead(ADC_PIN));
    sum += value;
    sumSquares += static_cast<uint32_t>(value) * value;
    if (value < minValue) minValue = value;
    if (value > maxValue) maxValue = value;
    delayMicroseconds(250);
  }

  const float mean = static_cast<float>(sum) / count;
  const float meanSquare = static_cast<float>(sumSquares) / count;
  float variance = meanSquare - mean * mean;
  if (variance < 0.0f) variance = 0.0f;

  return {mean, sqrtf(variance), minValue, maxValue};
}

void runSweep() {
  Serial.println();
  Serial.println("Starting DAC -> ADC loopback sweep...");
  Serial.println("Keep GPIO25 connected directly to GPIO34.");
  Serial.println("dac_code,adc_mean,adc_sigma,adc_min,adc_max");

  for (uint16_t code = 0; code <= 255; ++code) {
    dacWrite(DAC_PIN, static_cast<uint8_t>(code));
    delay(2); // Allow the DAC output to settle before sampling.
    const SampleStats stats = readAveraged(READS_PER_STEP);

    Serial.printf("%u,%.2f,%.3f,%u,%u\n",
                  code,
                  stats.mean,
                  stats.sigma,
                  stats.minimum,
                  stats.maximum);
  }

  dacWrite(DAC_PIN, 0);
  Serial.println("Sweep complete. Send 'r' to run it again.");
  Serial.println("Note: ADC readings include both DAC and ADC errors.");
}
}

void runLoopbackSetup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  dacWrite(DAC_PIN, 0);

  Serial.println();
  Serial.println("====================================");
  Serial.println("MixedSignalLab - DAC/ADC Loopback");
  Serial.println("ESP32 classic WROOM-32");
  Serial.println("DAC1 GPIO25 -> ADC1_CH6 GPIO34");
  Serial.println("ADC: 12-bit, 11 dB attenuation");
  Serial.printf("Averaging: %u reads per DAC code\n", READS_PER_STEP);
  Serial.println("====================================");
  Serial.println("Commands: r = run sweep, h = help");
  Serial.println("Send r after opening Serial Monitor to start the sweep.");

  sweepRequested = false;
}

void runLoopbackLoop() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r' || command == 'R') {
      sweepRequested = true;
    } else if (command == 'h' || command == 'H') {
      Serial.println("Commands: r = run 0..255 DAC sweep, h = help");
    }
  }

  if (sweepRequested) {
    sweepRequested = false;
    runSweep();
    lastReportMs = millis();
  }

  if (millis() - lastReportMs > 1000) {
    lastReportMs = millis();
    // Idle at zero volts between sweeps; do not generate an unintended signal.
    dacWrite(DAC_PIN, 0);
  }
  delay(10);
}
