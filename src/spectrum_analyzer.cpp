#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdint.h>
#include "esp_timer.h"
#include "spectrum_analyzer.h"

namespace {
constexpr uint8_t DAC_PIN = 25;       // Classic ESP32 DAC1
constexpr uint8_t ADC_PIN = 34;       // ADC1_CH6, input-only
constexpr uint16_t FFT_SIZE = 1024;
constexpr uint16_t HALF_FFT = FFT_SIZE / 2;
constexpr uint8_t TONE_BIN = 32;      // 32 cycles in 1024 samples
constexpr uint8_t DAC_CENTER = 128;
constexpr uint8_t DAC_AMPLITUDE = 76;
constexpr uint16_t ADC_FULL_SCALE = 4095;
constexpr uint8_t DAC_SETTLE_US = 12;
constexpr float PI_F = 3.14159265358979323846f;

TFT_eSPI tft;
float realPart[FFT_SIZE];
float imagPart[FFT_SIZE];
float magnitude[HALF_FFT + 1];
uint16_t rawSamples[FFT_SIZE];
uint32_t sampleTimesUs[FFT_SIZE];

uint32_t requestedSampleRate = 4000;
bool resultAvailable = false;
float measuredSampleRate = 0.0f;
float measuredToneHz = 0.0f;
float frequencyResolution = 0.0f;
float peakMagnitude = 0.0f;
float thdPercent = 0.0f;
float noiseFloorDb = -120.0f;
uint16_t peakBin = 0;
uint32_t minSampleIntervalUs = UINT32_MAX;
uint32_t maxSampleIntervalUs = 0;

void printHelp() {
  Serial.println();
  Serial.println("MixedSignalLab Spectrum Analyser commands:");
  Serial.println("  r = capture, FFT, draw spectrum and print CSV");
  Serial.println("  1 = set requested sample rate to 1000 samples/s");
  Serial.println("  2 = set requested sample rate to 2000 samples/s");
  Serial.println("  4 = set requested sample rate to 4000 samples/s");
  Serial.println("  p = redraw last spectrum");
  Serial.println("  h = help");
  Serial.println();
}

void drawWaitingScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 36, TFT_NAVY);
  tft.drawFastHLine(0, 36, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MixedSignalLab", 9, 7, 4);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("SPECTRUM ANALYSER", 10, 50, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("DAC1 GPIO25 -> ADC1 GPIO34", 10, 83, 2);
  tft.drawString("FFT size: 1024 points", 10, 115, 2);
  tft.drawString(String("Requested Fs: ") + String(requestedSampleRate) + " S/s", 10, 147, 2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Serial: send r to capture", 10, 185, 2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("1/2/4 = rate   h = help", 10, 213, 2);
}

void waitUntilUs(uint64_t targetUs) {
  // Short, bounded busy-wait keeps the requested software sampling cadence
  // more consistent than delay() for this small 1024-sample capture.
  while (static_cast<uint64_t>(esp_timer_get_time()) < targetUs) {
    // Intentionally empty.
  }
}

void captureSamples() {
  const uint32_t periodUs = 1000000UL / requestedSampleRate;
  const uint64_t startUs = static_cast<uint64_t>(esp_timer_get_time());
  minSampleIntervalUs = UINT32_MAX;
  maxSampleIntervalUs = 0;

  Serial.println();
  Serial.println("SPECTRUM_CAPTURE_BEGIN");
  Serial.printf("Requested sample rate: %lu samples/s\n", static_cast<unsigned long>(requestedSampleRate));
  Serial.printf("Samples: %u, DAC test tone: FFT bin %u\n", FFT_SIZE, TONE_BIN);
  Serial.println("Acquiring 1024 samples using software-paced ADC1 reads...");

  uint32_t previousTime = 0;
  for (uint16_t i = 0; i < FFT_SIZE; ++i) {
    const uint64_t deadline = startUs + (static_cast<uint64_t>(i) * periodUs);
    waitUntilUs(deadline);

    const float phase = (2.0f * PI_F * static_cast<float>(TONE_BIN) * static_cast<float>(i)) /
                        static_cast<float>(FFT_SIZE);
    int16_t dacCode = static_cast<int16_t>(lroundf(static_cast<float>(DAC_CENTER) +
                                                   static_cast<float>(DAC_AMPLITUDE) * sinf(phase)));
    if (dacCode < 0) dacCode = 0;
    if (dacCode > 255) dacCode = 255;
    dacWrite(DAC_PIN, static_cast<uint8_t>(dacCode));

    // Allow the DAC output to settle before taking the ADC sample.
    delayMicroseconds(DAC_SETTLE_US);
    rawSamples[i] = static_cast<uint16_t>(analogRead(ADC_PIN));
    sampleTimesUs[i] = static_cast<uint32_t>(esp_timer_get_time());

    if (i > 0) {
      const uint32_t interval = sampleTimesUs[i] - previousTime;
      if (interval < minSampleIntervalUs) minSampleIntervalUs = interval;
      if (interval > maxSampleIntervalUs) maxSampleIntervalUs = interval;
    }
    previousTime = sampleTimesUs[i];
  }

  dacWrite(DAC_PIN, DAC_CENTER);

  const uint32_t elapsedUs = sampleTimesUs[FFT_SIZE - 1] - sampleTimesUs[0];
  if (elapsedUs > 0) {
    measuredSampleRate = (static_cast<float>(FFT_SIZE - 1) * 1000000.0f) /
                         static_cast<float>(elapsedUs);
  } else {
    measuredSampleRate = static_cast<float>(requestedSampleRate);
  }
  frequencyResolution = measuredSampleRate / static_cast<float>(FFT_SIZE);
  measuredToneHz = measuredSampleRate * static_cast<float>(TONE_BIN) / static_cast<float>(FFT_SIZE);

  Serial.printf("Measured average sample rate: %.2f samples/s\n", measuredSampleRate);
  Serial.printf("Sample interval min/max: %lu / %lu us\n",
                static_cast<unsigned long>(minSampleIntervalUs),
                static_cast<unsigned long>(maxSampleIntervalUs));
  Serial.printf("Measured DAC tone estimate: %.2f Hz\n", measuredToneHz);
  Serial.println("SPECTRUM_CAPTURE_END");
}

void fftInPlace() {
  // Remove DC offset before applying the Hann window.
  double sum = 0.0;
  for (uint16_t i = 0; i < FFT_SIZE; ++i) sum += rawSamples[i];
  const float mean = static_cast<float>(sum / FFT_SIZE);

  float windowSum = 0.0f;
  for (uint16_t i = 0; i < FFT_SIZE; ++i) {
    const float window = 0.5f - 0.5f * cosf((2.0f * PI_F * static_cast<float>(i)) /
                                             static_cast<float>(FFT_SIZE - 1));
    windowSum += window;
    realPart[i] = (static_cast<float>(rawSamples[i]) - mean) * window;
    imagPart[i] = 0.0f;
  }

  // Iterative radix-2 Cooley-Tukey FFT.
  uint16_t j = 0;
  for (uint16_t i = 1; i < FFT_SIZE; ++i) {
    uint16_t bit = FFT_SIZE >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      const float tempReal = realPart[i];
      realPart[i] = realPart[j];
      realPart[j] = tempReal;
      const float tempImag = imagPart[i];
      imagPart[i] = imagPart[j];
      imagPart[j] = tempImag;
    }
  }

  for (uint16_t length = 2; length <= FFT_SIZE; length <<= 1) {
    const float angle = -2.0f * PI_F / static_cast<float>(length);
    const float wLenReal = cosf(angle);
    const float wLenImag = sinf(angle);
    for (uint16_t base = 0; base < FFT_SIZE; base += length) {
      float wReal = 1.0f;
      float wImag = 0.0f;
      const uint16_t half = length >> 1;
      for (uint16_t k = 0; k < half; ++k) {
        const uint16_t even = base + k;
        const uint16_t odd = even + half;
        const float oddReal = realPart[odd] * wReal - imagPart[odd] * wImag;
        const float oddImag = realPart[odd] * wImag + imagPart[odd] * wReal;
        realPart[odd] = realPart[even] - oddReal;
        imagPart[odd] = imagPart[even] - oddImag;
        realPart[even] += oddReal;
        imagPart[even] += oddImag;
        const float nextWReal = wReal * wLenReal - wImag * wLenImag;
        wImag = wReal * wLenImag + wImag * wLenReal;
        wReal = nextWReal;
      }
    }
  }

  // Single-sided amplitude spectrum. Hann coherent-gain correction uses sum(window).
  for (uint16_t bin = 0; bin <= HALF_FFT; ++bin) {
    const float mag = sqrtf(realPart[bin] * realPart[bin] + imagPart[bin] * imagPart[bin]);
    const float scale = (bin == 0 || bin == HALF_FFT) ? 1.0f : 2.0f;
    magnitude[bin] = (windowSum > 0.0f) ? (scale * mag / windowSum) : 0.0f;
  }
}

float dbRelativeToPeak(float value) {
  if (value <= 0.000001f || peakMagnitude <= 0.000001f) return -120.0f;
  return 20.0f * log10f(value / peakMagnitude);
}

float maxAroundBin(uint16_t centerBin) {
  float value = 0.0f;
  const uint16_t first = (centerBin > 1) ? centerBin - 1 : 1;
  const uint16_t last = (centerBin + 1 < HALF_FFT) ? centerBin + 1 : HALF_FFT - 1;
  for (uint16_t b = first; b <= last; ++b) {
    if (magnitude[b] > value) value = magnitude[b];
  }
  return value;
}

void calculateSpectrumMetrics() {
  peakBin = 1;
  peakMagnitude = magnitude[1];
  for (uint16_t bin = 2; bin < HALF_FFT; ++bin) {
    if (magnitude[bin] > peakMagnitude) {
      peakMagnitude = magnitude[bin];
      peakBin = bin;
    }
  }

  const float fundamental = maxAroundBin(peakBin);
  double harmonicSquares = 0.0;
  uint16_t harmonicCount = 0;
  for (uint16_t harmonic = 2; harmonic <= 15; ++harmonic) {
    const uint32_t harmonicBin = static_cast<uint32_t>(peakBin) * harmonic;
    if (harmonicBin >= HALF_FFT) break;
    const float amplitude = maxAroundBin(static_cast<uint16_t>(harmonicBin));
    harmonicSquares += static_cast<double>(amplitude) * amplitude;
    ++harmonicCount;
  }
  thdPercent = (fundamental > 0.000001f && harmonicCount > 0)
                   ? (100.0f * sqrtf(static_cast<float>(harmonicSquares)) / fundamental)
                   : 0.0f;

  // Coarse average noise floor excluding +/-2 bins around the first 15 harmonics.
  double noiseSum = 0.0;
  uint16_t noiseBins = 0;
  for (uint16_t bin = 2; bin < HALF_FFT; ++bin) {
    bool isHarmonicRegion = false;
    for (uint16_t harmonic = 1; harmonic <= 15; ++harmonic) {
      const uint32_t center = static_cast<uint32_t>(peakBin) * harmonic;
      if (center >= HALF_FFT) break;
      const uint16_t low = (center > 2) ? static_cast<uint16_t>(center - 2) : 1;
      const uint16_t high = static_cast<uint16_t>((center + 2 < HALF_FFT) ? center + 2 : HALF_FFT - 1);
      if (bin >= low && bin <= high) {
        isHarmonicRegion = true;
        break;
      }
    }
    if (!isHarmonicRegion) {
      noiseSum += magnitude[bin];
      ++noiseBins;
    }
  }
  const float noiseMean = (noiseBins > 0) ? static_cast<float>(noiseSum / noiseBins) : 0.0f;
  noiseFloorDb = (noiseMean > 0.000001f && peakMagnitude > 0.000001f)
                     ? 20.0f * log10f(noiseMean / peakMagnitude)
                     : -120.0f;
}

void drawSpectrumScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 27, TFT_NAVY);
  tft.drawFastHLine(0, 27, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MIXEDSIGNALLAB - FFT", 7, 6, 2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Fs " + String(measuredSampleRate, 0) + " S/s", 7, 32, 1);
  tft.drawString("Bin " + String(peakBin) + "  F " + String(peakBin * frequencyResolution, 1) + " Hz", 112, 32, 1);
  tft.drawString("THD " + String(thdPercent, 2) + "%", 7, 44, 1);
  tft.drawString("Noise " + String(noiseFloorDb, 1) + " dB", 112, 44, 1);

  constexpr int16_t left = 25;
  constexpr int16_t right = 312;
  constexpr int16_t top = 68;
  constexpr int16_t bottom = 207;
  constexpr int16_t width = right - left;
  constexpr int16_t height = bottom - top;

  tft.drawRect(left, top, width, height, TFT_DARKGREY);
  for (int db = 0; db >= -60; db -= 20) {
    const int16_t y = top + static_cast<int16_t>((-db / 60.0f) * height);
    tft.drawFastHLine(left + 1, y, width - 2, TFT_DARKGREY);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawRightString(String(db), left - 3, y - 4, 1);
  }

  // Compress 0..Nyquist into display columns, keeping the maximum bin per column.
  for (int16_t x = 0; x < width - 2; ++x) {
    uint16_t firstBin = static_cast<uint32_t>(x) * HALF_FFT / (width - 2);
    uint16_t lastBin = static_cast<uint32_t>(x + 1) * HALF_FFT / (width - 2);
    if (lastBin <= firstBin) lastBin = firstBin + 1;
    if (lastBin > HALF_FFT) lastBin = HALF_FFT;
    float columnDb = -120.0f;
    for (uint16_t bin = firstBin; bin <= lastBin; ++bin) {
      const float db = dbRelativeToPeak(magnitude[bin]);
      if (db > columnDb) columnDb = db;
    }
    if (columnDb < -60.0f) columnDb = -60.0f;
    if (columnDb > 0.0f) columnDb = 0.0f;
    const int16_t barHeight = static_cast<int16_t>(((columnDb + 60.0f) / 60.0f) * height);
    const int16_t y = bottom - barHeight;
    const uint16_t color = (x > 0 && x < width - 3) ? TFT_GREEN : TFT_CYAN;
    tft.drawFastVLine(left + 1 + x, y, barHeight, color);
  }

  // Frequency-axis labels for the 0..Nyquist display.
  const float nyquist = measuredSampleRate * 0.5f;
  for (uint8_t label = 0; label <= 4; ++label) {
    const int16_t x = left + static_cast<int32_t>(label) * width / 4;
    tft.drawFastVLine(x, bottom, 3, TFT_LIGHTGREY);
    const float hz = nyquist * label / 4.0f;
    String text = (hz >= 1000.0f) ? String(hz / 1000.0f, 1) + "k" : String(hz, 0);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawCentreString(text, x, 212, 1);
  }
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Hz", 4, 225, 1);
}

void printSpectrumCsv() {
  Serial.println();
  Serial.println("SPECTRUM_CSV_BEGIN");
  Serial.printf("# requested_fs_hz=%lu, measured_fs_hz=%.3f, fft_size=%u, peak_bin=%u, peak_hz=%.3f, thd_percent=%.4f, noise_floor_db=%.3f\n",
                static_cast<unsigned long>(requestedSampleRate), measuredSampleRate,
                FFT_SIZE, peakBin, peakBin * frequencyResolution, thdPercent, noiseFloorDb);
  Serial.println("bin,frequency_hz,magnitude_adc_counts,relative_db");
  for (uint16_t bin = 0; bin <= HALF_FFT; ++bin) {
    Serial.printf("%u,%.3f,%.5f,%.3f\n", bin,
                  bin * frequencyResolution,
                  magnitude[bin],
                  dbRelativeToPeak(magnitude[bin]));
  }
  Serial.println("SPECTRUM_CSV_END");
}

void runCapture() {
  captureSamples();
  fftInPlace();
  calculateSpectrumMetrics();
  resultAvailable = true;
  drawSpectrumScreen();
  Serial.printf("FFT peak: bin %u, %.2f Hz, magnitude %.2f ADC counts\n",
                peakBin, peakBin * frequencyResolution, peakMagnitude);
  Serial.printf("THD (harmonics 2..15, where below Nyquist): %.3f %%\n", thdPercent);
  Serial.printf("Mean spectral noise estimate: %.2f dB relative to peak\n", noiseFloorDb);
  printSpectrumCsv();
  Serial.println("Send r to capture again, 1/2/4 to change requested rate, p to redraw, h for help.");
}

} // namespace

void runSpectrumAnalyzerSetup() {
  Serial.begin(115200);
  delay(250);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  dacWrite(DAC_PIN, DAC_CENTER);

  tft.init();
  tft.setRotation(1);
  drawWaitingScreen();

  Serial.println();
  Serial.println("====================================================");
  Serial.println("MixedSignalLab - 1024-point Spectrum Analyser");
  Serial.println("Target: classic ESP32-WROOM-32 / ILI9341 TFT");
  Serial.println("DAC1 GPIO25 -> ADC1_CH6 GPIO34 loopback required");
  Serial.println("ADC: 12-bit raw, 11 dB attenuation");
  Serial.println("FFT: 1024 points, Hann window, radix-2 FFT");
  Serial.println("Default requested sample rate: 4000 samples/s");
  Serial.println("Commands: r=run, 1/2/4=sample rate, p=plot, h=help");
  Serial.println("====================================================");
  printHelp();
}

void runSpectrumAnalyzerLoop() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == '\r' || command == '\n' || command == ' ') continue;

    switch (command) {
      case 'r':
      case 'R':
        runCapture();
        break;
      case '1':
        requestedSampleRate = 1000;
        Serial.println("Requested sample rate set to 1000 samples/s.");
        if (!resultAvailable) drawWaitingScreen();
        break;
      case '2':
        requestedSampleRate = 2000;
        Serial.println("Requested sample rate set to 2000 samples/s.");
        if (!resultAvailable) drawWaitingScreen();
        break;
      case '4':
        requestedSampleRate = 4000;
        Serial.println("Requested sample rate set to 4000 samples/s.");
        if (!resultAvailable) drawWaitingScreen();
        break;
      case 'p':
      case 'P':
        if (resultAvailable) drawSpectrumScreen();
        else drawWaitingScreen();
        break;
      case 'h':
      case 'H':
        printHelp();
        break;
      default:
        Serial.printf("Unknown command '%c'. Send h for help.\n", command);
        break;
    }
  }
  delay(2);
}
