#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdint.h>
#include "adc_characterization.h"

namespace {
constexpr uint8_t DAC_PIN = 25;        // Classic ESP32 DAC1
constexpr uint8_t ADC_PIN = 34;        // ADC1_CH6, input-only
constexpr uint8_t READS_PER_STEP = 64;
constexpr uint16_t DAC_CODES = 256;
constexpr uint16_t ADC_MAX_CODE = 4095;

TFT_eSPI tft;
bool sweepRequested = false;
bool resultsAvailable = false;

struct SampleStats {
  float mean;
  float sigma;
  uint16_t minimum;
  uint16_t maximum;
};

SampleStats rawStats[DAC_CODES];
float inlLsb[DAC_CODES];
float dnlLsb[DAC_CODES];
uint8_t correctionLut[DAC_CODES];
float beforeErrorLsb[DAC_CODES];
float afterErrorLsb[DAC_CODES];

float idealStep = 0.0f;
float meanSigma = 0.0f;
float maxSigma = 0.0f;
float maxAbsInl = 0.0f;
float rmsInl = 0.0f;
float maxAbsDnl = 0.0f;
float rmsBefore = 0.0f;
float rmsAfter = 0.0f;
bool monotonic = true;

SampleStats readAveraged(uint8_t count) {
  uint64_t sum = 0;
  uint64_t sumSquares = 0;
  uint16_t minValue = ADC_MAX_CODE;
  uint16_t maxValue = 0;

  // Discard a few conversions immediately after changing the DAC output.
  for (uint8_t i = 0; i < 4; ++i) {
    (void)analogRead(ADC_PIN);
  }

  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t value = static_cast<uint16_t>(analogRead(ADC_PIN));
    sum += value;
    sumSquares += static_cast<uint64_t>(value) * value;
    if (value < minValue) minValue = value;
    if (value > maxValue) maxValue = value;
    delayMicroseconds(180);
  }

  const double mean = static_cast<double>(sum) / count;
  const double meanSquare = static_cast<double>(sumSquares) / count;
  double variance = meanSquare - mean * mean;
  if (variance < 0.0) variance = 0.0;

  SampleStats result;
  result.mean = static_cast<float>(mean);
  result.sigma = static_cast<float>(sqrt(variance));
  result.minimum = minValue;
  result.maximum = maxValue;
  return result;
}

void drawWaitingScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 38, TFT_NAVY);
  tft.drawFastHLine(0, 38, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MixedSignalLab", 10, 8, 4);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("ADC CHARACTERISATION", 10, 53, 2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("DAC1 GPIO25 -> ADC1 GPIO34", 10, 83, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Sweep: 256 DAC codes", 10, 119, 2);
  tft.drawString("Averages: 64 reads / code", 10, 145, 2);
  tft.drawString("Serial: send r to run", 10, 177, 2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("h = help   p = show plot", 10, 205, 2);
}

void drawProgress(uint16_t code) {
  constexpr int16_t x = 10;
  constexpr int16_t y = 214;
  constexpr int16_t width = 300;
  constexpr int16_t height = 12;
  const int16_t filled = static_cast<int16_t>((static_cast<uint32_t>(code + 1) * width) / DAC_CODES);
  tft.fillRect(x, y, width, height, TFT_DARKGREY);
  if (filled > 0) tft.fillRect(x, y, filled, height, TFT_CYAN);
  tft.drawRect(x, y, width, height, TFT_WHITE);
}

uint8_t findDacCodeForTarget(float target) {
  // A linear search is deliberate: it also tolerates small non-monotonic
  // fluctuations in the measured loopback curve.
  uint8_t bestCode = 0;
  float bestError = fabsf(rawStats[0].mean - target);
  for (uint16_t code = 1; code < DAC_CODES; ++code) {
    const float error = fabsf(rawStats[code].mean - target);
    if (error < bestError) {
      bestError = error;
      bestCode = static_cast<uint8_t>(code);
    }
  }
  return bestCode;
}

void calculateMetricsAndLut() {
  const float low = rawStats[0].mean;
  const float high = rawStats[255].mean;
  idealStep = (high - low) / 255.0f;
  if (idealStep < 0.001f) idealStep = 0.001f;

  double inlSquares = 0.0;
  double beforeSquares = 0.0;
  double afterSquares = 0.0;
  double sigmaSum = 0.0;
  maxSigma = 0.0f;
  maxAbsInl = 0.0f;
  maxAbsDnl = 0.0f;
  monotonic = true;

  for (uint16_t code = 0; code < DAC_CODES; ++code) {
    const float ideal = low + static_cast<float>(code) * idealStep;
    beforeErrorLsb[code] = (rawStats[code].mean - ideal) / idealStep;
    inlLsb[code] = beforeErrorLsb[code];
    inlSquares += static_cast<double>(inlLsb[code]) * inlLsb[code];
    beforeSquares += static_cast<double>(beforeErrorLsb[code]) * beforeErrorLsb[code];

    sigmaSum += rawStats[code].sigma;
    if (rawStats[code].sigma > maxSigma) maxSigma = rawStats[code].sigma;

    if (code == 0) {
      dnlLsb[code] = 0.0f; // No preceding transition for code zero.
    } else {
      const float step = rawStats[code].mean - rawStats[code - 1].mean;
      dnlLsb[code] = (step / idealStep) - 1.0f;
      if (step < 0.0f) monotonic = false;
    }
    if (fabsf(inlLsb[code]) > maxAbsInl) maxAbsInl = fabsf(inlLsb[code]);
    if (fabsf(dnlLsb[code]) > maxAbsDnl) maxAbsDnl = fabsf(dnlLsb[code]);
  }

  // Construct a 256-entry inverse LUT. Index = desired uniformly spaced
  // loopback output level; value = DAC code that best approximates it.
  // This corrects the combined DAC->wire->ADC transfer only.
  for (uint16_t targetIndex = 0; targetIndex < DAC_CODES; ++targetIndex) {
    const float target = low + static_cast<float>(targetIndex) * idealStep;
    correctionLut[targetIndex] = findDacCodeForTarget(target);
    const float correctedMean = rawStats[correctionLut[targetIndex]].mean;
    afterErrorLsb[targetIndex] = (correctedMean - target) / idealStep;
    afterSquares += static_cast<double>(afterErrorLsb[targetIndex]) * afterErrorLsb[targetIndex];
  }

  meanSigma = static_cast<float>(sigmaSum / DAC_CODES);
  rmsInl = sqrtf(static_cast<float>(inlSquares / DAC_CODES));
  rmsBefore = sqrtf(static_cast<float>(beforeSquares / DAC_CODES));
  rmsAfter = sqrtf(static_cast<float>(afterSquares / DAC_CODES));
}

void printCsv() {
  Serial.println();
  Serial.println("ADC_CHAR_CSV_BEGIN");
  Serial.println("dac_code,adc_mean,adc_sigma,adc_min,adc_max,loopback_inl_lsb,loopback_dnl_lsb,correction_dac_code,corrected_adc_mean,corrected_error_lsb");

  for (uint16_t code = 0; code < DAC_CODES; ++code) {
    const uint8_t mappedCode = correctionLut[code];
    const float correctedMean = rawStats[mappedCode].mean;
    Serial.printf("%u,%.2f,%.3f,%u,%u,%.4f,%.4f,%u,%.2f,%.4f\n",
                  code,
                  rawStats[code].mean,
                  rawStats[code].sigma,
                  rawStats[code].minimum,
                  rawStats[code].maximum,
                  inlLsb[code],
                  dnlLsb[code],
                  mappedCode,
                  correctedMean,
                  afterErrorLsb[code]);
  }
  Serial.println("ADC_CHAR_CSV_END");
}

void drawErrorPlot() {
  constexpr int16_t plotLeft = 22;
  constexpr int16_t plotRight = 309;
  constexpr int16_t plotTop = 125;
  constexpr int16_t plotBottom = 226;
  constexpr int16_t plotWidth = plotRight - plotLeft;
  constexpr int16_t plotHeight = plotBottom - plotTop;

  float maxError = 0.5f;
  for (uint16_t i = 0; i < DAC_CODES; ++i) {
    maxError = fmaxf(maxError, fabsf(beforeErrorLsb[i]));
    maxError = fmaxf(maxError, fabsf(afterErrorLsb[i]));
  }
  maxError = ceilf(maxError * 2.0f) / 2.0f;

  tft.fillRect(plotLeft, plotTop, plotWidth + 1, plotHeight + 1, TFT_BLACK);
  tft.drawRect(plotLeft, plotTop, plotWidth + 1, plotHeight + 1, TFT_DARKGREY);
  const int16_t zeroY = plotTop + plotHeight / 2;
  tft.drawFastHLine(plotLeft, zeroY, plotWidth, TFT_LIGHTGREY);
  tft.drawFastVLine(plotLeft, plotTop, plotHeight, TFT_DARKGREY);

  int16_t prevRawX = plotLeft;
  int16_t prevRawY = zeroY;
  int16_t prevCorrectedX = plotLeft;
  int16_t prevCorrectedY = zeroY;

  for (uint16_t i = 0; i < DAC_CODES; ++i) {
    const int16_t x = plotLeft + static_cast<int32_t>(i) * plotWidth / 255;
    const float rawNorm = beforeErrorLsb[i] / maxError;
    const float correctedNorm = afterErrorLsb[i] / maxError;
    const int16_t rawY = zeroY - static_cast<int16_t>(rawNorm * (plotHeight / 2 - 2));
    const int16_t correctedY = zeroY - static_cast<int16_t>(correctedNorm * (plotHeight / 2 - 2));

    if (i > 0) {
      tft.drawLine(prevRawX, prevRawY, x, rawY, TFT_YELLOW);
      tft.drawLine(prevCorrectedX, prevCorrectedY, x, correctedY, TFT_GREEN);
    }
    prevRawX = x;
    prevRawY = rawY;
    prevCorrectedX = x;
    prevCorrectedY = correctedY;
  }
}

void drawResultsScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 32, TFT_NAVY);
  tft.drawFastHLine(0, 32, tft.width(), TFT_CYAN);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("ADC LOOPBACK RESULTS", 7, 8, 2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Noise sigma avg/max:", 8, 40, 1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(meanSigma, 2) + " / " + String(maxSigma, 2) + " ADC counts", 145, 40, 1);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Loopback INL max/RMS:", 8, 54, 1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(maxAbsInl, 2) + " / " + String(rmsInl, 2) + " LSB", 145, 54, 1);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Loopback DNL max abs:", 8, 68, 1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(maxAbsDnl, 2) + " LSB", 145, 68, 1);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("RMS error before/after:", 8, 82, 1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(rmsBefore, 3) + " / " + String(rmsAfter, 3) + " LSB", 145, 82, 1);

  tft.setTextColor(monotonic ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(monotonic ? "Measured curve: monotonic" : "WARNING: non-monotonic samples", 8, 97, 1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Error plot (yellow=raw, green=LUT corrected)", 8, 110, 1);
  drawErrorPlot();
}

void runSweep() {
  Serial.println();
  Serial.println("MixedSignalLab ADC Characterisation starting");
  Serial.printf("DAC GPIO%u -> ADC GPIO%u | %u reads/code\n", DAC_PIN, ADC_PIN, READS_PER_STEP);
  Serial.println("Keep GPIO25 connected directly to GPIO34.");
  Serial.println("Measuring combined DAC + wiring + ADC transfer; not intrinsic ADC specs.");

  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 36, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("ADC CHARACTERISATION", 10, 9, 2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Running 256-code DAC sweep...", 10, 65, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Please keep the loopback wire connected.", 10, 95, 1);

  for (uint16_t code = 0; code < DAC_CODES; ++code) {
    dacWrite(DAC_PIN, static_cast<uint8_t>(code));
    delay(3);
    rawStats[code] = readAveraged(READS_PER_STEP);
    if ((code & 7U) == 0U || code == 255U) {
      drawProgress(code);
    }
  }

  // Return the physical DAC output to zero after acquisition.
  dacWrite(DAC_PIN, 0);
  calculateMetricsAndLut();
  resultsAvailable = true;
  drawResultsScreen();

  Serial.println();
  Serial.println("ADC CHARACTERISATION SUMMARY");
  Serial.printf("Average noise sigma: %.3f ADC counts\n", meanSigma);
  Serial.printf("Maximum noise sigma: %.3f ADC counts\n", maxSigma);
  Serial.printf("Endpoint ideal step: %.4f ADC counts / DAC code\n", idealStep);
  Serial.printf("Maximum absolute loopback INL: %.4f LSB\n", maxAbsInl);
  Serial.printf("RMS loopback INL: %.4f LSB\n", rmsInl);
  Serial.printf("Maximum absolute loopback DNL: %.4f LSB\n", maxAbsDnl);
  Serial.printf("RMS transfer error before LUT: %.4f LSB\n", rmsBefore);
  Serial.printf("RMS transfer error after LUT: %.4f LSB\n", rmsAfter);
  Serial.printf("Measured curve monotonic: %s\n", monotonic ? "YES" : "NO");
  Serial.println("Correction LUT is stored in RAM only and is lost after reset.");
  Serial.println("CSV columns include combined loopback metrics, not intrinsic ADC specifications.");

  printCsv();
  Serial.println("Sweep complete. Commands: r=repeat, p=redraw plot, h=help");
}

void printHelp() {
  Serial.println();
  Serial.println("MixedSignalLab ADC Characterisation commands:");
  Serial.println("  r : run the 256-code DAC->ADC sweep and print CSV");
  Serial.println("  p : redraw the latest results plot on the TFT");
  Serial.println("  h : print this help");
}
} // namespace

void runAdcCharacterizationSetup() {
  Serial.begin(115200);
  delay(350);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  dacWrite(DAC_PIN, 0);

  tft.init();
  tft.setRotation(1); // Landscape, 320 x 240.
  drawWaitingScreen();

  Serial.println();
  Serial.println("===============================================");
  Serial.println("MixedSignalLab - ADC Characterisation");
  Serial.println("Board: classic ESP32-WROOM-32");
  Serial.println("DAC1 GPIO25 -> ADC1_CH6 GPIO34");
  Serial.println("ADC resolution: 12-bit raw, 11 dB attenuation");
  Serial.printf("Averaging: %u reads per DAC code\n", READS_PER_STEP);
  Serial.println("Metrics: loopback INL/DNL, noise sigma, LUT residual error");
  Serial.println("Commands: r=run, p=plot, h=help");
  Serial.println("===============================================");
  Serial.println("NOTE: Results characterize the combined DAC-ADC loopback path.");
}

void runAdcCharacterizationLoop() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r' || command == 'R') {
      sweepRequested = true;
    } else if (command == 'p' || command == 'P') {
      if (resultsAvailable) drawResultsScreen();
      else Serial.println("No results yet. Send r to run a sweep.");
    } else if (command == 'h' || command == 'H') {
      printHelp();
    }
  }

  if (sweepRequested) {
    sweepRequested = false;
    runSweep();
  }
  delay(10);
}
