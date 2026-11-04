#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "harmonic_comparison.h"

namespace {
constexpr uint16_t SAMPLE_COUNT = 1020;       // divisible by six: exactly 170 samples per 60-degree sector
constexpr uint8_t MAX_HARMONIC = 15;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float MODULATION_INDEX = 0.80f;     // Match the PWM generator's default 80% setting
constexpr float THI_GAIN = 1.15470053838f;
constexpr float DISPLAY_FLOOR_DB = -60.0f;

TFT_eSPI tft;
char commandBuffer[48];
size_t commandLength = 0;

struct SpectrumResult {
  float amplitude[MAX_HARMONIC + 1] = {};
  float dbc[MAX_HARMONIC + 1] = {};
  float thdH15Percent = 0.0f;
  float thdToNyquistPercent = 0.0f;
  float fundamental = 0.0f;
};

SpectrumResult results[3];
float thiInjectedThirdDbc = -120.0f;
bool calculated = false;

const char* modeLabel(uint8_t mode) {
  switch (mode) {
    case 0: return "SINUSOIDAL PWM";
    case 1: return "THIRD-HARMONIC INJECTION";
    default: return "SIX-STEP";
  }
}

float wrapAngle(float angle) {
  angle = fmodf(angle, TWO_PI_F);
  if (angle < 0.0f) angle += TWO_PI_F;
  return angle;
}

float sineReference(float angle) {
  return sinf(angle);
}

float thiReference(float angle) {
  // Conventional THI: preserve the fundamental and scale the composite
  // reference to +/-1 using 2/sqrt(3). The triplen term cancels in line-line.
  return (sinf(angle) + (1.0f / 6.0f) * sinf(3.0f * angle)) * THI_GAIN;
}

void sixStepReferences(float angle, float refs[3]) {
  // Same six 60-degree sectors as the PWM generator.
  const float wrapped = wrapAngle(angle);
  uint8_t sector = static_cast<uint8_t>(wrapped / (PI_F / 3.0f));
  if (sector > 5) sector = 5;
  switch (sector) {
    case 0: refs[0] = 1.0f; refs[1] = 0.0f; refs[2] = 0.5f; break;
    case 1: refs[0] = 1.0f; refs[1] = 0.5f; refs[2] = 0.0f; break;
    case 2: refs[0] = 0.5f; refs[1] = 1.0f; refs[2] = 0.0f; break;
    case 3: refs[0] = 0.0f; refs[1] = 1.0f; refs[2] = 0.5f; break;
    case 4: refs[0] = 0.0f; refs[1] = 0.5f; refs[2] = 1.0f; break;
    default: refs[0] = 0.5f; refs[1] = 0.0f; refs[2] = 1.0f; break;
  }
}

void generateLineLineWaveform(uint8_t mode, float waveform[SAMPLE_COUNT]) {
  const float phaseOffsets[3] = {0.0f, -TWO_PI_F / 3.0f, TWO_PI_F / 3.0f};
  for (uint16_t n = 0; n < SAMPLE_COUNT; ++n) {
    const float theta = TWO_PI_F * static_cast<float>(n) / static_cast<float>(SAMPLE_COUNT);
    float pole[3] = {0.0f, 0.0f, 0.0f};

    if (mode == 2) {
      float refs[3] = {0.0f, 0.0f, 0.0f};
      sixStepReferences(theta, refs);
      for (uint8_t phase = 0; phase < 3; ++phase) {
        // Convert 0/50/100% reference level to a normalized pole voltage.
        pole[phase] = MODULATION_INDEX * (2.0f * (refs[phase] - 0.5f));
      }
    } else {
      for (uint8_t phase = 0; phase < 3; ++phase) {
        const float angle = theta + phaseOffsets[phase];
        const float reference = (mode == 1) ? thiReference(angle) : sineReference(angle);
        pole[phase] = MODULATION_INDEX * reference;
      }
    }

    // Analyze line-to-line reference voltage Vab. The common-mode component,
    // including the injected third harmonic in THI, cancels in this subtraction.
    waveform[n] = pole[0] - pole[1];
  }
}

void calculateDiscreteHarmonics(const float waveform[SAMPLE_COUNT], SpectrumResult& result) {
  double mean = 0.0;
  for (uint16_t n = 0; n < SAMPLE_COUNT; ++n) mean += waveform[n];
  mean /= static_cast<double>(SAMPLE_COUNT);

  double variance = 0.0;
  for (uint16_t n = 0; n < SAMPLE_COUNT; ++n) {
    const double centered = static_cast<double>(waveform[n]) - mean;
    variance += centered * centered;
  }
  variance /= static_cast<double>(SAMPLE_COUNT);

  for (uint8_t harmonic = 1; harmonic <= MAX_HARMONIC; ++harmonic) {
    double cosineSum = 0.0;
    double sineSum = 0.0;
    for (uint16_t n = 0; n < SAMPLE_COUNT; ++n) {
      const double angle = TWO_PI_F * static_cast<double>(harmonic) * n / SAMPLE_COUNT;
      const double sample = static_cast<double>(waveform[n]) - mean;
      cosineSum += sample * cos(angle);
      sineSum += sample * sin(angle);
    }
    const double re = (2.0 / SAMPLE_COUNT) * cosineSum;
    const double im = (2.0 / SAMPLE_COUNT) * sineSum;
    result.amplitude[harmonic] = static_cast<float>(sqrt(re * re + im * im));
  }

  result.fundamental = result.amplitude[1];

  // THD limited to the harmonics shown in the bar chart (H2 through H15).
  double h15Power = 0.0;
  for (uint8_t harmonic = 2; harmonic <= MAX_HARMONIC; ++harmonic) {
    const double amplitude = result.amplitude[harmonic];
    h15Power += amplitude * amplitude;
  }
  result.thdH15Percent = (result.fundamental > 1.0e-9f)
      ? 100.0f * sqrtf(static_cast<float>(h15Power)) / result.fundamental
      : 0.0f;

  // Parseval-based THD across all non-fundamental DFT bins through Nyquist.
  // For a non-Nyquist sinusoid, mean-square power is A_peak^2 / 2.
  const double fundamentalPower = static_cast<double>(result.fundamental) * result.fundamental / 2.0;
  const double otherHarmonicPower = (variance > fundamentalPower) ? (variance - fundamentalPower) : 0.0;
  result.thdToNyquistPercent = (fundamentalPower > 1.0e-18)
      ? 100.0f * sqrtf(static_cast<float>(otherHarmonicPower / fundamentalPower))
      : 0.0f;

  for (uint8_t harmonic = 1; harmonic <= MAX_HARMONIC; ++harmonic) {
    const float ratio = (result.fundamental > 1.0e-9f)
        ? result.amplitude[harmonic] / result.fundamental : 0.0f;
    result.dbc[harmonic] = (ratio > 1.0e-6f) ? 20.0f * log10f(ratio) : -120.0f;
  }
}

void calculateThiInjectedThird() {
  // Report the deliberate third harmonic in one THI phase-leg reference.
  // It is a zero-sequence/common-mode term and therefore cancels from Vab.
  double c1 = 0.0, s1 = 0.0, c3 = 0.0, s3 = 0.0;
  for (uint16_t n = 0; n < SAMPLE_COUNT; ++n) {
    const double angle = TWO_PI_F * n / SAMPLE_COUNT;
    const double value = thiReference(static_cast<float>(angle));
    c1 += value * cos(angle); s1 += value * sin(angle);
    c3 += value * cos(3.0 * angle); s3 += value * sin(3.0 * angle);
  }
  const double a1 = (2.0 / SAMPLE_COUNT) * sqrt(c1 * c1 + s1 * s1);
  const double a3 = (2.0 / SAMPLE_COUNT) * sqrt(c3 * c3 + s3 * s3);
  thiInjectedThirdDbc = (a1 > 1.0e-9 && a3 > 1.0e-9)
      ? static_cast<float>(20.0 * log10(a3 / a1)) : -120.0f;
}

void calculateAll() {
  float waveform[SAMPLE_COUNT];
  for (uint8_t mode = 0; mode < 3; ++mode) {
    generateLineLineWaveform(mode, waveform);
    calculateDiscreteHarmonics(waveform, results[mode]);
  }
  calculateThiInjectedThird();
  calculated = true;
}

void drawWaitingScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 30, TFT_NAVY);
  tft.drawFastHLine(0, 30, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MIXEDSIGNALLAB - HARMONICS", 7, 7, 2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Reference-waveform analysis", 8, 48, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("3-phase Vab, one electrical cycle", 8, 83, 2);
  tft.drawString("1020 samples / cycle; chart H1-15", 8, 113, 2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Computing harmonic comparison...", 8, 166, 2);
}

void drawComparisonScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 27, TFT_NAVY);
  tft.drawFastHLine(0, 27, 320, TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MIXEDSIGNALLAB - HARMONICS", 6, 6, 2);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("SPWM " + String(results[0].thdToNyquistPercent, 2) + "%", 6, 33, 1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("THI " + String(results[1].thdToNyquistPercent, 2) + "%", 105, 33, 1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("SIX " + String(results[2].thdToNyquistPercent, 1) + "%", 205, 33, 1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Total THD to Nyquist | THI phase H3: " + String(thiInjectedThirdDbc, 1) + " dBc", 7, 48, 1);

  constexpr int16_t left = 25;
  constexpr int16_t right = 314;
  constexpr int16_t top = 65;
  constexpr int16_t bottom = 193;
  constexpr int16_t plotWidth = right - left;
  constexpr int16_t plotHeight = bottom - top;

  tft.drawRect(left, top, plotWidth, plotHeight, TFT_DARKGREY);
  for (int db = 0; db >= -60; db -= 20) {
    const int16_t y = top + static_cast<int16_t>((-db / 60.0f) * plotHeight);
    tft.drawFastHLine(left + 1, y, plotWidth - 2, TFT_DARKGREY);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawRightString(String(db), left - 3, y - 4, 1);
  }

  const uint16_t colors[3] = {TFT_GREEN, TFT_CYAN, TFT_YELLOW};
  constexpr float DB_FLOOR = -60.0f;
  constexpr int16_t groupWidth = plotWidth / MAX_HARMONIC;
  for (uint8_t harmonic = 1; harmonic <= MAX_HARMONIC; ++harmonic) {
    const int16_t groupStart = left + (harmonic - 1) * groupWidth;
    for (uint8_t mode = 0; mode < 3; ++mode) {
      float db = results[mode].dbc[harmonic];
      if (db < DB_FLOOR) db = DB_FLOOR;
      if (db > 0.0f) db = 0.0f;
      const int16_t barHeight = static_cast<int16_t>(((db - DB_FLOOR) / -DB_FLOOR) * plotHeight);
      const int16_t barX = groupStart + 2 + mode * 5;
      if (barHeight > 0) tft.drawFastVLine(barX, bottom - barHeight, barHeight, colors[mode]);
    }
    if (harmonic == 1 || (harmonic % 2) == 1) {
      const int16_t x = groupStart + groupWidth / 2;
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.drawCentreString(String(harmonic), x, 197, 1);
    }
  }

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("HARMONIC ORDER", 8, 218, 1);
  tft.drawRightString("dBc", 312, 218, 1);
}

void printHelp() {
  Serial.println();
  Serial.println("MixedSignalLab - Harmonic comparison");
  Serial.println("  c = calculate all three modes and print CSV");
  Serial.println("  p = redraw comparison chart");
  Serial.println("  h = help");
  Serial.println();
  Serial.println("Analysis: 1020 reference samples over one electrical cycle (170 per 60-degree sector).");
  Serial.println("Metric: line-to-line Vab reference waveform.");
  Serial.println("CSV reports THD to Nyquist and a separate H2-H15 value matching the chart.");
  Serial.println("This is reference-waveform analysis, not measured PWM power-stage THD.");
}

void printCsv() {
  Serial.println();
  Serial.println("HARMONIC_COMPARE_BEGIN");
  Serial.printf("# samples_per_cycle=%u, modulation_index=%.3f, output=Vab_reference, chart_max_harmonic=%u\n",
                SAMPLE_COUNT, MODULATION_INDEX, MAX_HARMONIC);
  Serial.printf("# THD_to_nyquist_percent: SPWM=%.6f, THI=%.6f, SIX_STEP=%.6f\n",
                results[0].thdToNyquistPercent, results[1].thdToNyquistPercent, results[2].thdToNyquistPercent);
  Serial.printf("# THD_H2_H15_percent: SPWM=%.6f, THI=%.6f, SIX_STEP=%.6f\n",
                results[0].thdH15Percent, results[1].thdH15Percent, results[2].thdH15Percent);
  Serial.printf("# THI_phase_leg_injected_3rd_dBc=%.3f; triplen common-mode cancels in Vab\n",
                thiInjectedThirdDbc);
  Serial.println("harmonic,spwm_amplitude,spwm_dbc,thi_amplitude,thi_dbc,six_step_amplitude,six_step_dbc");
  for (uint8_t harmonic = 1; harmonic <= MAX_HARMONIC; ++harmonic) {
    Serial.printf("%u,%.7f,%.3f,%.7f,%.3f,%.7f,%.3f\n",
                  harmonic,
                  results[0].amplitude[harmonic], results[0].dbc[harmonic],
                  results[1].amplitude[harmonic], results[1].dbc[harmonic],
                  results[2].amplitude[harmonic], results[2].dbc[harmonic]);
  }
  Serial.println("HARMONIC_COMPARE_END");
}

void processCommand(char* line) {
  while (*line && isspace(static_cast<unsigned char>(*line))) ++line;
  if (*line == '\0') return;
  for (char* p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

  if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0) {
    printHelp();
  } else if (strcmp(line, "p") == 0) {
    if (calculated) drawComparisonScreen();
    else Serial.println("No results yet. Send c to calculate.");
  } else if (strcmp(line, "c") == 0 || strcmp(line, "run") == 0) {
    Serial.println("Calculating 3-phase line-to-line reference harmonics...");
    calculateAll();
    drawComparisonScreen();
    printCsv();
    Serial.println("Harmonic comparison complete.");
  } else {
    Serial.println("Command not recognised. Send h for help.");
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        Serial.print("\r\n> ");
        processCommand(commandBuffer);
        commandLength = 0;
        Serial.print("\r\n> ");
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
      Serial.write(c);
    } else {
      commandLength = 0;
      Serial.println("\r\nInput line too long; command discarded.");
    }
  }
}
} // namespace

void runHarmonicComparisonSetup() {
  Serial.begin(115200);
  delay(250);
  tft.init();
  tft.setRotation(1);
  drawWaitingScreen();
  calculateAll();
  drawComparisonScreen();

  Serial.println();
  Serial.println("====================================================");
  Serial.println("MixedSignalLab - Harmonic Comparison");
  Serial.println("Classic ESP32-WROOM-32 / ILI9341 TFT");
  Serial.println("Compares SPWM, THI and six-step reference waveforms.");
  Serial.println("No physical PWM output or motor load is measured here.");
  Serial.println("Send c for CSV, p to redraw, h for help.");
  Serial.println("====================================================");
  printCsv();
  printHelp();
}

void runHarmonicComparisonLoop() {
  pollSerial();
  delay(2);
}
