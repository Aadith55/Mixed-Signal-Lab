#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_timer.h"
#include "pwm_generator.h"

namespace {
constexpr uint8_t PWM_PINS[3] = {16, 17, 4};
constexpr uint8_t PWM_CHANNELS[3] = {0, 1, 2};
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint16_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION_BITS) - 1U;
constexpr uint32_t DEFAULT_CARRIER_HZ = 10000;
constexpr float DEFAULT_ELECTRICAL_HZ = 5.0f;
constexpr float DEFAULT_AMPLITUDE_PERCENT = 80.0f;
constexpr uint32_t CONTROL_UPDATE_US = 1000; // 1 kHz duty-reference update
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;

TFT_eSPI tft;
char commandBuffer[80];
size_t commandLength = 0;

enum class WaveMode : uint8_t { SINE, THIRD_HARMONIC, SIX_STEP, STATIC };
WaveMode mode = WaveMode::SINE;
bool running = false;
uint32_t carrierHz = DEFAULT_CARRIER_HZ;
float actualCarrierHz = 0.0f;
float electricalHz = DEFAULT_ELECTRICAL_HZ;
float amplitudePercent = DEFAULT_AMPLITUDE_PERCENT;
float staticDuty[3] = {50.0f, 50.0f, 50.0f};
float currentDuty[3] = {0.0f, 0.0f, 0.0f};
uint64_t runStartUs = 0;
uint32_t lastControlUs = 0;
uint32_t lastUiMs = 0;

const char* modeName() {
  switch (mode) {
    case WaveMode::SINE: return "SINE SPWM";
    case WaveMode::THIRD_HARMONIC: return "THI SPWM";
    case WaveMode::SIX_STEP: return "SIX-STEP";
    case WaveMode::STATIC: return "STATIC DUTY";
  }
  return "UNKNOWN";
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void writeDuty(uint8_t index, float dutyPercent) {
  dutyPercent = clampFloat(dutyPercent, 0.0f, 100.0f);
  currentDuty[index] = dutyPercent;
  const uint32_t dutyCounts = static_cast<uint32_t>(lroundf((dutyPercent / 100.0f) * PWM_MAX_DUTY));
  ledcWrite(PWM_CHANNELS[index], dutyCounts);
}

void writeAllLow() {
  for (uint8_t i = 0; i < 3; ++i) writeDuty(i, 0.0f);
}

void configureCarrier() {
  for (uint8_t i = 0; i < 3; ++i) {
    actualCarrierHz = static_cast<float>(ledcSetup(PWM_CHANNELS[i], carrierHz, PWM_RESOLUTION_BITS));
    ledcAttachPin(PWM_PINS[i], PWM_CHANNELS[i]);
  }
  Serial.printf("PWM carrier configured: requested=%lu Hz, reported=%.1f Hz, resolution=%u bits\n",
                static_cast<unsigned long>(carrierHz), actualCarrierHz, PWM_RESOLUTION_BITS);
}

float sineReference(float angle) {
  return sinf(angle);
}

float thirdHarmonicReference(float angle) {
  // Conventional THI: add one-sixth third harmonic, then scale by 2/sqrt(3)
  // so the combined reference has a +/-1 peak while preserving the fundamental.
  constexpr float THI_GAIN = 1.15470053838f;
  return (sinf(angle) + (1.0f / 6.0f) * sinf(3.0f * angle)) * THI_GAIN;
}

void sixStepReferences(float angle, float refs[3]) {
  // Six 60-degree sectors. Values are normalized phase-leg duty references:
  // 100%, 50% (neutral), or 0%. This is a logic-level modulation pattern only.
  float wrapped = fmodf(angle, TWO_PI_F);
  if (wrapped < 0.0f) wrapped += TWO_PI_F;
  const uint8_t sector = static_cast<uint8_t>(wrapped / (PI_F / 3.0f));
  switch (sector) {
    case 0: refs[0] = 1.0f; refs[1] = 0.0f; refs[2] = 0.5f; break;
    case 1: refs[0] = 1.0f; refs[1] = 0.5f; refs[2] = 0.0f; break;
    case 2: refs[0] = 0.5f; refs[1] = 1.0f; refs[2] = 0.0f; break;
    case 3: refs[0] = 0.0f; refs[1] = 1.0f; refs[2] = 0.5f; break;
    case 4: refs[0] = 0.0f; refs[1] = 0.5f; refs[2] = 1.0f; break;
    default: refs[0] = 0.5f; refs[1] = 0.0f; refs[2] = 1.0f; break;
  }
}

void updateModulation(uint32_t nowUs) {
  if (!running) return;

  if (mode == WaveMode::STATIC) {
    for (uint8_t i = 0; i < 3; ++i) writeDuty(i, staticDuty[i]);
    return;
  }

  const float elapsedSeconds = static_cast<float>(static_cast<uint32_t>(nowUs - static_cast<uint32_t>(runStartUs))) / 1000000.0f;
  const float theta = fmodf(TWO_PI_F * electricalHz * elapsedSeconds, TWO_PI_F);
  const float modulation = amplitudePercent / 100.0f;
  const float phaseOffsets[3] = {0.0f, -2.0f * PI_F / 3.0f, 2.0f * PI_F / 3.0f};
  float refs[3] = {0.0f, 0.0f, 0.0f};

  if (mode == WaveMode::SIX_STEP) {
    sixStepReferences(theta, refs);
    for (uint8_t i = 0; i < 3; ++i) {
      const float duty = 50.0f + (refs[i] - 0.5f) * 100.0f * modulation;
      writeDuty(i, duty);
    }
    return;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    const float phase = theta + phaseOffsets[i];
    refs[i] = (mode == WaveMode::THIRD_HARMONIC)
                  ? thirdHarmonicReference(phase)
                  : sineReference(phase);
    writeDuty(i, 50.0f + 50.0f * modulation * refs[i]);
  }
}

void drawStaticScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 30, TFT_NAVY);
  tft.drawFastHLine(0, 30, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MIXEDSIGNALLAB - PWM", 8, 7, 2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("3-channel logic-level PWM", 8, 39, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Carrier " + String(carrierHz) + " Hz", 8, 65, 2);
  tft.drawString("Elec " + String(electricalHz, 1) + " Hz", 164, 65, 2);
  tft.drawString("Mode " + String(modeName()), 8, 91, 2);
  tft.drawString("Amp " + String(amplitudePercent, 0) + "%", 164, 91, 2);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("A", 12, 121, 2);
  tft.drawString("B", 12, 154, 2);
  tft.drawString("C", 12, 187, 2);
  tft.drawRect(40, 120, 198, 13, TFT_DARKGREY);
  tft.drawRect(40, 153, 198, 13, TFT_DARKGREY);
  tft.drawRect(40, 186, 198, 13, TFT_DARKGREY);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("GPIO16 / GPIO17 / GPIO4", 8, 218, 1);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawRightString(running ? "RUNNING" : "STOPPED", 310, 39, 2);
}

void drawLiveValues() {
  // Update only small regions to avoid full-screen flicker.
  tft.fillRect(222, 37, 94, 22, TFT_BLACK);
  tft.setTextColor(running ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawRightString(running ? "RUNNING" : "STOPPED", 311, 39, 2);

  const int16_t yPositions[3] = {122, 155, 188};
  for (uint8_t i = 0; i < 3; ++i) {
    tft.fillRect(41, yPositions[i], 196, 11, TFT_BLACK);
    const int16_t barWidth = static_cast<int16_t>(currentDuty[i] * 1.94f);
    if (barWidth > 0) tft.fillRect(41, yPositions[i], barWidth, 11,
                                  (i == 0) ? TFT_GREEN : ((i == 1) ? TFT_CYAN : TFT_YELLOW));
    tft.drawRect(40, yPositions[i] - 2, 198, 13, TFT_DARKGREY);
    tft.fillRect(245, yPositions[i] - 3, 70, 17, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawRightString(String(currentDuty[i], 1) + "%", 312, yPositions[i] - 2, 2);
  }
}

void redrawScreen() {
  drawStaticScreen();
  drawLiveValues();
}

void printHelp() {
  Serial.println();
  Serial.println("MixedSignalLab - 3-channel PWM generator");
  Serial.println("Commands (send a line, then Enter):");
  Serial.println("  h                 help");
  Serial.println("  r                 start modulation");
  Serial.println("  x                 stop all PWM outputs (0% duty)");
  Serial.println("  f <Hz>            carrier frequency, 1000..20000");
  Serial.println("  e <Hz>            electrical/reference frequency, 0.1..20");
  Serial.println("  a <percent>       modulation amplitude, 0..100");
  Serial.println("  m sine            sinusoidal PWM references");
  Serial.println("  m thi             third-harmonic-injection references");
  Serial.println("  m six             six-step 60-degree sector references");
  Serial.println("  m static          fixed independent channel duties");
  Serial.println("  d <ch> <percent>  set static duty for channel 1, 2 or 3");
  Serial.println("  p                 redraw TFT");
  Serial.println("  ?                 print current status");
  Serial.println();
  Serial.println("Safety: outputs are 3.3V logic-level signals only.");
  Serial.println("No complementary outputs, dead-time, gate driver or motor power stage.");
}

void printStatus() {
  Serial.printf("PWM status: %s | mode=%s | carrier=%lu Hz (reported %.1f Hz) | electrical=%.2f Hz | amplitude=%.1f%%\n",
                running ? "RUNNING" : "STOPPED", modeName(),
                static_cast<unsigned long>(carrierHz), actualCarrierHz,
                electricalHz, amplitudePercent);
  Serial.printf("Duty A/B/C = %.2f / %.2f / %.2f %%\n", currentDuty[0], currentDuty[1], currentDuty[2]);
}

void startOutputs() {
  running = true;
  runStartUs = esp_timer_get_time();
  lastControlUs = micros();
  updateModulation(lastControlUs);
  Serial.println("PWM modulation STARTED.");
  printStatus();
  redrawScreen();
}

void stopOutputs() {
  running = false;
  writeAllLow();
  Serial.println("PWM outputs STOPPED; all channels set to 0% duty.");
  redrawScreen();
}

void processCommand(char* line) {
  while (*line && isspace(static_cast<unsigned char>(*line))) ++line;
  if (*line == '\0') return;
  for (char* p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

  char command[16] = {0};
  float value = 0.0f;
  int channel = 0;

  if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0) { printHelp(); return; }
  if (strcmp(line, "r") == 0 || strcmp(line, "start") == 0) { startOutputs(); return; }
  if (strcmp(line, "x") == 0 || strcmp(line, "stop") == 0) { stopOutputs(); return; }
  if (strcmp(line, "p") == 0) { redrawScreen(); return; }
  if (strcmp(line, "?") == 0 || strcmp(line, "status") == 0) { printStatus(); return; }

  if (sscanf(line, "%15s %f", command, &value) == 2) {
    if (strcmp(command, "f") == 0 || strcmp(command, "freq") == 0) {
      if (value < 1000.0f || value > 20000.0f) {
        Serial.println("Carrier must be between 1000 and 20000 Hz."); return;
      }
      carrierHz = static_cast<uint32_t>(lroundf(value));
      configureCarrier();
      if (running) updateModulation(micros()); else writeAllLow();
      redrawScreen();
      return;
    }
    if (strcmp(command, "e") == 0 || strcmp(command, "elec") == 0) {
      if (value < 0.1f || value > 20.0f) {
        Serial.println("Electrical/reference frequency must be between 0.1 and 20 Hz."); return;
      }
      electricalHz = value;
      if (running) runStartUs = esp_timer_get_time();
      redrawScreen();
      Serial.printf("Electrical/reference frequency set to %.2f Hz.\n", electricalHz);
      return;
    }
    if (strcmp(command, "a") == 0 || strcmp(command, "amp") == 0) {
      amplitudePercent = clampFloat(value, 0.0f, 100.0f);
      if (running) updateModulation(micros());
      redrawScreen();
      Serial.printf("Modulation amplitude set to %.1f%%.\n", amplitudePercent);
      return;
    }
    if (strcmp(command, "m") == 0 || strcmp(command, "mode") == 0) {
      // The mode token is handled below; the float parse is not used here.
    }
  }

  char modeToken[24] = {0};
  if (sscanf(line, "%15s %23s", command, modeToken) == 2 &&
      (strcmp(command, "m") == 0 || strcmp(command, "mode") == 0)) {
    if (strcmp(modeToken, "sine") == 0) mode = WaveMode::SINE;
    else if (strcmp(modeToken, "thi") == 0 || strcmp(modeToken, "third") == 0) mode = WaveMode::THIRD_HARMONIC;
    else if (strcmp(modeToken, "six") == 0 || strcmp(modeToken, "sixstep") == 0) mode = WaveMode::SIX_STEP;
    else if (strcmp(modeToken, "static") == 0) mode = WaveMode::STATIC;
    else { Serial.println("Mode must be sine, thi, six, or static."); return; }
    if (running) {
      runStartUs = esp_timer_get_time();
      updateModulation(micros());
    }
    Serial.printf("Mode set to %s.\n", modeName());
    redrawScreen();
    return;
  }

  float duty = 0.0f;
  if (sscanf(line, "%15s %d %f", command, &channel, &duty) == 3 &&
      (strcmp(command, "d") == 0 || strcmp(command, "duty") == 0)) {
    if (channel < 1 || channel > 3) { Serial.println("Channel must be 1, 2, or 3."); return; }
    if (duty < 0.0f || duty > 100.0f) { Serial.println("Duty must be between 0 and 100%."); return; }
    staticDuty[channel - 1] = duty;
    if (mode == WaveMode::STATIC && running) updateModulation(micros());
    redrawScreen();
    Serial.printf("Static duty CH%d set to %.1f%%.\n", channel, duty);
    return;
  }

  Serial.println("Command not recognised. Send h for help.");
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    // Accept LF, CR, or CRLF line endings. Some serial monitors send only CR;
    // the previous parser ignored CR and therefore appeared not to accept input.
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
      // Local echo makes it clear that the ESP32 received the typed character.
      Serial.write(c);
    } else {
      commandLength = 0;
      Serial.println("\r\nInput line too long; command discarded.");
    }
  }
}

} // namespace

void runPwmGeneratorSetup() {
  Serial.begin(115200);
  delay(250);

  tft.init();
  tft.setRotation(1);

  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);
    digitalWrite(PWM_PINS[i], LOW);
  }
  configureCarrier();
  writeAllLow();
  redrawScreen();

  Serial.println();
  Serial.println("====================================================");
  Serial.println("MixedSignalLab - 3-channel PWM Generator");
  Serial.println("Target: classic ESP32-WROOM-32 / ILI9341 TFT");
  Serial.println("Outputs: GPIO16, GPIO17, GPIO4 (3.3V logic only)");
  Serial.println("LEDC: 10-bit resolution; independent duty per channel");
  Serial.println("Default carrier=10kHz, reference=5Hz, amplitude=80%");
  Serial.println("Default state: STOPPED (all outputs low)");
  Serial.println("Commands: h=help, r=start, x=stop, f/e/a/m/d/p/?");
  Serial.println("No complementary outputs or dead-time; do not connect a motor or power stage.");
  Serial.println("====================================================");
  printHelp();
}

void runPwmGeneratorLoop() {
  pollSerial();
  const uint32_t now = micros();
  if (running && static_cast<uint32_t>(now - lastControlUs) >= CONTROL_UPDATE_US) {
    lastControlUs = now;
    updateModulation(now);
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastUiMs >= 250) {
    lastUiMs = nowMs;
    drawLiveValues();
  }
  delay(1);
}
