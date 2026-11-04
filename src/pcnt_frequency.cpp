#include <Arduino.h>
#include <TFT_eSPI.h>
#include <driver/pcnt.h>
#include <esp_err.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pcnt_frequency.h"

namespace {
constexpr gpio_num_t PWM_OUTPUT_GPIO = GPIO_NUM_16;
constexpr gpio_num_t PCNT_INPUT_GPIO = GPIO_NUM_35;
constexpr uint8_t LEDC_CHANNEL = 3; // Separate from the three-channel PWM app's channels.
constexpr uint8_t LEDC_RESOLUTION_BITS = 10;
constexpr uint16_t LEDC_MAX_DUTY = (1U << LEDC_RESOLUTION_BITS) - 1U;
constexpr uint32_t DEFAULT_PWM_HZ = 10000;
constexpr float DEFAULT_DUTY_PERCENT = 50.0f;
constexpr uint32_t MIN_PWM_HZ = 1000;
constexpr uint32_t MAX_PWM_HZ = 20000;
constexpr uint32_t GATE_INTERVAL_US = 1000000UL;
constexpr uint16_t PCNT_GLITCH_FILTER_APB_CYCLES = 80; // 1 us at 80 MHz APB clock.
constexpr int16_t PCNT_HIGH_LIMIT = 30000; // Above 20 kcounts in a 1 s gate.

TFT_eSPI tft;
char commandBuffer[80];
size_t commandLength = 0;
bool pcntReady = false;
bool pwmAttached = false;
bool running = false;
uint32_t requestedPwmHz = DEFAULT_PWM_HZ;
float actualPwmHz = 0.0f;
float dutyPercent = DEFAULT_DUTY_PERCENT;
int16_t lastPulseCount = 0;
float measuredHz = 0.0f;
float errorPercent = 0.0f;
uint32_t lastGateUs = 0;
uint32_t lastUiMs = 0;

void drawLiveValues();

uint32_t dutyCountsFromPercent(float percent) {
  const float bounded = constrain(percent, 0.0f, 100.0f);
  return static_cast<uint32_t>(lroundf((bounded / 100.0f) * LEDC_MAX_DUTY));
}

void drawScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, tft.width(), 32, TFT_NAVY);
  tft.drawFastHLine(0, 32, tft.width(), TFT_CYAN);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MIXEDSIGNALLAB - PCNT", 8, 8, 2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("Hardware pulse counter", 8, 43, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("PWM GPIO16 -> PCNT GPIO35", 8, 68, 2);
  tft.drawString("Target " + String(requestedPwmHz) + " Hz", 8, 94, 2);
  tft.drawString("Duty " + String(dutyPercent, 1) + "%", 178, 94, 2);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Measured frequency", 8, 124, 2);
  tft.drawString("Pulse count / 1 s gate", 8, 169, 2);
  tft.drawString("Frequency error", 8, 199, 2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Connect GPIO16 to GPIO35", 8, 225, 1);
  drawLiveValues();
}

void drawLiveValues() {
  tft.fillRect(8, 145, 304, 23, TFT_BLACK);
  tft.setTextColor(running ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(running ? "RUNNING" : "STOPPED", 235, 43, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  const String freqText = running ? (String(measuredHz, 2) + " Hz") : String("-- Hz");
  tft.drawString(freqText, 8, 145, 2);

  tft.fillRect(8, 185, 305, 15, TFT_BLACK);
  tft.drawString(String(lastPulseCount) + " pulses", 8, 185, 2);
  tft.fillRect(155, 199, 158, 20, TFT_BLACK);
  tft.drawString(running ? (String(errorPercent, 3) + "%") : String("--"), 155, 199, 2);
}

void printHelp() {
  Serial.println();
  Serial.println("MixedSignalLab - PCNT frequency verification");
  Serial.println("Connect GPIO16 (LEDC PWM output) to GPIO35 (PCNT input) with one jumper.");
  Serial.println("Commands (send a line, then Enter):");
  Serial.println("  h                 help");
  Serial.println("  r                 start PWM and frequency measurement");
  Serial.println("  x                 stop PWM and pause PCNT");
  Serial.println("  f <Hz>            PWM frequency, 1000..20000");
  Serial.println("  d <percent>       PWM duty cycle, 10..90");
  Serial.println("  p                 redraw TFT");
  Serial.println("  ?                 print current status");
  Serial.println("PCNT counts rising edges on GPIO35 over a measured 1-second gate.");
  Serial.println("GPIO35 is input-only and has no internal pull-up/down; connect the jumper.");
  Serial.println("Logic-level test only. Do not connect a motor or power stage.");
}

void printStatus() {
  Serial.printf("PCNT status: %s | output=GPIO%u | input=GPIO%u | requested=%lu Hz | LEDC-reported=%.2f Hz | duty=%.1f%%\n",
                running ? "RUNNING" : "STOPPED",
                static_cast<unsigned>(PWM_OUTPUT_GPIO), static_cast<unsigned>(PCNT_INPUT_GPIO),
                static_cast<unsigned long>(requestedPwmHz), actualPwmHz, dutyPercent);
  if (running) {
    Serial.printf("Last measurement: pulses=%d | measured=%.3f Hz | error=%+.4f%%\n",
                  lastPulseCount, measuredHz, errorPercent);
  }
}

void configurePwm() {
  actualPwmHz = static_cast<float>(ledcSetup(LEDC_CHANNEL, requestedPwmHz, LEDC_RESOLUTION_BITS));
  if (!pwmAttached) {
    ledcAttachPin(static_cast<uint8_t>(PWM_OUTPUT_GPIO), LEDC_CHANNEL);
    pwmAttached = true;
  }
  ledcWrite(LEDC_CHANNEL, running ? dutyCountsFromPercent(dutyPercent) : 0U);
  Serial.printf("LEDC configured: requested=%lu Hz, reported=%.2f Hz, duty=%.1f%%\n",
                static_cast<unsigned long>(requestedPwmHz), actualPwmHz,
                running ? dutyPercent : 0.0f);
}

bool configurePcnt() {
  pcnt_config_t config = {};
  config.pulse_gpio_num = static_cast<int>(PCNT_INPUT_GPIO);
  config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  config.lctrl_mode = PCNT_MODE_KEEP;
  config.hctrl_mode = PCNT_MODE_KEEP;
  config.pos_mode = PCNT_COUNT_INC;
  config.neg_mode = PCNT_COUNT_DIS;
  config.counter_h_lim = PCNT_HIGH_LIMIT;
  config.counter_l_lim = 0;
  config.unit = PCNT_UNIT_0;
  config.channel = PCNT_CHANNEL_0;

  esp_err_t err = pcnt_unit_config(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: pcnt_unit_config failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = pcnt_set_filter_value(PCNT_UNIT_0, PCNT_GLITCH_FILTER_APB_CYCLES);
  if (err != ESP_OK) {
    Serial.printf("ERROR: pcnt_set_filter_value failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = pcnt_filter_enable(PCNT_UNIT_0);
  if (err != ESP_OK) {
    Serial.printf("ERROR: pcnt_filter_enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  return true;
}

void startMeasurement() {
  if (!pcntReady) {
    Serial.println("PCNT is not configured; cannot start.");
    return;
  }
  running = true;
  configurePwm();
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  lastGateUs = micros();
  lastPulseCount = 0;
  measuredHz = 0.0f;
  errorPercent = 0.0f;
  Serial.println("PWM started. PCNT frequency measurement active.");
  drawScreen();
}

void stopMeasurement() {
  running = false;
  ledcWrite(LEDC_CHANNEL, 0);
  if (pcntReady) {
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
  }
  lastPulseCount = 0;
  measuredHz = 0.0f;
  errorPercent = 0.0f;
  Serial.println("PWM stopped; output GPIO16 held low and PCNT paused.");
  drawScreen();
}

void takeMeasurement() {
  if (!running || !pcntReady) return;

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastGateUs;
  if (elapsedUs < GATE_INTERVAL_US) return;

  pcnt_counter_pause(PCNT_UNIT_0);
  int16_t count = 0;
  const esp_err_t readErr = pcnt_get_counter_value(PCNT_UNIT_0, &count);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  if (readErr != ESP_OK) {
    Serial.printf("ERROR: pcnt_get_counter_value failed: %s\n", esp_err_to_name(readErr));
    lastGateUs = nowUs;
    return;
  }

  lastPulseCount = count;
  measuredHz = (static_cast<float>(count) * 1000000.0f) / static_cast<float>(elapsedUs);
  errorPercent = (actualPwmHz > 0.0f)
                     ? ((measuredHz - actualPwmHz) / actualPwmHz) * 100.0f
                     : 0.0f;
  lastGateUs = nowUs;

  Serial.printf("PCNT_MEASUREMENT,requested_hz=%lu,ledc_reported_hz=%.3f,gate_us=%lu,pulses=%d,measured_hz=%.3f,error_percent=%+.5f\n",
                static_cast<unsigned long>(requestedPwmHz), actualPwmHz,
                static_cast<unsigned long>(elapsedUs), count, measuredHz, errorPercent);
  drawLiveValues();
}

void processCommand(char* line) {
  while (*line && isspace(static_cast<unsigned char>(*line))) ++line;
  if (*line == '\0') return;
  for (char* p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

  if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0) { printHelp(); return; }
  if (strcmp(line, "r") == 0 || strcmp(line, "start") == 0) { startMeasurement(); return; }
  if (strcmp(line, "x") == 0 || strcmp(line, "stop") == 0) { stopMeasurement(); return; }
  if (strcmp(line, "p") == 0) { drawScreen(); return; }
  if (strcmp(line, "?") == 0 || strcmp(line, "status") == 0) { printStatus(); return; }

  char command[16] = {0};
  float value = 0.0f;
  if (sscanf(line, "%15s %f", command, &value) == 2) {
    if (strcmp(command, "f") == 0 || strcmp(command, "freq") == 0) {
      if (!isfinite(value) || value < MIN_PWM_HZ || value > MAX_PWM_HZ) {
        Serial.println("Frequency must be between 1000 and 20000 Hz.");
        return;
      }
      requestedPwmHz = static_cast<uint32_t>(lroundf(value));
      configurePwm();
      if (running) {
        pcnt_counter_pause(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_0);
        pcnt_counter_resume(PCNT_UNIT_0);
        lastGateUs = micros();
      }
      drawScreen();
      return;
    }
    if (strcmp(command, "d") == 0 || strcmp(command, "duty") == 0) {
      if (!isfinite(value) || value < 10.0f || value > 90.0f) {
        Serial.println("Duty must be between 10 and 90 percent for reliable PCNT pulse widths.");
        return;
      }
      dutyPercent = value;
      if (running) ledcWrite(LEDC_CHANNEL, dutyCountsFromPercent(dutyPercent));
      Serial.printf("Duty cycle set to %.1f%%.\n", dutyPercent);
      drawScreen();
      return;
    }
  }

  Serial.println("Command not recognised. Send h for help.");
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
      Serial.println("\r\nInput too long; command discarded.");
    }
  }
}
} // namespace

void runPcntFrequencySetup() {
  Serial.begin(115200);
  delay(250);

  tft.init();
  tft.setRotation(1);

  pinMode(static_cast<uint8_t>(PWM_OUTPUT_GPIO), OUTPUT);
  digitalWrite(static_cast<uint8_t>(PWM_OUTPUT_GPIO), LOW);
  configurePwm();

  pcntReady = configurePcnt();
  if (pcntReady) {
    Serial.println("PCNT unit 0 / channel 0 configured for rising-edge counting on GPIO35.");
    Serial.printf("PCNT glitch filter: %u APB cycles (%.2f us at 80 MHz).\n",
                  PCNT_GLITCH_FILTER_APB_CYCLES,
                  static_cast<float>(PCNT_GLITCH_FILTER_APB_CYCLES) / 80.0f);
  }

  drawScreen();
  Serial.println();
  Serial.println("====================================================");
  Serial.println("MixedSignalLab - PCNT Frequency Verification");
  Serial.println("Target: classic ESP32-WROOM-32 / Arduino Core 2.x");
  Serial.println("LEDC PWM output: GPIO16");
  Serial.println("PCNT pulse input: GPIO35 (input-only, no internal pull-up/down)");
  Serial.println("REQUIRED JUMPER: GPIO16 -> GPIO35");
  Serial.println("Default: 10 kHz, 50% duty, PWM STOPPED until command 'r'.");
  Serial.println("Measurement: rising-edge count over an actual 1-second gate.");
  Serial.println("Logic-level only; no motor or power stage.");
  Serial.println("====================================================");
  printHelp();
}

void runPcntFrequencyLoop() {
  pollSerial();
  takeMeasurement();
  const uint32_t nowMs = millis();
  if (nowMs - lastUiMs >= 500) {
    lastUiMs = nowMs;
    drawLiveValues();
  }
  delay(1);
}
