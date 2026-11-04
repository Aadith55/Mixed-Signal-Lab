#include <Arduino.h>
#include <TFT_eSPI.h>
#include "hello_screen.h"

namespace {
TFT_eSPI tft;
uint32_t lastRefreshMs = 0;

void drawStaticScreen() {
  const int16_t w = tft.width();
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, w, 42, TFT_NAVY);
  tft.drawFastHLine(0, 42, w, TFT_CYAN);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("MixedSignalLab", 12, 10, 4);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("TFT HARDWARE TEST", 12, 58, 2);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Display initialised", 12, 94, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Controller: ILI9341", 12, 130, 2);
  tft.drawString("Interface: SPI", 12, 158, 2);
  tft.drawString("Resolution:", 12, 178, 2);
  tft.drawString(String(w) + " x " + String(tft.height()), 138, 178, 2);

  tft.drawRoundRect(8, 218, w - 16, 20, 5, TFT_DARKGREY);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawCentreString("Hello from ESP32!", w / 2, 220, 2);
}
}

void runHelloSetup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("====================================");
  Serial.println("MixedSignalLab - TFT Hello World");
  Serial.println("ESP32-WROOM-32 / ILI9341 / TFT_eSPI");
  Serial.println("====================================");

  tft.init();
  tft.setRotation(1); // Landscape: 320 x 240
  drawStaticScreen();

  Serial.printf("TFT initialised: %d x %d pixels\n", tft.width(), tft.height());
  Serial.println("If you can read the screen, TFT wiring is working.");
  lastRefreshMs = millis();
}

void runHelloLoop() {
  if (millis() - lastRefreshMs >= 1000) {
    lastRefreshMs = millis();
    const uint32_t seconds = millis() / 1000;
    tft.fillRect(12, 198, tft.width() - 24, 16, TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(String("Uptime: ") + String(seconds) + " s", 12, 198, 2);
  }
  delay(5);
}
