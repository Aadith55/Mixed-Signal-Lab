#include <Arduino.h>

#if defined(APP_TFT_HELLO)
  #include <SPI.h>
  #include <TFT_eSPI.h>
  #include "hello_screen.h"
#elif defined(APP_DAC_ADC_LOOPBACK)
  #include "loopback_test.h"
#elif defined(APP_ADC_CHARACTERIZATION)
  #include "adc_characterization.h"
#elif defined(APP_SPECTRUM_ANALYZER)
  #include "spectrum_analyzer.h"
#elif defined(APP_PWM_GENERATOR)
  #include "pwm_generator.h"
#elif defined(APP_HARMONIC_COMPARISON)
  #include "harmonic_comparison.h"
#elif defined(APP_PCNT_FREQUENCY)
  #include "pcnt_frequency.h"
#else
  #error "Select a PlatformIO environment: tft_hello, dac_adc_loopback, adc_characterization, spectrum_analyzer, pwm_generator, harmonic_comparison, or pcnt_frequency"
#endif

void setup() {
#if defined(APP_TFT_HELLO)
  runHelloSetup();
#elif defined(APP_DAC_ADC_LOOPBACK)
  runLoopbackSetup();
#elif defined(APP_ADC_CHARACTERIZATION)
  runAdcCharacterizationSetup();
#elif defined(APP_SPECTRUM_ANALYZER)
  runSpectrumAnalyzerSetup();
#elif defined(APP_PWM_GENERATOR)
  runPwmGeneratorSetup();
#elif defined(APP_HARMONIC_COMPARISON)
  runHarmonicComparisonSetup();
#elif defined(APP_PCNT_FREQUENCY)
  runPcntFrequencySetup();
#endif
}

void loop() {
#if defined(APP_TFT_HELLO)
  runHelloLoop();
#elif defined(APP_DAC_ADC_LOOPBACK)
  runLoopbackLoop();
#elif defined(APP_ADC_CHARACTERIZATION)
  runAdcCharacterizationLoop();
#elif defined(APP_SPECTRUM_ANALYZER)
  runSpectrumAnalyzerLoop();
#elif defined(APP_PWM_GENERATOR)
  runPwmGeneratorLoop();
#elif defined(APP_HARMONIC_COMPARISON)
  runHarmonicComparisonLoop();
#elif defined(APP_PCNT_FREQUENCY)
  runPcntFrequencyLoop();
#endif
}
