# MixedSignalLab v1.0.0 — User-Side Test Results

Measurements below were supplied from validation on one classic ESP32-WROOM-32 board and one ILI9341 display. They are experimental observations, not production specifications.

## Hardware

- MCU: classic ESP32-WROOM-32 / ESP32 DevKit (PlatformIO board: esp32dev)
- Display: 2.4-inch LCDWIKI MSP2402, ILI9341 SPI, 240 x 320 native / 320 x 240 landscape
- TFT CS: GPIO33
- DAC1-to-ADC1 loopback: GPIO25 to GPIO34
- PWM outputs: GPIO16, GPIO17, GPIO4
- PCNT measurement loopback: GPIO16 to GPIO35

## TFT and DAC/ADC characterisation

- TFT hello-world: passed.
- DAC-to-ADC loopback: passed.
- Characterisation sweep: 256 DAC codes, 64 ADC samples per code.
- Average noise sigma: 7.27 raw ADC counts.
- Maximum noise sigma in captured run: 18.84 raw ADC counts.
- Maximum absolute loopback INL: 22.08 endpoint-referenced loopback LSB.
- RMS loopback INL: 15.08 loopback LSB.
- Calculated RMS transfer error before / after inverse LUT: 15.081 / 0.340 loopback LSB.
- LUT improvement was calculated on the same calibration dataset; independent validation remains outstanding.
- INL/DNL describe the combined DAC-wire-ADC loopback, not intrinsic ADC performance.

## FFT spectrum analyser

| Requested rate | Measured rate | Peak bin | Peak frequency | THD | Relative noise floor |
|---:|---:|---:|---:|---:|---:|
| 1 kS/s | 1,011 S/s | 32 | approx. 31.6 Hz | 1.19% | -68.6 dB |
| 2 kS/s | 2,043.217 S/s | 32 | 63.851 Hz | 1.123% | -69.504 dB |
| 4 kS/s | 4,183 S/s | 32 | approx. 130.7 Hz | approx. 1.12% | -67.0 dB |

The 2 kS/s capture reported 1024 FFT points and bin spacing approximately 1.9953 Hz. The peak at bin 32 corresponds to 63.851 Hz. The test-tone frequency scales with sample rate because the DAC waveform uses 32 cycles per 1024-sample capture.

The FFT uses software-paced ADC acquisition. It is not DMA-timed acquisition.

## Three-channel PWM

- Serial command parser: passed (help, start, stop, frequency and mode commands).
- SPWM, THI and six-step selection and TFT status rendering: passed.
- Tested settings: 10 kHz carrier, 5 Hz electrical/reference frequency, 80% modulation amplitude.
- Example displayed duty references:
  - SPWM: A/B/C = 21.7% / 39.7% / 88.6%.
  - THI: A/B/C = 59.2% / 79.2% / 20.4%.
  - Six-step: A/B/C = 10% / 50% / 90%.
- Screenshots verify firmware mode and reference duty values. GPIO16 was subsequently measured by PCNT; GPIO17 and GPIO4 were not separately instrumented.

Outputs are 3.3 V logic-level signals only. There are no complementary outputs, dead-time insertion, gate drivers, or motor power stage.

## Harmonic comparison v0.5.1

Analysis uses 1020 samples per cycle at modulation index 0.8. Output is ideal line-to-line Vab reference, not physical inverter or motor voltage/current.

| Result | SPWM | THI | Six-step |
|---|---:|---:|---:|
| Fundamental amplitude (normalised reference units) | 1.3856406 | 1.6000000 | 1.5278898 |
| THD H2-H15 | 0.000010% | 0.000010% | 27.313402% |
| THD through Nyquist | 0.028614% | 0.000000% | 31.083647% |

THI phase-leg injected third harmonic: -15.563 dBc. Triplen common-mode cancels from ideal line-to-line Vab. The chart displays orders 1–15; THD through Nyquist includes higher orders up to the sampled waveform's Nyquist limit.

## PCNT frequency verification v0.6.0

A one-second measurement gate was used. GPIO16 generated LEDC PWM and GPIO35 received it through a jumper wire.

| Requested frequency | Measured range in supplied log | Maximum absolute reported error |
|---:|---:|---:|
| 5 kHz | 5,000.000–5,000.620 Hz | 0.01239% |
| 10 kHz | 9,999.000–10,000.070 Hz | 0.01000% |
| 20 kHz | 19,999.000–20,001.000 Hz | 0.00500% |

At a one-second gate, one-count quantisation corresponds to approximately 1 Hz. An initial command attempt included a terminal escape sequence and was rejected; the subsequent clean 20 kHz command was accepted and measured.

## Scope and completion

All seven firmware environments have been built and exercised by the user to the extent described above. Strongest direct hardware validations are TFT display, DAC/ADC loopback, software-paced FFT loopback, logic-level PWM control and PCNT frequency measurement. Harmonic comparison is a firmware calculation on ideal reference waveforms, not a physical inverter measurement.
