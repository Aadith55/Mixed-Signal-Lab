# MixedSignalLab

**ESP32-WROOM-32 Mixed-Signal Instrumentation and Analysis Platform**

MixedSignalLab is a PlatformIO project built around the classic ESP32-WROOM-32 and a 2.4-inch ILI9341 SPI TFT. It combines analog loopback characterization, FFT-based signal analysis, three-channel logic-level PWM generation, reference-waveform harmonic analysis, and hardware pulse counting.

The project was developed and tested incrementally using an ESP32 DevKit, an ILI9341 TFT, and jumper wires. It does not require external sensors, a motor, a gate driver, or a custom PCB.

> **Scope note:** This is an educational instrumentation/firmware prototype. Some experiments are direct hardware measurements; harmonic comparison is a numerical analysis of ideal reference waveforms. See [Test results and limitations](#test-results-and-limitations).

## Features

Seven independent PlatformIO firmware environments are provided:

| Environment | Purpose |
|---|---|
| `tft_hello` | Verify TFT_eSPI, ILI9341, SPI, orientation and basic rendering |
| `dac_adc_loopback` | Sweep the ESP32 DAC and read back through the ADC |
| `adc_characterization` | 256-code DAC-to-ADC characterization, noise statistics, loopback INL/DNL, inverse LUT and TFT error plot |
| `spectrum_analyzer` | 1024-point Hann-windowed FFT, single-sided spectrum, harmonic estimate, THD estimate and serial CSV |
| `pwm_generator` | Three independent 10-bit LEDC PWM outputs; sinusoidal, third-harmonic-injection, six-step and static-duty modes |
| `harmonic_comparison` | Compare SPWM, THI and six-step line-to-line reference waveforms and calculate harmonic amplitudes/THD |
| `pcnt_frequency` | Generate a PWM signal and measure its frequency using the ESP32 PCNT peripheral |

Each environment compiles only `src/main.cpp` plus the selected module. The firmware experiments are separate builds, not a single run-time multi-mode application. Select the required environment in PlatformIO Project Tasks and upload that environment.

## Hardware

- Microcontroller: classic ESP32-WROOM-32 (PlatformIO board ID: `esp32dev`)
- Display: 2.4-inch LCDWIKI MSP2402 / ILI9341 SPI TFT, 240 x 320 native orientation
- Framework: Arduino framework on Espressif32 platform
- Build system: PlatformIO
- Display library: Bodmer TFT_eSPI 2.5.43

### Pin assignment

| Function | ESP32 GPIO |
|---|---:|
| TFT SCK | 18 |
| TFT MOSI | 23 |
| TFT MISO | 19 |
| TFT CS | 33 |
| TFT DC | 21 |
| TFT RESET | 22 |
| TFT VCC | V5 (the tested LCDWIKI module accepts 3.3–5 V supply) |
| TFT GND | GND |
| TFT LED/backlight | 3V3 |
| DAC1 output | 25 |
| ADC1_CH6 input | 34 |
| PWM channel A | 16 |
| PWM channel B | 17 |
| PWM channel C | 4 |
| PCNT input | 35 |

GPIO25 is connected to GPIO34 for DAC-to-ADC loopback. GPIO16 is connected to GPIO35 for PCNT frequency measurement. The two jumpers are required only for their respective experiments. The TFT wiring remains connected for all display-based environments.

**Electrical cautions**

- ESP32 GPIO signals use 3.3 V logic. Do not apply 5 V logic to ESP32 inputs.
- The particular LCDWIKI MSP2402 module used in testing accepted V5 at VCC; verify the exact board variant before using 5 V on another module.
- GPIO35 is input-only and has no internal pull-up or pull-down. Connect GPIO16 to GPIO35 for the PCNT experiment.
- PWM outputs are logic-level signals only. Do not connect them directly to a motor, power stage, MOSFET gate, or inductive load.
- No complementary PWM pairs, dead-time insertion, isolation, gate driver, over-current protection, or power stage is implemented.

## Build environment

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Install the **PlatformIO IDE** extension.
3. Extract this repository and open the folder containing `platformio.ini`.
4. PlatformIO installs the Espressif32 toolchain and TFT_eSPI dependency on the first build.
5. Select an environment under **PlatformIO: Project Tasks**.
6. Click **Build**. After a successful build, click **Upload**.
7. Open **Monitor** at 115200 baud.

The PlatformIO configuration pins the platform to `espressif32@6.9.0`, uses the Arduino framework, and declares `bodmer/TFT_eSPI@2.5.43` in `lib_deps`. TFT_eSPI pin and driver configuration is supplied using build flags.

The default environment is `tft_hello`, selected as a safe first test.

## Experiment procedures

### 1. TFT hello-world

Build and upload `env:tft_hello`. The display should show MixedSignalLab, ILI9341/SPI information, resolution and uptime. The serial monitor reports TFT initialization.

### 2. DAC-to-ADC loopback

Connect GPIO25 (DAC1) directly to GPIO34 (ADC1_CH6). Build and upload `env:dac_adc_loopback`. Open Serial Monitor at 115200 baud and send `r` to run a 0–255 DAC sweep. The firmware prints `dac_code, adc_mean, adc_sigma, adc_min, adc_max`.

### 3. ADC loopback characterization

Keep GPIO25 connected to GPIO34. Build and upload `env:adc_characterization`. Send `r` to acquire 64 ADC readings per DAC code over all 256 DAC codes. Send `p` to redraw the results plot and `h` for help. CSV is bracketed by `ADC_CHAR_CSV_BEGIN` and `ADC_CHAR_CSV_END`.

The firmware computes:
- Mean, standard deviation, minimum and maximum per DAC code.
- Endpoint-referenced combined-loopback INL and step-based DNL.
- A 256-entry inverse lookup table (LUT).
- Before/after correction residual error estimates.
- A TFT graph with uncorrected and LUT-corrected residuals.

The LUT is held in RAM and is lost after reset. The reported after-LUT error is calculated from the same calibration data used to build the LUT; it is not independent validation.

### 4. FFT spectrum analyser

Keep GPIO25 connected to GPIO34. Build and upload `env:spectrum_analyzer`. Commands over Serial Monitor:

| Command | Action |
|---|---|
| `r` | Capture 1024 samples, compute FFT, draw spectrum and print CSV |
| `1` | Request 1 kS/s |
| `2` | Request 2 kS/s |
| `4` | Request 4 kS/s |
| `p` | Redraw last spectrum |
| `h` | Help |

The test signal is generated using the ESP32 DAC and looped back to the ADC. The firmware uses software-paced sampling, removes DC, applies a Hann window, and computes a radix-2 FFT. It reports the actual average sample rate measured over the acquisition. The test tone is located at FFT bin 32, so its frequency scales with the sample rate.

This is not DMA-timed acquisition. The measured sample interval and rate are affected by software execution and peripheral conversion time. The reported THD estimate is based on harmonics 2–15 where they lie below Nyquist; it is not a calibrated instrument-grade SINAD/ENOB test.

### 5. Three-channel PWM generator

Build and upload `env:pwm_generator`. All outputs start stopped at 0% duty. The serial command parser accepts LF, CR and CRLF line endings.

| Command | Action |
|---|---|
| `h` | Help |
| `r` | Start modulation |
| `x` | Stop all channels (0% duty) |
| `f <Hz>` | Carrier frequency, 1000–20000 Hz |
| `e <Hz>` | Electrical/reference frequency, 0.1–20 Hz |
| `a <percent>` | Modulation amplitude, 0–100% |
| `m sine` | Sinusoidal PWM references |
| `m thi` | Third-harmonic-injection references |
| `m six` | Six-step 60-degree sector references |
| `m static` | Fixed independent channel duties |
| `d <ch> <percent>` | Set static duty for channel 1, 2 or 3 |
| `p` | Redraw TFT |
| `?` | Print current status |

The implementation uses three LEDC channels on GPIO16, GPIO17 and GPIO4, with 10-bit duty resolution. The control firmware updates the reference duty values at a 1 kHz software rate. These are single-ended 3.3 V logic-level outputs, not motor-drive signals.

### 6. Harmonic comparison

Build and upload `env:harmonic_comparison`. The firmware computes all three references automatically at startup. Commands: `c` to recalculate and print CSV, `p` to redraw the chart, `h` for help.

The comparison uses 1020 samples per electrical cycle (170 samples per 60-degree sector) and analyses ideal line-to-line `Vab_reference` for SPWM, THI and six-step. The chart shows harmonics 1–15. The serial report provides both H2–H15 THD and THD through Nyquist. The injected THI third-harmonic phase-leg component is also reported separately; triplen common-mode content cancels in ideal line-to-line subtraction.

**This module analyses mathematical reference waveforms. It does not sample the physical PWM pins, an inverter output, motor phase voltage, or motor current.**

### 7. PCNT frequency verification

Connect GPIO16 to GPIO35 with one jumper. Build and upload `env:pcnt_frequency`. The firmware configures LEDC on GPIO16 and PCNT rising-edge counting on GPIO35.

Commands: `h` help, `r` start, `x` stop, `f <Hz>` set 1000–20000 Hz, `d <percent>` set 10–90% duty, `p` redraw TFT, `?` status. The measurement gate is approximately one second; the firmware calculates frequency using the measured gate duration and prints a CSV-like line beginning with `PCNT_MEASUREMENT`.

## Mathematical notes

### ADC loopback metrics

The characterization uses an endpoint line through the measured means at DAC codes 0 and 255:

`idealStep = (mean[255] - mean[0]) / 255`

For code `k`, the endpoint-referenced loopback INL is:

`INL[k] = (mean[k] - (mean[0] + k * idealStep)) / idealStep`

For codes after the first, step-based loopback DNL is:

`DNL[k] = ((mean[k] - mean[k-1]) / idealStep) - 1`

These are **combined DAC-to-wire-to-ADC loopback metrics**. They do not isolate the ADC's intrinsic INL/DNL because the on-chip DAC also contributes nonlinearity.

### Noise sigma

For N raw ADC reads at a fixed DAC code:

`sigma = sqrt(mean(x^2) - mean(x)^2)`

The implementation discards a few readings after changing the DAC output, then collects the configured number of samples.

### FFT frequency resolution

For N = 1024 samples and measured sampling rate Fs:

`frequency_resolution = Fs / N`

The frequency corresponding to FFT bin k is:

`f[k] = k * Fs / N`

The firmware removes the sample mean, applies a Hann window, then executes an iterative radix-2 Cooley–Tukey FFT. Magnitudes are scaled for a single-sided amplitude spectrum using the Hann coherent-gain sum.

### THD

For a fundamental amplitude A1 and harmonic amplitudes A2...An:

`THD(%) = 100 * sqrt(A2^2 + A3^2 + ... + An^2) / A1`

The harmonic-comparison module reports two ranges: H2–H15 (matching the on-screen chart) and all non-fundamental DFT content through Nyquist (using Parseval's relation for the sampled reference). Harmonic comparison results are for calculated reference waveforms.

### ENOB

For a dynamic ADC measurement, ENOB should be derived from SINAD:

`ENOB = (SINAD_dB - 1.76) / 6.02`

A static DAC sweep's noise sigma alone is not a valid substitute for dynamic SINAD. MixedSignalLab's current static loopback characterization does not claim an ADC ENOB result.

## Test results and limitations

User-side test results, including the supplied PCNT measurement logs and FFT/harmonic CSV results, are recorded in [TEST_RESULTS.md](TEST_RESULTS.md).

Key outcomes reported from the tested setup:
- TFT initialization and DAC-to-ADC loopback: passed.
- ADC loopback characterization: 256 DAC codes, 64 reads/code; average sigma 7.27 ADC counts; maximum sigma 18.84 counts; maximum absolute loopback INL 22.08 LSB; RMS loopback INL 15.08 LSB.
- Calculated inverse-LUT residual RMS error reduced from 15.081 to 0.340 loopback LSB on the calibration dataset; independent validation remains outstanding.
- FFT at a requested 2 kS/s: measured 2.043217 kS/s; peak bin 32 at 63.851 Hz; reported THD 1.123%; relative noise-floor estimate -69.504 dB.
- PWM modes SPWM, THI and six-step: command handling and TFT duty-reference display passed. Physical output measurements were not performed on all three PWM channels.
- Harmonic comparison: SPWM, THI and six-step reference-waveform calculations completed; six-step THD through Nyquist 31.083647% at modulation index 0.8.
- PCNT: GPIO16-to-GPIO35 loopback at 5, 10 and 20 kHz; maximum reported absolute error 0.01239%, 0.01000% and 0.00500%, respectively.

The results are specific to the supplied board, wiring, firmware version and test conditions. They should not be treated as calibrated instrument specifications.

## Repository contents

```text
Mixed-Signal-Lab/
├── include/                 # Module interfaces
├── src/
│   ├── main.cpp             # Environment-based entry point
│   ├── hello_screen.cpp
│   ├── loopback_test.cpp
│   ├── adc_characterization.cpp
│   ├── spectrum_analyzer.cpp
│   ├── pwm_generator.cpp
│   ├── harmonic_comparison.cpp
│   └── pcnt_frequency.cpp
├── platformio.ini
├── TEST_RESULTS.md
├── CHANGELOG.md
├── VERSION
└── .gitignore
```

## License

No license has been declared yet. Add a LICENSE file before granting others explicit reuse rights.
