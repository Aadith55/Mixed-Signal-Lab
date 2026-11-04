# Changelog

## v1.0.0 — Consolidated release

- Consolidated seven independently selectable PlatformIO environments.
- Retained TFT hello-world and raw DAC/ADC loopback verification.
- Added DAC-to-ADC loopback characterization, noise statistics, loopback INL/DNL, inverse LUT and TFT plots.
- Added 1024-point Hann-windowed FFT spectrum analyser and serial spectrum CSV.
- Added three-channel logic-level PWM generator with SPWM, THI, six-step and static modes.
- Added harmonic comparison for SPWM, THI and six-step reference waveforms.
- Corrected THI normalization and added THD H2–H15 and through-Nyquist metrics.
- Added PCNT frequency verification using GPIO16 PWM output looped back to GPIO35.
- Added measured results and interpretation limits in TEST_RESULTS.md.
- Set the default PlatformIO environment to the safe TFT hello-world test.

See README.md for pin assignments, build instructions, experiment steps and limitations.
