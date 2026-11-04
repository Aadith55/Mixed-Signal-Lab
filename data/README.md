# Experimental data captures

This folder stores compressed serial captures supplied during board validation.

| File | Description |
|---|---|
| `adc_characterization.csv.gz` | Full 256-code DAC-to-ADC characterization capture, including means, sigma, extrema, loopback INL/DNL, LUT code and calculated correction residual. |
| `spectrum_2ksps.csv.gz` | Full 1024-point FFT single-sided spectrum capture from the 2 kS/s requested-rate run, including metadata and bins 0–512. |

Both files are gzip-compressed plain-text serial captures. Decompress with `gzip -dk <file>` or a desktop archive utility.

The original capture markers (for example, `ADC_CHAR_CSV_BEGIN/END` and `SPECTRUM_CSV_BEGIN/END`) are retained. See the repository root `TEST_RESULTS.md` for a summary and the measurement qualifications.
