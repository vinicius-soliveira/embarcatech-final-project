# Digital Guitar Tuner — EmbarcaTech Final Project

A digital guitar tuner developed in C for the **Raspberry Pi Pico W (RP2040)** as the final project of the EmbarcaTech embedded systems program.

The firmware captures audio from a microphone, estimates its dominant frequency using a Fast Fourier Transform (FFT), identifies standard guitar-string notes, and provides tuning guidance through an SSD1306 OLED display and a 5 × 5 addressable LED matrix.

## Features

- Microphone acquisition using the RP2040 ADC and DMA.
- 4,096-sample audio buffers at a nominal 4 kS/s sampling rate.
- Hann-window preprocessing and a recursive radix-2 FFT implemented in C.
- Dominant-frequency estimation from the FFT magnitude spectrum.
- Detection of the six standard guitar tuning notes (E2–E4).
- OLED instructions for tightening or loosening a string.
- LED feedback for low, high, and in-tune conditions.
- Button interrupt with software debouncing and an inactivity timeout.

## Hardware

The firmware targets the Raspberry Pi Pico W and uses the following peripherals:

| Component | Interface / GPIO | Purpose |
| --- | --- | --- |
| Microphone module | ADC2 / GP28 | Audio input |
| SSD1306 OLED | I²C1; SDA GP14, SCL GP15 | Note and tuning instructions |
| 25-pixel WS2812-compatible LED matrix | PIO / GP7 | Visual tuning indication |
| Push button A | GP5 | Restart detection after timeout |

Hardware connections should be checked against the specific board and microphone module used. The microphone signal must remain within the RP2040 ADC input range.

## Signal Processing

The firmware follows this processing sequence:

```text
Microphone
    |
    v
RP2040 ADC --> DMA --> 4,096-sample buffer
                         |
                         v
                   ADC conversion
                         |
                         v
                    Hann window
                         |
                         v
                    4,096-point FFT
                         |
                         v
                 Magnitude / peak bin
                         |
                         v
                  Note identification
                         |
                         v
                  Tuning comparison
                         |
                   +-----+-----+
                   |           |
                   v           v
                 OLED      LED matrix
```

The nominal sampling rate is **4,000 samples/s**, with an FFT-bin spacing of approximately **0.977 Hz**. The peak detector selects the largest magnitude in the nonnegative-frequency half of the FFT and converts its bin index to frequency. It does not perform interpolation between bins or harmonic-aware fundamental-frequency estimation.

The note-identification logic accepts frequencies between 75 and 350 Hz and compares them with standard tuning references using a relative-frequency tolerance. After a string is identified, the firmware classifies its measured frequency as low, high, or tuned. The in-tune comparison uses a 0.5 Hz threshold in the source code; actual accuracy depends on sampling, signal conditions, and peak estimation.

### Standard tuning references

| String | Note | Frequency |
| --- | --- | ---: |
| 6 | E2 | 82.41 Hz |
| 5 | A2 | 110.00 Hz |
| 4 | D3 | 146.83 Hz |
| 3 | G3 | 196.00 Hz |
| 2 | B3 | 246.94 Hz |
| 1 | E4 | 329.63 Hz |

## Application Flow

The application is organized as a state machine with the following states:

- `WELCOME` — displays the startup screen.
- `DETECTING` — captures and analyzes audio to identify a guitar string.
- `TUNING` — compares the detected frequency with the selected reference.
- `LOW` / `HIGH` — asks the user to tighten or loosen the string.
- `CORRECT` — indicates that the string is tuned.
- `IDLE` — waits for button input after the detection timeout.

A timer limits the initial detection interval to approximately 20 seconds. Pressing button A while idle restarts the detection cycle.

## Building

### Requirements

- Raspberry Pi Pico SDK **1.3.0 or newer**
- ARM GNU toolchain (`arm-none-eabi-gcc`)
- CMake and a compatible build tool
- Raspberry Pi Pico W (or compatible RP2040 hardware with equivalent wiring)

Make sure the `PICO_SDK_PATH` environment variable points to your local Pico SDK installation.

```bash
git clone https://github.com/vinicius-soliveira/embarcatech-final-project.git
cd embarcatech-final-project

export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build
cmake --build build -j
```

**Note:** The root `CMakeLists.txt` currently specifies `/usr/bin/arm-none-eabi-gcc` explicitly. Adjust that path if your ARM toolchain is installed elsewhere.

The build generates a UF2 image under `build/src/` (normally `final_project.uf2`). To flash it, connect the Pico W in BOOTSEL mode and copy the UF2 file to its mounted storage device.

## Repository Structure

```text
.
├── CMakeLists.txt          # SDK and top-level build configuration
├── pico_sdk_import.cmake   # Pico SDK integration
├── src/
│   ├── CMakeLists.txt      # Firmware target and linked hardware libraries
│   └── main.c              # Application, sampling, DSP, and state machine
└── inc/
    ├── bitmap.h           # Bitmap data
    ├── neopixel.c         # Addressable LED driver
    ├── ws2818b.pio         # PIO program for LED communication
    └── ssd1306/           # OLED display driver
```

## Technical Notes

This project demonstrates embedded C programming, peripheral integration, DMA-based sampling, FFT-based signal analysis, event-driven input, and user feedback on resource-constrained microcontroller hardware.

The FFT implementation and peak-bin selection are educational and could be improved with lower-stack-memory processing, spectral peak interpolation, and fundamental-frequency detection for signals with strong harmonics.

## Author

[Vinicius Oliveira](https://github.com/vinicius-soliveira)
