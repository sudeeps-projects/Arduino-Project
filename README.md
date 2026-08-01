# SilentSleep

Snore detector for the Arduino UNO R4 WiFi. Listens for snoring and answers with a
gentle vibration, prompting the sleeper to shift position. Everything runs on the
board — no phone, no cloud required, no audio ever leaves the device.

IEEE Arduino Contest 2026 — *Smart Solutions to Daily Challenges*.

## Folders

| Folder | What it is |
|---|---|
| `ieee_arduino_contest_2026_silentSleep/` | **Main build.** Detector + TFT + haptics. No network. 31% flash, 43% RAM. |
| `ieee_arduino_contest_2026_silentSleep_cloud/` | Same detector, plus Arduino IoT Cloud. Publishes `snoreDetected`; shows link state on the LED matrix and TFT. |
| `sudeeps-prototype/` | Early bring-up sketches — motor, I²C, mic, TFT tested one at a time. |

Detection logic is identical in both contest builds.

## Hardware

| Part | Connection |
|---|---|
| Arduino UNO R4 WiFi | — |
| MAX9814 mic amplifier | `OUT` → **A1**, VDD → 5 V |
| DRV2605L haptic driver | I²C — SDA `A4`, SCL `A5` (addr 0x5A) |
| Coin ERM vibration motor | DRV2605L output |
| 1.54" 240×240 ST7789 TFT | CS `10`, DC `9`, RST `8`, SCK `13`, MOSI `11` |

Wiring reference: `SilentSleepSchematic.fzz` (Fritzing).

## How it decides

A rhythm is not evidence — a ticking clock has perfect rhythm. Each 100 ms slice is
run through a 256-point FFT, and the test is **tonality**: what share of the energy
sits in a single pitch. Snoring is turbulent air, so it spreads. Voices and ringtones
are notes, so they stack.

Per-slice readings are too noisy to threshold, so tonality is **averaged over a whole
breath**. Measured on this hardware:

| Sound | Breath-mean tonality |
|---|---|
| Snoring | **16–29%** |
| iPhone ringtone | 47–49% |
| Whistle | 57% |
| Speech | 59% |

A breath must clear four tests — loud and dense, a breathing-shaped envelope,
0.6–3.5 s long, and tonality ≤ 38%. Three consecutive breaths trigger the motor.

## Known limitation

Speech over a running fan measures ~24%, inside the snoring range. The broadband fan
noise dilutes the voice's tonality. No rule tested separates the two, so this case is
unsolved rather than hidden.

## Build

Arduino IDE, board **Arduino UNO R4 WiFi**. Libraries: `Adafruit_DRV2605`,
`Adafruit_GFX`, `Adafruit_ST7789`. Serial Monitor at **9600**.

The `_cloud` build additionally needs `ArduinoIoTCloud`, `Arduino_ConnectionHandler`
and `Arduino_NetworkConfigurator` — easiest to open it in the Arduino Cloud editor,
which supplies them. WiFi credentials are provisioned over BLE/Serial and stored on
the board, so none appear in this repository.
