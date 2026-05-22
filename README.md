# DIY Paragliding Variometer + Altimeter (ESP32)

A compact, open-source flight instrument for paragliding built on ESP32.
This project reads pressure/temperature from an **MS5611**, computes:

- **Relative altitude** (meters)
- **Vertical speed / vario** (m/s)

…and presents data on a **128x64 SSD1306 OLED** while generating classic vario beeps on a passive buzzer.

## Features

- Fast sensor + vario loop (target ~80 Hz)
- Smoothed altitude and responsive climb/sink audio
- SSD1306 live HUD for ALT, VAR, TEMP, and pressure
- Startup pressure calibration for relative altitude zeroing
- Serial debug telemetry

## Hardware

- ESP32 development board
- MS5611 barometric sensor (I2C)
- SSD1306 OLED display, 128x64 (I2C, address `0x3C`)
- Passive piezo buzzer
- Jumper wires + power source

## Pin Map

### I2C bus

| ESP32 pin | Connected to |
|---|---|
| GPIO21 (`SDA`) | MS5611 SDA, SSD1306 SDA |
| GPIO22 (`SCL`) | MS5611 SCL, SSD1306 SCL |
| 3V3 | MS5611 VCC, SSD1306 VCC |
| GND | MS5611 GND, SSD1306 GND |

### Audio

| ESP32 pin | Connected to |
|---|---|
| GPIO25 | Passive buzzer (+) |
| GND | Passive buzzer (-) |

## Build / Flash

1. Open the project in Arduino IDE (or PlatformIO with equivalent libs).
2. Install required libraries:
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
   - `MS5611` (compatible I2C library)
3. Select your ESP32 board and port.
4. Build and upload `DIY_Paragliding_Vario.ino`.

## Runtime Notes

- On boot, the firmware performs a short calibration window (~1.2 s) and sets current pressure as altitude reference (`0 m`).
- Vario audio behavior:
  - **Climb**: pulsed beeps with pitch/rate increasing with climb rate
  - **Near-zero / mild sink**: silent
  - **Strong sink**: continuous low tone

## Repository Layout

- `DIY_Paragliding_Vario.ino` — main firmware
- `VarioMath.h/.cpp` — altitude and vario math helpers

## License

This project is licensed under the GPL-3.0 License. See `LICENSE`.
