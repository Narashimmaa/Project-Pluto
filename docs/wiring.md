# Wiring (ESP32 NodeMCU-32S)

| Function | GPIO |
|---|---|
| MPU6050 SDA / SCL | 21 / 22 |
| Motor A R_EN / L_EN | 16 / 17 |
| Motor A RPWM / LPWM | 25 / 26 |
| Motor B R_EN / L_EN | 18 / 19 |
| Motor B RPWM / LPWM | 27 / 32 |

## Power
- MPU6050 VCC -> 3.3V, GND -> GND, **AD0 -> GND** (fixes address at 0x68).
- Battery (3S Li-ion, 11.1 V) -> both BTS7960 B+ / B-.
- ESP32 powered from USB while testing.
- **All grounds common**: ESP32, both BTS7960, battery negative.
- BTS7960 R_EN and L_EN go to the GPIOs above.

## Known issues and fixes
- **MPU6050 drops off the bus when motors run:** solder a 0.1 uF ceramic capacitor across each motor's terminals; keep SDA/SCL short and away from motor wires; consider 4.7 kOhm pull-ups.
- **"invalid header: 0xffffffff" boot loop:** a failed upload. Lower upload speed to 115200, use a short data cable, hold BOOT during upload, try a 10 uF capacitor from EN to GND.
- **I2C address flips 0x68/0x69:** AD0 was floating. Tie it to GND.
- **WHO_AM_I reads 0x70:** clone chip (MPU6500-type). The raw-register code works with it.
