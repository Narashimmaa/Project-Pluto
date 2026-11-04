# Self-Balancing Desk Robot

Two-wheel self-balancing robot built on an ESP32 (NodeMCU-32S), an MPU6050 IMU and two BTS7960 motor drivers. Planned extension: a Raspberry Pi 4B + webcam for face recognition and conversation, sending simple drive commands to the ESP32 over USB serial.

Created by Amar.

## Hardware
- ESP32 NodeMCU-32S
- MPU6050 (clone, WHO_AM_I 0x70)
- 2x BTS7960 motor drivers
- 2x 12 V 300 RPM metal-gear DC motors (no encoders)
- 3S Li-ion pack (11.1 V) with BMS

## Repository layout
```
firmware/balance_esp32/   main balancing sketch
tools/i2c_scanner/        confirm the MPU6050 address
tools/mpu_raw_test/       check sensor data and tilt axis
tools/motor_test/         spin each motor, check direction
docs/wiring.md            pinout, power and known issues
```

## Build order
1. Flash `tools/i2c_scanner`. Expect 0x68 (AD0 to GND).
2. Flash `tools/mpu_raw_test`. Flat board: one axis near 8192. Tilt forward and back and confirm the angle moves smoothly.
3. Flash `tools/motor_test` with wheels off the ground. Note which motor needs its direction flipped.
4. Flash `firmware/balance_esp32`:
   - Hold the robot at its balance pose, read the angle on Serial, and set `targetAngle`.
   - Set `DIR_A` / `DIR_B` so both wheels drive toward the direction of the fall.
   - Tune `Kp`, then `Kd`, then a small `Ki`.

## Safety features in the firmware
- Drivers disabled until the MPU6050 is found and the gyro calibrated.
- Motors stay off until the robot is held within `ARM_WINDOW` of `targetAngle`.
- Any run of 5 failed sensor reads cuts the motors and disarms.
- Motors cut when the tilt exceeds `FALL_ANGLE`.

## Requirements
Arduino IDE with **ESP32 Arduino core 3.x** (uses `ledcAttach`). Board: "ESP32 Dev Module" or "NodeMCU-32S".
