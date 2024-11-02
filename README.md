# Vethram

Vethram is a fall prediction hardware solution aiming elder people. It predicts the fall based on a pre-trained ML model. It uses ESP32, ESP8266 and MPU6050 IMU at its core.

## Current progress

- Slave devices sketch and implementation done.
- Master device(USB device) done(UDP).
- Processing simulation and testing done.
- Multi slave support for both master and processing script done.

## Building

Download the source code and flash the `ino` files to corresponding boards. For master scripts use `ESP32 DEV KIT V1` and for slave devices use `ESP8266 01`. Connect master to your PC and run the processing script on processing.

## To Do List

- Wireless Master using TCP.
- Processing TCP receive.
- Python connection and TCP receive.
- GUI connections.
- ML Model
