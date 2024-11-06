# Vethram

Vethram is a fall prediction hardware solution designed for elderly individuals. It utilizes an ESP32, ESP8266, and an MPU6050 IMU to predict falls based on a pre-trained ML model.

## Current Progress

- **Slave Device Implementation**: Firmware for slave devices has been sketched and implemented.
- **Master Device (USB Device)**: Master device setup with UDP functionality completed.
- **Processing Simulation**: Simulation and testing in the Processing environment are complete.
- **Multi-Slave Support**: Both the master device and processing script support multiple slave devices.

## Building Instructions

1. Download the source code from this repository.
2. Flash the `.ino` files onto the respective boards:
   - **Master Device**: Use `ESP32 DEV KIT V1`.
   - **Slave Devices**: Use `ESP8266 01`.
3. Connect the master device to your PC.
4. Run the processing script to initialize communication and testing.

## To-Do List

- **Wireless Master**: Implement TCP communication for the master device.
- **Processing TCP Receive**: Update the processing script to support TCP-based data reception.
- **Python Connection**: Establish TCP connections and data reception in Python.
- **GUI Integration**: Implement GUI to display data and predictions.
- **ML Model Integration**: Integrate and test the pre-trained ML model for fall prediction.

---
