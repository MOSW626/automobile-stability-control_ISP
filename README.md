# Automobile Stability Control

For a Korean version of this document, see [README_KO.md](README_KO.md).

This repository contains code and reference material for a small project that explores
basic stability control using an Arduino, inertial sensors and a motor.  The project logs
sensor values over a serial connection and drives the motor using simple PID loops.

## Directory layout

- `src/` – Arduino sketches (`*.ino`) and Python scripts for acquiring sensor data.
- `pic/` – Pictures and short videos of the hardware setup.
- `preceding_papers/` – Research papers consulted during development.
- `report/` – Posters and project reports.

The subdirectories inside `src` include various experiments.  The main Python scripts
are `src/main.py` and `src/last/last.py`.

## Running the Python scripts

The scripts require Python 3 and the `pyserial` package.  Install it with:

```bash
pip install pyserial
```

### `src/main.py`

This script reads a continuous data stream from an Arduino and stores the values in a
CSV file whose name is based on the current time.  Edit the `port` variable near the
start of the script so that it matches the serial port of your board (e.g. `COM3` on
Windows or `/dev/ttyUSB0` on Linux).  Then run:

```bash
python src/main.py
```

### `src/last/last.py`

`last.py` performs a similar task but expects data formatted as `time,angle,y_velocity`.
Adjust the port passed to `serial.Serial` on line 4, then run:

```bash
python src/last/last.py
```

The received values will be saved to `sensor_data.csv`.

## Building and uploading the Arduino sketches

1. Open the desired `.ino` file in the Arduino IDE (for example
   `src/231031_control/231031_control.ino`).
2. Select the correct board type and serial port from the **Tools** menu.
3. Click **Verify** to build, then **Upload** to flash the program to the board.

The sketches assume an Arduino compatible board with an IMU (MPU6050/MPU9250) and a
motor driver connected to the pins specified in each file.
Make sure the sensor and motor driver are wired according to the pin definitions in the
sketch you choose to upload.

---

This README provides a quick overview of the project structure and explains how to run
the provided Python scripts as well as how to upload the Arduino code.
