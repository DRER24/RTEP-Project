# Radar Target Tracking System

## Overview

This project implements a real-time radar target tracking system using a Raspberry Pi. The system detects targets using a radar sensor and automatically tracks them by controlling servo motors for pan and tilt movements.

## Features

- Real-time radar data processing
- Automatic target detection and tracking
- Servo motor control for physical tracking
- Kalman filtering for smooth motion
- Multi-threaded architecture for responsive performance
- Signal handling for graceful shutdown

## Hardware Requirements

- Raspberry Pi (tested on Raspberry Pi 4)
- Radar sensor (connected via UART/Serial)
- PCA9685 PWM controller
- Servo motors for pan/tilt mechanism
- I2C connection for servo control

## Software Architecture

The system is built with a modular, object-oriented architecture following SOLID principles:

- **Target**: Represents a detected radar target with position and angle information
- **RadarData**: Manages radar frame data and target extraction
- **KalmanFilter**: Implements a 1D Kalman filter for smoothing servo movements
- **ServoController**: Controls servo motors via the PCA9685 PWM controller
- **SerialComm**: Handles communication with the radar sensor
- **SignalHandler**: Manages system signals for graceful shutdown
- **RadarSystem**: Coordinates all components into a complete system

## Building and Installation

### Prerequisites

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git
```

### Build

```bash
mkdir build && cd build
cmake ..
make
```

### Install

```bash
sudo make install
```

## Usage

```bash
radar_tracking [radar_port] [radar_baud] [i2c_device]
```

Default values:
- radar_port: /dev/ttyAMA0
- radar_baud: 9600
- i2c_device: /dev/i2c-1

Example:
```bash
radar_tracking /dev/ttyS0 57600 /dev/i2c-0
```

## Testing

To build and run tests:

```bash
mkdir build && cd build
cmake -DBUILD_TESTS=ON ..
make
ctest
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributors

- [Your Name] - Initial implementation
