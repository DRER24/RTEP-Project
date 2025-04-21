# Radar Target Tracking System

## Overview

This project implements a real-time radar target tracking system using a Raspberry Pi. The system detects targets using a radar sensor and automatically tracks them by controlling servo motors for pan and tilt movements.

The system has evolved from earlier versions: previous implementations relied solely on mmWave technology, which provided a strong foundation with near-perfect visualization and sensing ability. The current version integrates servo motors with the radar system, which presents different trade-offs. While point cloud data capture and visualization is more challenging with the servo implementation, the system can now detect human presence more swiftly and easily through physical tracking.

## Features

- Real-time radar data processing
- Automatic target detection and tracking
- Servo motor control for physical tracking
- Kalman filtering for smooth motion
- Multi-threaded architecture for responsive performance
- Signal handling for graceful shutdown

## Hardware Requirements

- Raspberry Pi (tested on Raspberry Pi 5)
- MMwave Radar sensor HLK-LD-6001 (connected via UART/Serial)
- PCA9685 PWM controller
- Servo motors for pan/tilt mechanism
- I2C connection for servo control
- Jumper wires and breadboard for prototyping
- Power supply (5V for Raspberry Pi, may need separate supply for servos depending on power requirements)

## Software Architecture

The system is built with a modular, object-oriented architecture following SOLID principles:

- **Target**: Represents a detected radar target with position and angle information
- **RadarData**: Manages radar frame data and target extraction
- **KalmanFilter**: Implements a 1D Kalman filter for smoothing servo movements
- **ServoController**: Controls servo motors via the PCA9685 PWM controller
- **SerialComm**: Handles communication with the radar sensor
- **SignalHandler**: Manages system signals for graceful shutdown
- **RadarSystem**: Coordinates all components into a complete system

## Detailed Installation and Setup

### Prerequisites

1. A Raspberry Pi with Raspberry Pi OS installed (Bullseye or newer recommended)
2. Required packages:
   ```bash
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git i2c-tools
   ```

3. Enable I2C interface:
   ```bash
   sudo raspi-config
   ```
   Navigate to "Interface Options" → "I2C" → "Yes" to enable the I2C interface

4. Verify I2C is working:
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see your PCA9685 address (default is 0x40)

5. Enable Serial interface (if not using USB):
   ```bash
   sudo raspi-config
   ```
   Navigate to "Interface Options" → "Serial Port" → Disable login shell, but enable serial hardware

### Hardware Setup

1. Connect the PCA9685 to Raspberry Pi:
   - VCC to 3.3V or 5V (check your PCA9685 specifications)
   - GND to GND
   - SCL to BCM 3 (SCL pin)
   - SDA to BCM 2 (SDA pin)

2. Connect servo motors to PCA9685:
   - Yaw servo to channel 0
   - Pitch servo to channel 1
   - Connect servo power as per your servo requirements

3. Connect radar module:
   - TX pin to Raspberry Pi RX
   - RX pin to Raspberry Pi TX
   - GND to GND
   - VCC to appropriate voltage (check your radar specifications)

### Software Installation

1. Clone the repository:
   ```bash
   git clone (https://github.com/DRER24/RTEP-Project/radar_tracking.git)
   cd radar_tracking
   ```

2. Create build directory and build the project:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

3. Install the executable:
   ```bash
   sudo make install
   ```

4. Verify installation:
   ```bash
   which radar_tracking
   ```
   This should output `/usr/local/bin/radar_tracking`

### Configuration

By default, the system uses the following configuration:
- Radar serial port: `/dev/ttyAMA0`
- Radar baud rate: 9600
- I2C device: `/dev/i2c-1`

If your hardware is connected differently, you'll need to provide these parameters when running the program.

## Usage

Run the radar tracking system with:

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

### Troubleshooting

If the system doesn't detect your radar or servos:

1. Check your connections and wiring
2. Verify I2C is enabled and working: `sudo i2cdetect -y 1`
3. Check serial port access permissions: `sudo usermod -a -G dialout $USER`
4. Restart and try again

If your servos are jittering:
1. Ensure you have a stable power supply with sufficient current
2. Check if the servo PWM frequency is appropriate for your servos

## Testing

To build and run tests:

```bash
mkdir build && cd build
cmake -DBUILD_TESTS=ON ..
make
ctest
```

## Performance Notes

The current servo motor implementation prioritizes human presence detection and physical tracking. If you require detailed point cloud visualization similar to the standalone mmWave version, you may need to modify the code to store and process the radar data differently.

## License



## Authors

- Abishek Srinivasan Moorthy (3043860S)
- Derrick Roy Edgar Rajappan (3023903R)
- Dheemanth S Naidu (3049973N)
- Yuxin Du (3008171)
- Zhuolin Li (3021316)
