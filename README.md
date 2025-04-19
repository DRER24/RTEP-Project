# RTEP-Project: mmWave Radar System

A real-time radar presence detection and visualization system using the HLK-LD6001 mmWave radar sensor as part of the Real-Time Embedded Programming project.

## Overview

This project implements a real-time human presence detection and visualization system that:

- Interfaces with the HLK-LD6001 mmWave radar sensor via UART
- Processes radar data to detect humans and other objects
- Visualizes presence data as a heat map in the terminal
- Records sensor data for later analysis
- Periodically generates graph files of the accumulated heat map

The system is built with a modular, multithreaded architecture and provides both real-time visualization and data logging capabilities.

## System Requirements

- Linux-based operating system 
- C++17 compatible compiler
- POSIX-compliant system with serial port support
- HLK-LD6001 mmWave radar sensor connected via USB-to-UART adapter

## Hardware Setup

1. Connect the HLK-LD6001 radar sensor to your computer using a USB-to-UART adapter
2. By default, the system expects the radar to be available at `/dev/ttyUSB0`
3. Ensure you have appropriate permissions to access the serial port (usually by adding your user to the `dialout` group)

```bash
# Add your user to the dialout group to access serial ports without sudo
sudo usermod -a -G dialout $USER
# Log out and log back in for the changes to take effect
```

## Installation

### Prerequisites

Install the required development tools and dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake g++ git
sudo apt install libpcl-dev libgpiod-dev
```

### Building from Source

Clone the repository and build the project:

```bash
# Clone the repository
git clone https://github.com/DRER24/RTEP-Project.git
cd RTEP-Project

# Build using CMake
mkdir build
cd build
cmake ..
make

# Alternative: Build using Make directly
make
```

## Usage

### Running the Application

From the build directory, run the application:

```bash
./radar_heatmap
```

The application will:
1. Initialize the radar detector
2. Start the heat map manager
3. Launch the visualization interface
4. Display real-time heat map data in the terminal

### Understanding the Interface

The visualization shows:
- A grid representing physical space around the radar (center marked with "R")
- Detection events marked with different symbols:
  - H: Human
  - M: Multiple People
  - O: Obstacle
  - S: Strong Object
  - D: Something Detected
- Heat map intensity shown with ASCII characters (from low to high): " .:-=+*#%@"
- Color coding from blue (low intensity) to red (high intensity)

The status bar shows:
- Current detection type and strength
- Total number of detection events
- Current position in centimeters
- Running time and graphs generated

### Data Output

The system generates two types of data output:

1. **CSV Data File** (`radar_data.csv`): Contains all detection events with timestamps, type, strength, position, and raw data
2. **Heat Map Graph Files** (`radar_graph_XXX.txt`): Text-based visualizations of accumulated heat map data generated every 10 seconds

### Exiting the Application

Press `Ctrl+C` to gracefully stop the application. This ensures all data is properly saved before exit.

## Configuration

Edit the header files to modify system parameters:

- `radar_detector.h`: Serial port settings and radar configuration
  - `SERIAL_PORT`: Path to the serial device (default: `/dev/ttyUSB0`)
  - `PACKET_SIZE`: Size of radar data packets (22 bytes for HLK-LD6001)

- `heatmap_manager.h`: Heat map visualization parameters
  - `GRID_SIZE`: Size of the heat map grid (default: 20×20)
  - `DECAY_RATE`: Rate at which heat map values decay (default: 0.995)
  - `MAX_INTENSITY`: Maximum intensity value (default: 10.0)
  - `CM_PER_CELL`: Physical scale of each cell in centimeters (default: 25cm)

- `visualizer.h`: Visualization update rate
  - `VIZ_UPDATE_MS`: Visualization refresh rate in milliseconds (default: 500ms)

## Project Structure

- `main.cpp`: Application entry point and component initialization
- `radar_detector.h/cpp`: Interfaces with the radar sensor and processes detection events
- `heatmap_manager.h/cpp`: Manages heat map data, file output, and graph generation
- `visualizer.h/cpp`: Handles terminal-based visualization of the heat map
- `radar_sensor.cpp`: Provides low-level sensor communication functions

## Extending the System

The modular architecture allows for easy extension:
- Add new visualization methods by implementing the `HeatMapVisualizerCallback` interface
- Implement additional data processing by adding listeners to the `RadarDetectorCallback` interface
- Modify the `HeatMapManager` to generate different graph formats or outputs

## Troubleshooting

### Common Issues

1. **Permission denied accessing serial port**
   - Add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`
   - Log out and log back in for the change to take effect

2. **Radar not detected**
   - Check connections to the HLK-LD6001 sensor
   - Verify the correct serial port in `radar_detector.h` (default: `/dev/ttyUSB0`)
   - Try running `ls -l /dev/ttyUSB*` to find available serial ports

3. **Compile errors**
   - Ensure you have a C++17 compatible compiler: `g++ --version`
   - Check that all dependencies are installed

## License

[Include your license information here]

---

## Project Information

This project was developed as part of the Real-Time Embedded Programming (RTEP) course.

### Authors
- Abishek Srinivasan Moorthy (3043860S)
- Derrick Roy Edgar Rajappan (3023903R)
- Dheemanth S Naidu (3049973N)
- Yuxin Du (3008171)
- Zhuolin Li (3021316)
