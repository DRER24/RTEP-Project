# Radar HeatMap Manager

A C++ library for real-time visualization of radar detection data using heatmaps and ASCII graphs.

## Overview

This library processes detection events from radar sensors (specifically designed for the HLK-LD6001 radar detector) and generates:

1. Real-time heatmaps for visualization
2. Text-based heatmap graphs saved to files at regular intervals
3. CSV data logging of all detection events

The system visualizes detection events with different intensities based on the type and strength of detection, implements heat diffusion to neighboring cells, and applies automatic decay over time.

## Features

- Real-time heatmap generation and visualization
- Detection event categorization (Human, Multiple People, Obstacle, Strong Object)
- Configurable grid size and physical scale (default: 20x20 grid, 25cm per cell)
- Automatic decay of heat intensity over time
- Text-based graph generation with:
  - Physical scale markers (cm)
  - Intensity legend
  - Radar position marker
  - Detection type indicators
- CSV data logging with timestamps

## Dependencies

- C++11 or higher
- `radar_detector.h` - The RadarDetector interface (not included in this repository)
- Standard library components: vectors, threads, mutex, chrono, fstream

## Installation

### Prerequisites

- C++11 compatible compiler (GCC, Clang, MSVC)
- CMake 3.10 or higher (for building with CMake)

### Building with CMake

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Basic Integration

```cpp
#include "heatmap_manager.h"

// Create a heatmap manager with custom output files
HeatMapManager heatmap("data_output.csv", "heatmap_graph_");

// Start the manager
if (heatmap.start()) {
    // Register with your radar detector
    yourRadarDetector.registerCallback(&heatmap);
    
    // Run your application...
    
    // When finished
    heatmap.stop();
}
```

### Visualization

To visualize the heatmap in real-time, implement the `HeatMapVisualizerCallback` interface:

```cpp
class YourVisualizer : public HeatMapVisualizerCallback {
public:
    void onHeatMapUpdated() override {
        auto heatmap = heatmapManager.getHeatMap();
        // Update your visualization...
    }
    
    void onNewGraphAvailable(const std::string& filename) override {
        // Process new graph file...
    }
};

// Register your visualizer
YourVisualizer visualizer;
heatmapManager.registerCallback(&visualizer);
```

## Configuration

You can modify the following parameters in `heatmap_manager.h`:

```cpp
// Heat map parameters
#define GRID_SIZE 20       // Grid dimensions (20x20)
#define DECAY_RATE 0.995   // Heat decay rate per cycle
#define MAX_INTENSITY 10.0 // Maximum heat intensity

// Physical scale parameters
#define CM_PER_CELL 25     // Each cell represents 25cm
```

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]
