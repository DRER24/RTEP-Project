#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "heatmap_manager.h"
#include "radar_detector.h"
#include <thread>
#include <atomic>
#include <chrono>

// ASCII visualization parameters
#define VIZ_UPDATE_MS 500    // Update rate in milliseconds

class Visualizer : public HeatMapVisualizerCallback {
private:
    HeatMapManager* heatmap_manager;
    RadarDetector* radar_detector;
    std::atomic<bool> running;
    std::thread visualizationThread;
    std::string latest_graph_file;
    
    // Clear the terminal screen
    void clearScreen();
    
    // Get ANSI color code for intensity value
    std::string getColorCode(float intensity);
    
    // Get color code for specific detection type
    std::string getDetectionTypeColor(uint8_t type);
    
    // Get symbol for specific detection type
    char getDetectionTypeSymbol(uint8_t type);
    
    // Create and print ASCII heat map visualization
    void printHeatMap();
    
    // Thread function for visualization
    void visualizationThread_func();

public:
    Visualizer(HeatMapManager* heatmapMgr, RadarDetector* radarDetector);
    ~Visualizer();
    
    // Start visualization
    void start();
    
    // Stop visualization
    void stop();
    
    // Implementations of HeatMapVisualizerCallback
    virtual void onHeatMapUpdated() override;
    virtual void onNewGraphAvailable(const std::string& filename) override;
};

#endif // VISUALIZER_H