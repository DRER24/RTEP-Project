#ifndef HEATMAP_MANAGER_H
#define HEATMAP_MANAGER_H

#include "radar_detector.h"
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>

// Heat map parameters
#define GRID_SIZE 20
#define DECAY_RATE 0.995
#define MAX_INTENSITY 10.0

// Physical scale parameters
#define CM_PER_CELL 25    // Each cell represents 25cm

// Interface for heat map visualization
class HeatMapVisualizerCallback {
public:
    virtual ~HeatMapVisualizerCallback() = default;
    virtual void onHeatMapUpdated() = 0;
    virtual void onNewGraphAvailable(const std::string& filename) = 0;
};

class HeatMapManager : public RadarDetectorCallback {
private:
    std::vector<std::vector<float>> heatmap;        // 2D heat map grid for visualization
    std::vector<std::vector<float>> graphHeatmap;   // Heat map for graph output
    std::atomic<bool> running;                      // Flag for clean shutdown
    std::thread decayThread;                        // Thread for decaying the heat map
    std::thread graphThread;                        // Thread for generating graphs
    std::mutex heatmap_mutex;                       // Mutex for thread-safe access to heatmap
    std::chrono::system_clock::time_point start_time;     // Collection start time
    std::chrono::system_clock::time_point last_graph_time; // Time when last graph was generated
    int graph_count;                                // Counter for generated graphs
    std::string output_filename;                    // Output data file name
    std::string graph_filename_prefix;              // Prefix for graph files
    std::vector<DetectionEvent> collected_data;     // Collected detection events
    std::ofstream data_file;                        // Output data file
    std::vector<HeatMapVisualizerCallback*> callbacks; // Visualization callbacks
    
    // ASCII characters for intensity levels (from low to high)
    const std::string intensityChars = " .:-=+*#%@";
    
    // Thread function for decaying the heat map
    void decayThread_func();
    
    // Thread function for generating graphs
    void graphThread_func();
    
    // Generate a graph file from current heat map data
    void generateGraph();
    
    // Apply decay to heat map
    void decayHeatMap();

public:
    HeatMapManager(const std::string& outputFile = "radar_data.csv", 
                  const std::string& graphPrefix = "radar_graph_");
    ~HeatMapManager();
    
    // Start heat map processing
    bool start();
    
    // Stop heat map processing
    void stop();
    
    // Register a visualizer callback
    void registerCallback(HeatMapVisualizerCallback* callback);
    
    // Unregister a visualizer callback
    void unregisterCallback(HeatMapVisualizerCallback* callback);
    
    // Get current heat map (thread-safe)
    std::vector<std::vector<float>> getHeatMap();
    
    // Get graph heat map (thread-safe)
    std::vector<std::vector<float>> getGraphHeatMap();
    
    // Get heat map intensity character set
    std::string getIntensityChars() const;
    
    // Get the number of graphs generated
    int getGraphCount() const;
    
    // Get start time
    std::chrono::system_clock::time_point getStartTime() const;
    
    // Get time until next graph
    int getTimeToNextGraph() const;
    
    // Implementation of RadarDetectorCallback
    virtual void onDetectionEvent(const DetectionEvent& event) override;
    
    // Open data file for writing
    bool openDataFile();
    
    // Save detection event to file
    void saveDetectionToFile(const DetectionEvent& event);
};

#endif // HEATMAP_MANAGER_H