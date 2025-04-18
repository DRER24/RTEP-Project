#include "radar_detector.h"
#include "heatmap_manager.h"
#include "visualizer.h"
#include <iostream>
#include <csignal>
#include <memory>

// Global flag for clean shutdown
std::atomic<bool> g_running(true);

// Signal handler for graceful termination
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down...\n";
    g_running = false;
}

int main() {
    // Register signal handler for Ctrl+C
    std::signal(SIGINT, signalHandler);
    
    // Seed random number generator
    srand(static_cast<unsigned int>(time(nullptr)));
    
    std::cout << "Starting HLK-LD6001 Radar Heat Map Visualization" << std::endl;
    std::cout << "==================================================" << std::endl;
    
    // Create the components
    std::cout << "Initializing radar detector..." << std::endl;
    auto radarDetector = std::make_unique<RadarDetector>();
    
    std::cout << "Initializing heat map manager..." << std::endl;
    auto heatMapManager = std::make_unique<HeatMapManager>();
    
    std::cout << "Initializing visualizer..." << std::endl;
    auto visualizer = std::make_unique<Visualizer>(heatMapManager.get(), radarDetector.get());
    
    // Connect the components
    std::cout << "Connecting components..." << std::endl;
    radarDetector->registerCallback(heatMapManager.get());
    heatMapManager->registerCallback(visualizer.get());
    
    // Start the components
    std::cout << "Starting components..." << std::endl;
    bool success = true;
    
    success = heatMapManager->start();
    if (!success) {
        std::cerr << "Failed to start heat map manager. Exiting." << std::endl;
        return 1;
    }
    
    success = radarDetector->start();
    if (!success) {
        std::cerr << "Failed to start radar detector. Exiting." << std::endl;
        heatMapManager->stop();
        return 1;
    }
    
    visualizer->start();
    
    std::cout << "System running. Press Ctrl+C to exit." << std::endl;
    
    // Main thread just waits until signaled to stop
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Shutdown in reverse order
    std::cout << "Shutting down components..." << std::endl;
    visualizer->stop();
    radarDetector->stop();
    heatMapManager->stop();
    
    std::cout << "System shutdown complete." << std::endl;
    
    return 0;
}