#include "visualizer.h"

Visualizer::Visualizer(HeatMapManager* heatmapMgr, RadarDetector* radarDetector) :
    heatmap_manager(heatmapMgr),
    radar_detector(radarDetector),
    running(false) {
}

Visualizer::~Visualizer() {
    stop();
}

void Visualizer::start() {
    if (running) {
        return; // Already running
    }
    
    running = true;
    visualizationThread = std::thread(&Visualizer::visualizationThread_func, this);
    
    std::cout << "[INFO] Visualizer Started" << std::endl;
}

void Visualizer::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    
    if (visualizationThread.joinable()) {
        visualizationThread.join();
    }
    
    std::cout << "[INFO] Visualizer stopped." << std::endl;
}

void Visualizer::onHeatMapUpdated() {
    // This method is called when the heat map is updated
    // We don't need to do anything here, as we already update at regular intervals
}

void Visualizer::onNewGraphAvailable(const std::string& filename) {
    latest_graph_file = filename;
}

void Visualizer::clearScreen() {
    std::cout << "\033[2J\033[1;1H"; // ANSI escape codes to clear screen and move cursor to top-left
}

std::string Visualizer::getColorCode(float intensity) {
    // Normalize intensity to 0-1 range
    float normalized = intensity / MAX_INTENSITY;
    if (normalized > 1.0) normalized = 1.0;
    
    // Use ANSI color codes for terminal output
    // Blue (low intensity) to Red (high intensity)
    if (normalized < 0.2) {
        return "\033[34m"; // Blue
    } else if (normalized < 0.4) {
        return "\033[36m"; // Cyan
    } else if (normalized < 0.6) {
        return "\033[32m"; // Green
    } else if (normalized < 0.8) {
        return "\033[33m"; // Yellow
    } else {
        return "\033[31m"; // Red
    }
}

// Get color code for specific detection type
std::string Visualizer::getDetectionTypeColor(uint8_t type) {
    switch(type) {
        case 1: // Human
            return "\033[1;35m"; // Bright magenta
        case 2: // Multiple People
            return "\033[1;36m"; // Bright cyan
        case 3: // Obstacle
            return "\033[1;33m"; // Bright yellow
        case 4: // Strong Object
            return "\033[1;31m"; // Bright red
        case 5: // Something Detected
            return "\033[1;32m"; // Bright green
        default:
            return "\033[1;37m"; // Bright white
    }
}

// Get symbol for specific detection type
char Visualizer::getDetectionTypeSymbol(uint8_t type) {
    switch(type) {
        case 1: // Human
            return 'H';
        case 2: // Multiple People
            return 'M';
        case 3: // Obstacle
            return 'O';
        case 4: // Strong Object
            return 'S';
        case 5: // Something Detected
            return 'D';
        default:
            return 'X';
    }
}

void Visualizer::printHeatMap() {
    // Get current heatmap data
    auto heatmap = heatmap_manager->getHeatMap();
    std::string intensityChars = heatmap_manager->getIntensityChars();
    
    // Get detection information
    int current_x, current_y;
    radar_detector->getCurrentPosition(current_x, current_y);
    uint8_t last_detection_type = radar_detector->getLastDetectionType();
    uint8_t last_detection_strength = radar_detector->getLastDetectionStrength();
    int detection_count = radar_detector->getDetectionCount();
    
    clearScreen();
    
    std::cout << "HLK-LD6001 Radar Heat Map Visualization (Scale: " << CM_PER_CELL << "cm per cell)\n";
    std::cout << "==========================================================================\n\n";
    
    // Calculate center point (radar position)
    int center_x = GRID_SIZE / 2;
    int center_y = GRID_SIZE / 2;
    
    // Print Y-axis labels and top axis
    std::cout << "     "; // Padding for y-axis labels
    
    // Print X-axis labels (top)
    for (int x = 0; x < GRID_SIZE; x++) {
        // Calculate cm value for this x position
        int cm_x = (x - center_x) * CM_PER_CELL;
        
        // Only print labels at intervals to avoid crowding
        if (x % 5 == 0) {
            std::cout << std::setw(4) << cm_x << "cm ";
        } else {
            std::cout << "     "; // Space for non-labeled positions
        }
    }
    std::cout << "\n     ";
    
    // Print top grid line
    for (int x = 0; x < GRID_SIZE; x++) {
        if (x % 5 == 0) {
            std::cout << "+----";
        } else {
            std::cout << "-----";
        }
    }
    std::cout << "+\n";
    
    // Print the heat map grid with axes
    for (int y = 0; y < GRID_SIZE; y++) {
        // Calculate cm value for this y position
        int cm_y = (center_y - y) * CM_PER_CELL; // Inverted because y increases downward
        
        // Print y-axis label
        if (y % 5 == 0) {
            std::cout << std::setw(4) << cm_y << "cm|";
        } else {
            std::cout << "     |";
        }
        
        // Print grid row
        for (int x = 0; x < GRID_SIZE; x++) {
            // Get intensity value
            float intensity = heatmap[y][x];
            
            // Map intensity to character
            int charIndex = (int)((intensity / MAX_INTENSITY) * intensityChars.length());
            if (charIndex >= (int)intensityChars.length()) {
                charIndex = intensityChars.length() - 1;
            }
            if (charIndex < 0) charIndex = 0;
            
            char intensityChar = intensityChars[charIndex];
            
            // Use ANSI color codes for terminal output
            std::string colorCode = getColorCode(intensity);
            
            // Mark radar position with R
            if (x == center_x && y == center_y) {
                std::cout << "\033[1;37m" << "R" << "\033[0m"; // Bright white R for Radar
            }
            // Mark current detection position with type-specific symbol
            else if (x == current_x && y == current_y) {
                std::string typeColor = getDetectionTypeColor(last_detection_type);
                char typeSymbol = getDetectionTypeSymbol(last_detection_type);
                std::cout << typeColor << typeSymbol << "\033[0m"; // Colored type-specific symbol
            } else {
                // Print colored intensity character
                std::cout << colorCode << intensityChar << "\033[0m";
            }
            
            // Add grid markers at intervals
            if (x % 5 == 4 && x < GRID_SIZE-1) {
                std::cout << colorCode << "|" << "\033[0m";
            } else {
                std::cout << " ";
            }
        }
        
        // Print right y-axis label
        if (y % 5 == 0) {
            std::cout << "| " << std::setw(4) << cm_y << "cm";
        } else {
            std::cout << "|";
        }
        std::cout << "\n";
        
        // Add horizontal grid lines at intervals
        if (y % 5 == 4 && y < GRID_SIZE-1) {
            std::cout << "     +";
            for (int x = 0; x < GRID_SIZE; x++) {
                if (x % 5 == 4 && x < GRID_SIZE-1) {
                    std::cout << "----+";
                } else {
                    std::cout << "-----";
                }
            }
            std::cout << "+\n";
        }
    }
    
    // Print bottom grid line
    std::cout << "     +";
    for (int x = 0; x < GRID_SIZE; x++) {
        if (x % 5 == 0) {
            std::cout << "----+";
        } else {
            std::cout << "-----";
        }
    }
    std::cout << "\n     ";
    
    // Print X-axis labels (bottom)
    for (int x = 0; x < GRID_SIZE; x++) {
        // Calculate cm value for this x position
        int cm_x = (x - center_x) * CM_PER_CELL;
        
        // Only print labels at intervals to avoid crowding
        if (x % 5 == 0) {
            std::cout << std::setw(4) << cm_x << "cm ";
        } else {
            std::cout << "     ";
        }
    }
    std::cout << "\n\n";
    
    // Print status information
    std::string typeLabel = radar_detector->getTypeLabel(last_detection_type);
    
    // Calculate position in cm
    int pos_x_cm = (current_x - center_x) * CM_PER_CELL;
    int pos_y_cm = (center_y - current_y) * CM_PER_CELL; // Inverted because y increases downward
    
    // Calculate elapsed time
    auto elapsed = std::chrono::system_clock::now() - heatmap_manager->getStartTime();
    int elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    
    // Calculate time until next graph
    int time_to_next_graph = heatmap_manager->getTimeToNextGraph();
    
    std::cout << "Detection: " << typeLabel << 
                 " | Strength: " << static_cast<int>(last_detection_strength) <<
                 " | Events: " << detection_count << 
                 " | Position: (" << pos_x_cm << "cm, " << pos_y_cm << "cm)" << std::endl;
    
    std::cout << "Running time: " << elapsed_sec << "s | Graphs generated: " << heatmap_manager->getGraphCount() << 
                 " | Next graph in: " << time_to_next_graph << "s" << std::endl;
    
    std::cout << "Latest graph file: " << (latest_graph_file.empty() ? "None yet" : latest_graph_file) << std::endl;
    
    // Print intensity legend
    std::cout << "\nIntensity: " << " ";
    for (char c : intensityChars) {
        std::cout << c << " ";
    }
    std::cout << "(Low to High)" << std::endl;
    
    // Print detection type legend
    std::cout << "Detection types: R = Radar | H = Human | M = Multiple People | O = Obstacle | S = Strong Object | D = Detected" << std::endl;
    
    std::cout << "\nPress Ctrl+C to exit" << std::endl;
}

void Visualizer::visualizationThread_func() {
    while (running) {
        printHeatMap();
        std::this_thread::sleep_for(std::chrono::milliseconds(VIZ_UPDATE_MS));
    }
}