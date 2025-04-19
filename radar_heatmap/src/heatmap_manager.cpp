#include "heatmap_manager.h"

HeatMapManager::HeatMapManager(const std::string& outputFile, const std::string& graphPrefix) : 
    heatmap(GRID_SIZE, std::vector<float>(GRID_SIZE, 0.0f)),
    graphHeatmap(GRID_SIZE, std::vector<float>(GRID_SIZE, 0.0f)),
    running(false),
    graph_count(0),
    output_filename(outputFile),
    graph_filename_prefix(graphPrefix) {
}

HeatMapManager::~HeatMapManager() {
    stop();
    
    // Close data file if open
    if (data_file.is_open()) {
        data_file.close();
    }
}

bool HeatMapManager::start() {
    if (running) {
        return true; // Already running
    }
    
    if (!openDataFile()) {
        std::cerr << "Failed to open data file." << std::endl;
        return false;
    }
    
    running = true;
    
    // Record start time and last graph time
    start_time = std::chrono::system_clock::now();
    last_graph_time = start_time;
    
    // Start decay thread
    decayThread = std::thread(&HeatMapManager::decayThread_func, this);
    
    // Start graph thread
    graphThread = std::thread(&HeatMapManager::graphThread_func, this);
    
    std::cout << "[INFO] Heat Map Manager Started" << std::endl;
    
    return true;
}

void HeatMapManager::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    
    // Wait for threads to finish
    if (decayThread.joinable()) {
        decayThread.join();
    }
    
    if (graphThread.joinable()) {
        graphThread.join();
    }
    
    // Generate final graph if needed
    auto time_since_last_graph = std::chrono::system_clock::now() - last_graph_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(time_since_last_graph).count() > 2) { // At least 2 seconds of data
        generateGraph();
    }
    
    // Close data file
    if (data_file.is_open()) {
        data_file.close();
    }
    
    std::cout << "[INFO] Heat Map Manager stopped." << std::endl;
    std::cout << "Data saved to " << output_filename << std::endl;
    std::cout << "Generated " << graph_count << " graph files with prefix " << graph_filename_prefix << std::endl;
}

void HeatMapManager::registerCallback(HeatMapVisualizerCallback* callback) {
    if (callback) {
        callbacks.push_back(callback);
    }
}

void HeatMapManager::unregisterCallback(HeatMapVisualizerCallback* callback) {
    callbacks.erase(
        std::remove(callbacks.begin(), callbacks.end(), callback),
        callbacks.end()
    );
}

std::vector<std::vector<float>> HeatMapManager::getHeatMap() {
    std::lock_guard<std::mutex> lock(heatmap_mutex);
    return heatmap;
}

std::vector<std::vector<float>> HeatMapManager::getGraphHeatMap() {
    std::lock_guard<std::mutex> lock(heatmap_mutex);
    return graphHeatmap;
}

std::string HeatMapManager::getIntensityChars() const {
    return intensityChars;
}

int HeatMapManager::getGraphCount() const {
    return graph_count;
}

std::chrono::system_clock::time_point HeatMapManager::getStartTime() const {
    return start_time;
}

int HeatMapManager::getTimeToNextGraph() const {
    // Calculate time until next graph
    auto time_since_last_graph = std::chrono::system_clock::now() - last_graph_time;
    int time_to_next_graph = 10 - std::chrono::duration_cast<std::chrono::seconds>(time_since_last_graph).count();
    if (time_to_next_graph < 0) time_to_next_graph = 0;
    return time_to_next_graph;
}

void HeatMapManager::onDetectionEvent(const DetectionEvent& event) {
    std::lock_guard<std::mutex> lock(heatmap_mutex);
    
    // Save to file
    saveDetectionToFile(event);
    
    // Store the event in our collection for reference
    collected_data.push_back(event);
    
    // Keep collection size manageable (optional, retain only last 100 events)
    if (collected_data.size() > 100) {
        collected_data.erase(collected_data.begin());
    }
    
    // Calculate intensity based on strength
    float intensity = (event.strength / 255.0f) * MAX_INTENSITY;
    
    // Apply different weights based on detection type
    float weight = 0.5f;
    switch(event.type) {
        case 1: weight = 1.0f; break;  // Human
        case 2: weight = 1.2f; break;  // Multiple People
        case 3: weight = 0.8f; break;  // Obstacle
        case 4: weight = 1.0f; break;  // Strong Object
        case 5: weight = 0.7f; break;  // Something Detected
    }
    
    intensity *= weight;
    
    // Add intensity to current position
    int x = event.grid_x;
    int y = event.grid_y;
    
    heatmap[y][x] += intensity;
    graphHeatmap[y][x] += intensity;
    
    // Add some intensity to neighboring cells (diffusion)
    for (int ny = y-1; ny <= y+1; ny++) {
        for (int nx = x-1; nx <= x+1; nx++) {
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && 
                (nx != x || ny != y)) {
                // Add less intensity to neighbors
                heatmap[ny][nx] += intensity * 0.3f;
                graphHeatmap[ny][nx] += intensity * 0.3f;
            }
        }
    }
    
    // Clip values to prevent excessive buildup
    float max_value = MAX_INTENSITY * 2;
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (heatmap[y][x] > max_value) {
                heatmap[y][x] = max_value;
            }
            if (graphHeatmap[y][x] > max_value) {
                graphHeatmap[y][x] = max_value;
            }
        }
    }
    
    // Notify visualization callbacks
    for (auto callback : callbacks) {
        callback->onHeatMapUpdated();
    }
}

bool HeatMapManager::openDataFile() {
    data_file.open(output_filename);
    if (!data_file.is_open()) {
        std::cerr << "Error opening output file: " << output_filename << std::endl;
        return false;
    }
    
    // Write CSV header
    data_file << "Timestamp,Type,TypeLabel,Strength,GridX,GridY,PosX_cm,PosY_cm,HexData" << std::endl;
    return true;
}

void HeatMapManager::saveDetectionToFile(const DetectionEvent& event) {
    if (!data_file.is_open()) {
        return;
    }
    
    // Get type label
    std::string label = "Unknown";
    if (event.type < 6) { // 6 is the size of typeLabels vector in RadarDetector
        const std::vector<std::string> typeLabels = {
            "None", "Human", "Multiple People", "Obstacle", "Strong Object", "Something Detected"
        };
        label = typeLabels[event.type];
    } else {
        label = "Unknown(" + std::to_string(event.type) + ")";
    }
    
    // Convert timestamp to string
    auto time_t_val = std::chrono::system_clock::to_time_t(event.timestamp);
    std::tm tm = *std::localtime(&time_t_val);
    std::ostringstream timestamp_ss;
    timestamp_ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    // Write CSV line
    data_file << timestamp_ss.str() << ","
              << static_cast<int>(event.type) << ","
              << label << ","
              << static_cast<int>(event.strength) << ","
              << event.grid_x << ","
              << event.grid_y << ","
              << event.cm_x << ","
              << event.cm_y << ","
              << event.hex_data << std::endl;
}

void HeatMapManager::decayThread_func() {
    auto lastDecayTime = std::chrono::steady_clock::now();
    
    while (running) {
        // Apply decay every 100ms
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastDecayTime).count() > 100) {
            decayHeatMap();
            lastDecayTime = currentTime;
        }
        
        // Sleep to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void HeatMapManager::graphThread_func() {
    while (running) {
        // Check if it's time to generate a graph
        auto current_time = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_graph_time).count() >= 10) { // Generate every 10 seconds
            generateGraph();
            last_graph_time = current_time;
        }
        
        // Sleep to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void HeatMapManager::decayHeatMap() {
    std::lock_guard<std::mutex> lock(heatmap_mutex);
    
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            heatmap[y][x] *= DECAY_RATE;
        }
    }
    
    // We don't decay the graph heatmap here since we want it to accumulate over the interval
}

void HeatMapManager::generateGraph() {
    std::lock_guard<std::mutex> lock(heatmap_mutex);
    
    // Create a filename with sequence number
    std::ostringstream filename;
    filename << graph_filename_prefix << std::setw(3) << std::setfill('0') << graph_count << ".txt";
    
    // Open file for writing
    std::ofstream graph_file(filename.str());
    if (!graph_file.is_open()) {
        std::cerr << "Error opening graph file: " << filename.str() << std::endl;
        return;
    }
    
    // Write header
    graph_file << "HLK-LD6001 Heat Map Graph #" << graph_count << "\n";
    graph_file << "Time period: 10 seconds\n";
    graph_file << "Scale: " << CM_PER_CELL << "cm per cell\n";
    graph_file << "========================================\n\n";
    
    // Calculate center point (radar position)
    int center_x = GRID_SIZE / 2;
    int center_y = GRID_SIZE / 2;
    
    // Write Y-axis labels and top axis
    graph_file << "     "; // Padding for y-axis labels
    
    // Write X-axis labels (top)
    for (int x = 0; x < GRID_SIZE; x += 5) {
        // Calculate cm value for this x position
        int cm_x = (x - center_x) * CM_PER_CELL;
        graph_file << std::setw(5) << cm_x << "cm";
    }
    graph_file << "\n";
    
    // Get the current detection location and type from the latest event
    int current_x = -1, current_y = -1;
    uint8_t detection_type = 0;
    
    // Use the most recent event's data if available
    if (!collected_data.empty()) {
        const auto& latestEvent = collected_data.back();
        current_x = latestEvent.grid_x;
        current_y = latestEvent.grid_y;
        detection_type = latestEvent.type;
    }
    
    // Write the heat map grid with axes
    for (int y = 0; y < GRID_SIZE; y++) {
        // Calculate cm value for this y position
        int cm_y = (center_y - y) * CM_PER_CELL; // Inverted because y increases downward
        
        // Print y-axis label
        if (y % 5 == 0) {
            graph_file << std::setw(4) << cm_y << "cm|";
        } else {
            graph_file << "     |";
        }
        
        // Print grid row
        for (int x = 0; x < GRID_SIZE; x++) {
            // Get intensity value
            float intensity = graphHeatmap[y][x];
            
            // Map intensity to character
            int charIndex = (int)((intensity / MAX_INTENSITY) * intensityChars.length());
            if (charIndex >= (int)intensityChars.length()) {
                charIndex = intensityChars.length() - 1;
            }
            if (charIndex < 0) charIndex = 0;
            
            char intensityChar = intensityChars[charIndex];
            
            // Mark radar position with R
            if (x == center_x && y == center_y) {
                graph_file << "R";
            }
            // Mark detection position with type-specific symbol
            else if (x == current_x && y == current_y) {
                // Determine symbol based on detection type
                char typeSymbol = 'X'; // Default
                switch(detection_type) {
                    case 1: typeSymbol = 'H'; break; // Human
                    case 2: typeSymbol = 'M'; break; // Multiple People
                    case 3: typeSymbol = 'O'; break; // Obstacle
                    case 4: typeSymbol = 'S'; break; // Strong Object
                    case 5: typeSymbol = 'D'; break; // Something Detected
                }
                graph_file << typeSymbol;
            } else {
                // Print intensity character
                graph_file << intensityChar;
            }
            
            graph_file << " ";
        }
        
        // Print right y-axis label
        if (y % 5 == 0) {
            graph_file << "| " << std::setw(4) << cm_y << "cm";
        } else {
            graph_file << "|";
        }
        graph_file << "\n";
    }
    
    // Write X-axis labels (bottom)
    graph_file << "     "; // Padding for y-axis labels
    for (int x = 0; x < GRID_SIZE; x += 5) {
        // Calculate cm value for this x position
        int cm_x = (x - center_x) * CM_PER_CELL;
        graph_file << std::setw(5) << cm_x << "cm";
    }
    graph_file << "\n\n";
    
    // Write status and legend
    graph_file << "Intensity legend: " << intensityChars << " (Low to High intensity)\n";
    graph_file << "Symbol legend: R = Radar | H = Human | M = Multiple People | O = Obstacle | S = Strong Object | D = Detected\n";
    
    // Close the file
    graph_file.close();
    
    std::cout << "\nGenerated graph #" << graph_count << " to " << filename.str() << "\n";
    
    // Notify callbacks about new graph
    for (auto callback : callbacks) {
        callback->onNewGraphAvailable(filename.str());
    }
    
    // Reset graph heat map for next period
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graphHeatmap[y][x] = 0.0f;
        }
    }
    
    // Increment graph counter
    graph_count++;
}