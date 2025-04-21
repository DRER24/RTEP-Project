#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>

// HLK-LD6001 expects 22-byte packets
#define PACKET_SIZE 22
#define SERIAL_PORT "/dev/ttyUSB0"

// Heat map parameters
#define GRID_SIZE 20
#define DECAY_RATE 0.995
#define MAX_INTENSITY 10.0

// Physical scale parameters
#define CM_PER_CELL 25    // Each cell represents 25cm (adjust as needed)
#define MAX_RANGE 500     // Maximum radar range in cm (5 meters)

// ASCII visualization parameters
#define VIZ_UPDATE_MS 500    // Update rate in milliseconds

class RadarHeatMap {
private:
    int fd;                                   // Serial port file descriptor
    std::vector<std::vector<float>> heatmap;  // 2D heat map grid
    int current_x;                            // Current position X
    int current_y;                            // Current position Y
    std::atomic<bool> running;                // Flag for clean shutdown
    uint8_t last_detection_type;              // Last detection type
    uint8_t last_detection_strength;          // Last detection strength
    int detection_count;                      // Counter for detections
    std::mutex heatmap_mutex;                 // Mutex for thread-safe access to heatmap
    
    // ASCII characters for intensity levels (from low to high)
    const std::string intensityChars = " .:-=+*#%@";
    
    // Detection type labels
    const std::vector<std::string> typeLabels = {
        "None", "Human", "Multiple People", "Obstacle", "Strong Object", "Something Detected"
    };

public:
    RadarHeatMap() : 
        fd(-1),
        heatmap(GRID_SIZE, std::vector<float>(GRID_SIZE, 0.0f)),
        current_x(GRID_SIZE / 2),
        current_y(GRID_SIZE / 2),
        running(true),
        last_detection_type(0),
        last_detection_strength(0),
        detection_count(0) {
    }
    
    ~RadarHeatMap() {
        if (fd != -1) {
            sendCommand("AT+STOP\n");
            close(fd);
        }
    }
    
    // Convert bytes to hex string
    std::string bytesToHexString(const std::vector<uint8_t>& data) {
        std::ostringstream oss;
        for (auto byte : data) {
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
        }
        return oss.str();
    }

    // Parse packet for detection info
    bool parsePacket(const std::vector<uint8_t>& data, uint8_t& type, uint8_t& strength) {
        if (data.size() != 22 || !(data[0] == 0x4D && data[1] == 0x62 && data[2] == 0x10 && data[3] == 0x00)) {
            return false;
        }
        
        type = data[18];
        strength = data[19];
        
        return true;
    }

    // Send a command to the radar
    bool sendCommand(const std::string& command) {
        std::cout << "Sending command: " << command;
        write(fd, command.c_str(), command.length());
        // Simple delay to allow command processing
        usleep(100000); // 100ms
        return true;
    }

    // Configure and open the serial port
    bool setupSerial() {
        fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            std::cerr << "Error opening " << SERIAL_PORT << std::endl;
            return false;
        }
        
        struct termios options;
        tcgetattr(fd, &options);
        
        // Serial settings: 9600 8E1
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= PARENB;     // Enable parity
        options.c_cflag &= ~PARODD;    // Even parity
        options.c_cflag &= ~CSTOPB;    // 1 stop bit
        options.c_cflag &= ~CRTSCTS;   // No flow control
        
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        
        tcsetattr(fd, TCSANOW, &options);
        tcflush(fd, TCIOFLUSH);
        
        return true;
    }
    
    // Clear the terminal screen
    void clearScreen() {
        std::cout << "\033[2J\033[1;1H"; // ANSI escape codes to clear screen and move cursor to top-left
    }

    // Get ANSI color code for intensity value
    std::string getColorCode(float intensity) {
        // Normalize intensity to 0-1 range (avoiding std::min)
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
    
    // Update the heat map with new detection
    void updateHeatMap(uint8_t type, uint8_t strength) {
        std::lock_guard<std::mutex> lock(heatmap_mutex);
        
        // Store last detection info
        last_detection_type = type;
        last_detection_strength = strength;
        detection_count++;
        
        // Simple movement logic based on detection type
        int dx = 0, dy = 0;
        
        // Use strength and type to influence movement
        if (type == 1 || type == 2) {  // Human or multiple people
            // More active movement
            dx = rand() % 5 - 2;  // Range: -2 to 2
            dy = rand() % 5 - 2;
        } else {
            // Less active movement
            dx = rand() % 3 - 1;  // Range: -1 to 1
            dy = rand() % 3 - 1;
        }
        
        // Update position with bounds checking
        current_x += dx;
        if (current_x < 0) current_x = 0;
        if (current_x >= GRID_SIZE) current_x = GRID_SIZE - 1;
        
        current_y += dy;
        if (current_y < 0) current_y = 0;
        if (current_y >= GRID_SIZE) current_y = GRID_SIZE - 1;
        
        // Calculate intensity based on strength
        float intensity = (strength / 255.0f) * MAX_INTENSITY;
        
        // Apply different weights based on detection type
        float weight = 0.5f;
        switch(type) {
            case 1: weight = 1.0f; break;  // Human
            case 2: weight = 1.2f; break;  // Multiple People
            case 3: weight = 0.8f; break;  // Obstacle
            case 4: weight = 1.0f; break;  // Strong Object
            case 5: weight = 0.7f; break;  // Something Detected
        }
        
        intensity *= weight;
        
        // Add intensity to current position
        heatmap[current_y][current_x] += intensity;
        
        // Add some intensity to neighboring cells (diffusion)
        for (int y = current_y-1; y <= current_y+1; y++) {
            for (int x = current_x-1; x <= current_x+1; x++) {
                if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && 
                    (x != current_x || y != current_y)) {
                    // Add less intensity to neighbors
                    heatmap[y][x] += intensity * 0.3f;
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
            }
        }
    }
    
    // Apply decay to heat map so old detections fade
    void decayHeatMap() {
        std::lock_guard<std::mutex> lock(heatmap_mutex);
        
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                heatmap[y][x] *= DECAY_RATE;
            }
        }
    }
    
    // Create and print ASCII heat map visualization with cm scale
    void printHeatMap() {
        std::lock_guard<std::mutex> lock(heatmap_mutex);
        
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
                // Mark current detection position with X
                else if (x == current_x && y == current_y) {
                    std::cout << "\033[1;37m" << "X" << "\033[0m"; // Bright white X
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
        std::string typeLabel = "None";
        if (last_detection_type < typeLabels.size()) {
            typeLabel = typeLabels[last_detection_type];
        } else {
            typeLabel = "Unknown(" + std::to_string(last_detection_type) + ")";
        }
        
        // Calculate position in cm
        int pos_x_cm = (current_x - center_x) * CM_PER_CELL;
        int pos_y_cm = (center_y - current_y) * CM_PER_CELL; // Inverted because y increases downward
        
        std::cout << "Detection: " << typeLabel << 
                     " | Strength: " << static_cast<int>(last_detection_strength) <<
                     " | Events: " << detection_count << 
                     " | Position: (" << pos_x_cm << "cm, " << pos_y_cm << "cm)" << std::endl;
        
        std::cout << "\nLegend: " << " ";
        for (char c : intensityChars) {
            std::cout << c << " ";
        }
        std::cout << "(Low to High intensity) | R = Radar Position | X = Detection" << std::endl;
        
        std::cout << "\nPress Ctrl+C to exit" << std::endl;
    }
    
    // Thread function for periodically sending init command
    void sendInitCommandThread() {
        const uint8_t init_cmd[] = {
            0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0xBE, 0x4B
        };
        
        while (running) {
            write(fd, init_cmd, sizeof(init_cmd));
            std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Send every 300ms
        }
    }
    
    // Thread function for visualization
    void visualizationThread() {
        while (running) {
            printHeatMap();
            std::this_thread::sleep_for(std::chrono::milliseconds(VIZ_UPDATE_MS));
        }
    }
    
    // Main processing loop
    void run() {
        if (!setupSerial()) {
            std::cerr << "Failed to initialize serial port." << std::endl;
            return;
        }
        
        // Configure radar
        sendCommand("AT+STOP\n");
        sendCommand("AT+TIME=100\n");  // 100ms scan interval
        sendCommand("AT+START\n");
        
        std::cout << "\n[INFO] HLK-LD6001 Heat Map Visualization Started\n\n";
        
        // Start thread for sending init commands
        std::thread initThread(&RadarHeatMap::sendInitCommandThread, this);
        
        // Start visualization thread
        std::thread vizThread(&RadarHeatMap::visualizationThread, this);
        
        std::vector<uint8_t> buffer;
        uint8_t byte;
        
        auto lastDecayTime = std::chrono::steady_clock::now();
        
        try {
            while (running) {
                // Periodically apply decay to heat map
                auto currentTime = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastDecayTime).count() > 100) {
                    decayHeatMap();
                    lastDecayTime = currentTime;
                }
                
                // Read serial data
                int n = read(fd, &byte, 1);
                if (n > 0) {
                    buffer.push_back(byte);
                    
                    // Look for packet header (0x4D 0x62)
                    if (buffer.size() >= 2 && buffer[buffer.size() - 2] == 0x4D && buffer[buffer.size() - 1] == 0x62) {
                        // Reset buffer to start with the header
                        std::vector<uint8_t> newBuffer;
                        newBuffer.push_back(0x4D);
                        newBuffer.push_back(0x62);
                        buffer = newBuffer;
                    }
                    
                    if (buffer.size() == PACKET_SIZE) {
                        uint8_t type, strength;
                        if (parsePacket(buffer, type, strength)) {
                            if (type > 0) { // Skip if no detection
                                // Get current timestamp
                                auto now = std::chrono::system_clock::now();
                                auto now_time = std::chrono::system_clock::to_time_t(now);
                                std::tm tm = *std::localtime(&now_time);
                                
                                // Get type label
                                std::string label = "Unknown";
                                if (type < typeLabels.size()) {
                                    label = typeLabels[type];
                                } else {
                                    label = "Unknown(" + std::to_string(type) + ")";
                                }
                                
                                // Print detection info
                                std::cout << std::put_time(&tm, "[%H:%M:%S] ") 
                                        << "Detected: " << std::left << std::setw(20) << label 
                                        << "Strength: " << std::setw(3) << static_cast<int>(strength) << std::endl;
                                
                                // Update heat map
                                updateHeatMap(type, strength);
                            }
                        }
                        buffer.clear();
                    }
                } else {
                    // Small delay to prevent CPU hogging
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        } catch (std::exception& e) {
            std::cerr << "Error in main loop: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown error in main loop" << std::endl;
        }
        
        // Handle cleanup
        running = false;
        
        // Wait for threads to finish
        if (initThread.joinable()) {
            initThread.join();
        }
        if (vizThread.joinable()) {
            vizThread.join();
        }
        
        // Clear screen one more time before exit
        clearScreen();
        std::cout << "\n[INFO] Heat Map visualization stopped." << std::endl;
    }
};

int main() {
    // Seed random number generator
    srand(static_cast<unsigned int>(time(nullptr)));
    
    RadarHeatMap heatMap;
    heatMap.run();
    
    return 0;
}
