#ifndef RADAR_DETECTOR_H
#define RADAR_DETECTOR_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <memory>

// HLK-LD6001 expects 22-byte packets
#define PACKET_SIZE 22
#define SERIAL_PORT "/dev/ttyUSB0"

// Structure to hold a single detection event
struct DetectionEvent {
    std::chrono::system_clock::time_point timestamp;
    uint8_t type;
    uint8_t strength;
    int grid_x;
    int grid_y;
    int cm_x;
    int cm_y;
    std::string hex_data;
};

// Interface for detection callbacks
class RadarDetectorCallback {
public:
    virtual ~RadarDetectorCallback() = default;
    virtual void onDetectionEvent(const DetectionEvent& event) = 0;
};

class RadarDetector {
private:
    int fd;                              // Serial port file descriptor
    std::atomic<bool> running;           // Flag for clean shutdown
    uint8_t last_detection_type;         // Last detection type
    uint8_t last_detection_strength;     // Last detection strength
    int detection_count;                 // Counter for detections
    int current_x;                       // Current position X
    int current_y;                       // Current position Y
    std::vector<RadarDetectorCallback*> callbacks; // List of callback objects
    std::thread initThread;              // Thread for sending init commands
    std::thread processingThread;        // Thread for processing serial data

    // Detection type labels
    const std::vector<std::string> typeLabels = {
        "None", "Human", "Multiple People", "Obstacle", "Strong Object", "Something Detected"
    };
    
    // Physical scale parameters
    static constexpr int CM_PER_CELL = 25;    // Each cell represents 25cm
    static constexpr int MAX_RANGE = 500;     // Maximum radar range in cm (5 meters)
    static constexpr int GRID_SIZE = 20;      // Grid size for detecting

    // Convert bytes to hex string
    std::string bytesToHexString(const std::vector<uint8_t>& data);

    // Parse packet for detection info
    bool parsePacket(const std::vector<uint8_t>& data, uint8_t& type, uint8_t& strength);

    // Send a command to the radar
    bool sendCommand(const std::string& command);

    // Configure and open the serial port
    bool setupSerial();

    // Thread function for periodically sending init command
    void sendInitCommandThread();

    // Thread function for processing serial data
    void processSerialData();

public:
    RadarDetector();
    ~RadarDetector();

    // Start radar detection
    bool start();

    // Stop radar detection
    void stop();

    // Register a callback for detection events
    void registerCallback(RadarDetectorCallback* callback);

    // Unregister a callback
    void unregisterCallback(RadarDetectorCallback* callback);

    // Get detection count
    int getDetectionCount() const;

    // Get last detection type
    uint8_t getLastDetectionType() const;

    // Get last detection strength
    uint8_t getLastDetectionStrength() const;

    // Get current positions
    void getCurrentPosition(int& x, int& y) const;

    // Get detection type label
    std::string getTypeLabel(uint8_t type) const;
};

#endif // RADAR_DETECTOR_H