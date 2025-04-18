#include "radar_detector.h"

RadarDetector::RadarDetector() : 
    fd(-1),
    running(false),
    last_detection_type(0),
    last_detection_strength(0),
    detection_count(0),
    current_x(GRID_SIZE / 2),
    current_y(GRID_SIZE / 2) {
}

RadarDetector::~RadarDetector() {
    stop();
}

bool RadarDetector::start() {
    if (running) {
        return true; // Already running
    }
    
    if (!setupSerial()) {
        std::cerr << "Failed to initialize serial port." << std::endl;
        return false;
    }
    
    // Configure radar
    sendCommand("AT+STOP\n");
    sendCommand("AT+TIME=100\n");  // 100ms scan interval
    sendCommand("AT+START\n");
    
    running = true;
    
    // Start thread for sending init commands
    initThread = std::thread(&RadarDetector::sendInitCommandThread, this);
    
    // Start processing thread
    processingThread = std::thread(&RadarDetector::processSerialData, this);
    
    std::cout << "[INFO] HLK-LD6001 Radar Detection Started" << std::endl;
    
    return true;
}

void RadarDetector::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    
    // Wait for threads to finish
    if (initThread.joinable()) {
        initThread.join();
    }
    
    if (processingThread.joinable()) {
        processingThread.join();
    }
    
    if (fd != -1) {
        sendCommand("AT+STOP\n");
        close(fd);
        fd = -1;
    }
    
    std::cout << "[INFO] Radar Detection stopped." << std::endl;
}

void RadarDetector::registerCallback(RadarDetectorCallback* callback) {
    if (callback) {
        callbacks.push_back(callback);
    }
}

void RadarDetector::unregisterCallback(RadarDetectorCallback* callback) {
    callbacks.erase(
        std::remove(callbacks.begin(), callbacks.end(), callback),
        callbacks.end()
    );
}

int RadarDetector::getDetectionCount() const {
    return detection_count;
}

uint8_t RadarDetector::getLastDetectionType() const {
    return last_detection_type;
}

uint8_t RadarDetector::getLastDetectionStrength() const {
    return last_detection_strength;
}

void RadarDetector::getCurrentPosition(int& x, int& y) const {
    x = current_x;
    y = current_y;
}

std::string RadarDetector::getTypeLabel(uint8_t type) const {
    if (type < typeLabels.size()) {
        return typeLabels[type];
    }
    return "Unknown(" + std::to_string(type) + ")";
}

std::string RadarDetector::bytesToHexString(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (auto byte : data) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return oss.str();
}

bool RadarDetector::parsePacket(const std::vector<uint8_t>& data, uint8_t& type, uint8_t& strength) {
    if (data.size() != 22 || !(data[0] == 0x4D && data[1] == 0x62 && data[2] == 0x10 && data[3] == 0x00)) {
        return false;
    }
    
    type = data[18];
    strength = data[19];
    
    return true;
}

bool RadarDetector::sendCommand(const std::string& command) {
    std::cout << "Sending command: " << command;
    write(fd, command.c_str(), command.length());
    // Simple delay to allow command processing
    usleep(100000); // 100ms
    return true;
}

bool RadarDetector::setupSerial() {
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

void RadarDetector::sendInitCommandThread() {
    const uint8_t init_cmd[] = {
        0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xBE, 0x4B
    };
    
    while (running) {
        write(fd, init_cmd, sizeof(init_cmd));
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Send every 300ms
    }
}

void RadarDetector::processSerialData() {
    std::vector<uint8_t> buffer;
    uint8_t byte;
    
    try {
        while (running) {
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
                            // Update internal state
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
                            
                            // Create detection event
                            DetectionEvent event;
                            event.timestamp = std::chrono::system_clock::now();
                            event.type = type;
                            event.strength = strength;
                            event.grid_x = current_x;
                            event.grid_y = current_y;
                            
                            // Calculate center point (radar position)
                            int center_x = GRID_SIZE / 2;
                            int center_y = GRID_SIZE / 2;
                            
                            // Calculate position in cm
                            event.cm_x = (current_x - center_x) * CM_PER_CELL;
                            event.cm_y = (center_y - current_y) * CM_PER_CELL; // Inverted because y increases downward
                            
                            // Store hex data
                            event.hex_data = bytesToHexString(buffer);
                            
                            // Notify all callbacks
                            for (auto callback : callbacks) {
                                callback->onDetectionEvent(event);
                            }
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
        std::cerr << "Error in processing thread: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in processing thread" << std::endl;
    }
}