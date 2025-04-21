#include <iostream>
#include "RadarSystem.h"

/**
 * @brief Main entry point for the radar tracking system
 * 
 * This program controls a radar-based target tracking system with servo motors
 * to physically follow detected targets.
 * 
 * Command line arguments:
 * 1. Radar serial port (default: /dev/ttyAMA0)
 * 2. Radar baud rate (default: 9600)
 * 3. I2C device for servo control (default: /dev/i2c-1)
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return Exit code
 */
int main(int argc, char* argv[]) {
    // Default parameters
    std::string radar_port = "/dev/ttyAMA0";
    int radar_baud = 9600;
    std::string i2c_device = "/dev/i2c-1";
    
    // Parse command line arguments
    if (argc >= 2) {
        radar_port = argv[1];
    }
    
    if (argc >= 3) {
        try {
            radar_baud = std::stoi(argv[2]);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Invalid baud rate: " << argv[2] << std::endl;
            return 1;
        }
    }
    
    if (argc >= 4) {
        i2c_device = argv[3];
    }
    
    // Create and run the radar system
    RadarSystem radar_system(radar_port, radar_baud, i2c_device);
    
    try {
        return radar_system.run();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception" << std::endl;
        return 1;
    }
}
