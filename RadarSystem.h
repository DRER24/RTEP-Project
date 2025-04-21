#ifndef RADAR_SYSTEM_H
#define RADAR_SYSTEM_H

#include <string>
#include <atomic>
#include "SerialComm.h"
#include "ServoController.h"
#include "Target.h"
#include "RadarData.h"

/**
 * @class RadarSystem
 * @brief Main system class integrating radar and servo control
 * 
 * This class coordinates the radar data acquisition and servo control
 * to create a complete target tracking system.
 */
class RadarSystem {
public:
    /**
     * @brief Construct a new RadarSystem object
     * @param radar_port Serial port for radar
     * @param radar_baud Baud rate for radar communication
     * @param i2c_device I2C device for servo controller
     */
    explicit RadarSystem(const std::string& radar_port = "/dev/ttyAMA0",
                       int radar_baud = 9600,
                       const std::string& i2c_device = "/dev/i2c-1");
    
    /**
     * @brief Destroy the RadarSystem object
     */
    ~RadarSystem();
    
    /**
     * @brief Initialize the system
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the system
     * @return true if start succeeded, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the system
     */
    void stop();
    
    /**
     * @brief Run the system main loop
     * @return Exit code
     */
    int run();
    
    /**
     * @brief Cleanup resources
     */
    void cleanup();

private:
    /**
     * @brief Callback for radar data
     * @param data The radar data
     */
    void onRadarData(const RadarData& data);
    
    // Components
    SerialComm serial_comm_;         ///< Serial communication with radar
    ServoController servo_controller_; ///< Servo controller
    
    // Configuration
    std::string radar_port_;         ///< Radar serial port
    int radar_baud_;                 ///< Radar baud rate
    std::string i2c_device_;         ///< I2C device for servos
    
    // Running state
    std::atomic<bool> running_;      ///< Flag indicating if the system is running
};

#endif // RADAR_SYSTEM_H
