#include "RadarSystem.h"
#include "SignalHandler.h"
#include <iostream>
#include <functional>
#include <thread>
#include <chrono>

RadarSystem::RadarSystem(const std::string& radar_port, int radar_baud, const std::string& i2c_device)
    : serial_comm_(radar_port, radar_baud),
      servo_controller_(i2c_device),
      radar_port_(radar_port),
      radar_baud_(radar_baud),
      i2c_device_(i2c_device),
      running_(false) {
}

RadarSystem::~RadarSystem() {
    // Ensure proper cleanup on destruction
    stop();
}

bool RadarSystem::initialize() {
    std::cout << "[INFO] Initializing radar system..." << std::endl;
    std::cout << "[INFO] Radar port: " << radar_port_ << " Baud rate: " << radar_baud_ << std::endl;
    std::cout << "[INFO] Servo I2C device: " << i2c_device_ << std::endl;
    
    // Register radar data callback
    serial_comm_.registerCallback(std::bind(&RadarSystem::onRadarData, this, std::placeholders::_1));
    
    // Initialize servo controller
    if (!servo_controller_.initialize()) {
        std::cerr << "[ERROR] Failed to initialize servo controller" << std::endl;
        return false;
    }
    
    // Open serial communication
    if (!serial_comm_.open()) {
        std::cerr << "[ERROR] Failed to open radar serial port" << std::endl;
        return false;
    }
    
    // Register cleanup callback
    SignalHandler::getInstance().registerCleanupCallback(std::bind(&RadarSystem::cleanup, this));
    SignalHandler::getInstance().setupSignalHandlers();
    
    std::cout << "[INFO] Radar system initialized successfully" << std::endl;
    return true;
}

bool RadarSystem::start() {
    if (running_) {
        std::cerr << "[WARNING] Radar system already running" << std::endl;
        return false;
    }
    
    // Start serial communication
    if (!serial_comm_.start()) {
        std::cerr << "[ERROR] Failed to start radar communication" << std::endl;
        return false;
    }
    
    // Start servo controller
    if (!servo_controller_.start()) {
        std::cerr << "[ERROR] Failed to start servo controller" << std::endl;
        serial_comm_.stop();
        return false;
    }
    
    running_ = true;
    std::cout << "[INFO] Radar system started" << std::endl;
    return true;
}

void RadarSystem::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "[INFO] Stopping radar system..." << std::endl;
    
    // Stop components in reverse dependency order
    // First serial communication
    serial_comm_.stop();
    
    // Then servo controller
    servo_controller_.stop();
    
    // Move servos to center position
    servo_controller_.setServoAngles(90.0f, 90.0f);
    
    // Disable all servo channels
    servo_controller_.disableAllChannels();
    
    // Reset PCA9685 to ensure clean shutdown
    servo_controller_.resetPCA9685();
    
    running_ = false;
    std::cout << "[INFO] Radar system stopped" << std::endl;
}

int RadarSystem::run() {
    // Initialize the system
    if (!initialize()) {
        std::cerr << "[ERROR] System initialization failed" << std::endl;
        return 1;
    }
    
    // Start the system
    if (!start()) {
        std::cerr << "[ERROR] System failed to start" << std::endl;
        return 1;
    }
    
    std::cout << "[INFO] System started successfully, press Ctrl+C to exit" << std::endl;
    
    // Main thread just keeps the program alive
    // The actual work is done in the component threads
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Normal exit
    stop();
    return 0;
}

void RadarSystem::cleanup() {
    std::cout << "[INFO] Performing system cleanup..." << std::endl;
    
    // First stop serial communication
    if (serial_comm_.isOpen()) {
        std::cout << "[INFO] Stopping serial communication..." << std::endl;
        serial_comm_.stop();
    }
    
    // Then handle servo controller shutdown
    std::cout << "[INFO] Stopping servo controller..." << std::endl;
    
    // Stop the servo controller thread
    servo_controller_.stop();
    
    // Set servos to center position
    std::cout << "[INFO] Setting servos to center position..." << std::endl;
    servo_controller_.setServoAngles(90.0f, 90.0f);
    
    // Ensure that PWM signals stop
    std::cout << "[INFO] Disabling all servo channels..." << std::endl;
    servo_controller_.disableAllChannels();
    
    // Reset the PCA9685 controller
    std::cout << "[INFO] Resetting PCA9685 controller..." << std::endl;
    servo_controller_.resetPCA9685();
    
    // Add a small delay to ensure I2C commands are processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    running_ = false;
    std::cout << "[INFO] System cleanup completed" << std::endl;
}

void RadarSystem::onRadarData(const RadarData& data) {
    // Print radar data
    std::cout << "[PARSED] Fault=" << static_cast<int>(data.getFault())
              << " Targets=" << static_cast<int>(data.getCount()) << std::endl;
              
    for (const auto& target : data.getTargets()) {
        std::cout << "  ID=" << static_cast<int>(target.getId())
                 << ", Dist=" << target.getDistance() << "m"
                 << ", Pitch=" << static_cast<int>(target.getPitch()) << "°"
                 << ", Yaw=" << static_cast<int>(target.getYaw()) << "°"
                 << ", X=" << target.getX() << "m"
                 << ", Y=" << target.getY() << "m" << std::endl;
    }
    
    // Find the best target (closest one) and update servo controller
    Target best_target = data.findBestTarget();
    if (best_target.isValid()) {
        servo_controller_.updateTarget(best_target);
    }
}
