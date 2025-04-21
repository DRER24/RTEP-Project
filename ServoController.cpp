#include "ServoController.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstring>

ServoController::ServoController(const std::string& i2c_device, uint8_t pca9685_addr,
                               int yaw_channel, int pitch_channel)
    : i2c_device_(i2c_device),
      pca9685_addr_(pca9685_addr),
      i2c_fd_(-1),
      yaw_channel_(yaw_channel),
      pitch_channel_(pitch_channel),
      running_(false),
      yaw_filter_(0.005f, 0.5f, 1.0f),
      pitch_filter_(0.01f, 0.1f, 1.0f),
      current_yaw_(90.0f),
      current_pitch_(90.0f) {
}

ServoController::~ServoController() {
    // Make sure to stop the thread and disable servos before destruction
    stop();
    
    // Disable all channels before closing
    disableAllChannels();
    
    // Reset the PCA9685 to ensure clean shutdown
    resetPCA9685();
    
    // Close I2C device
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool ServoController::initialize() {
    // Open I2C device
    i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "[ERROR] Cannot open I2C device: " << i2c_device_ << std::endl;
        return false;
    }
    
    // Set PCA9685 address
    if (ioctl(i2c_fd_, I2C_SLAVE, pca9685_addr_) < 0) {
        std::cerr << "[ERROR] Cannot set I2C address" << std::endl;
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }
    
    // Initialize PCA9685
    if (!initPCA9685()) {
        std::cerr << "[ERROR] Failed to initialize PCA9685" << std::endl;
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }
    
    // Set initial servo positions (center)
    setServoAngle(yaw_channel_, 90.0f);
    setServoAngle(pitch_channel_, 90.0f);
    
    std::cout << "[INFO] Servo controller initialized, facing forward" << std::endl;
    return true;
}

bool ServoController::start() {
    if (running_) {
        std::cerr << "[WARNING] Servo controller is already running" << std::endl;
        return false;
    }
    
    // Check if I2C is initialized
    if (i2c_fd_ < 0) {
        std::cerr << "[ERROR] Cannot start servo controller, I2C not initialized" << std::endl;
        return false;
    }
    
    // Start control thread
    running_ = true;
    control_thread_ = std::thread(&ServoController::controlThreadFunc, this);
    
    std::cout << "[INFO] Servo controller started" << std::endl;
    return true;
}

void ServoController::stop() {
    if (!running_) {
        return;
    }
    
    // Signal thread to stop and wait for it
    running_ = false;
    target_cv_.notify_one();
    
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // Explicitly disable both servo channels
    disableChannel(yaw_channel_);
    disableChannel(pitch_channel_);
    
    // Ensure we actually see these commands by flushing them with a small delay
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    std::cout << "[INFO] Servo controller stopped and channels disabled" << std::endl;
}

void ServoController::updateTarget(const Target& target) {
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        current_target_ = target;
    }
    target_cv_.notify_one();
}

bool ServoController::setServoAngles(float yaw_angle, float pitch_angle) {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    // Clamp angles to valid range
    if (yaw_angle < 0) yaw_angle = 0;
    if (yaw_angle > 180) yaw_angle = 180;
    if (pitch_angle < 0) pitch_angle = 0;
    if (pitch_angle > 180) pitch_angle = 180;
    
    // Update current angles
    current_yaw_ = yaw_angle;
    current_pitch_ = pitch_angle;
    
    // Set servo angles
    return setServoAngle(yaw_channel_, yaw_angle) && 
           setServoAngle(pitch_channel_, pitch_angle);
}

bool ServoController::disableAllChannels() {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    bool success = true;
    
    // Disable all 16 channels
    for (int channel = 0; channel < 16; channel++) {
        if (!disableChannel(channel)) {
            success = false;
        }
    }
    
    // Add a small delay to ensure commands complete
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    return success;
}

bool ServoController::resetPCA9685() {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    // Reset the PCA9685 by setting bit 7 of MODE1 register
    uint8_t reset_cmd[2] = {MODE1_REG, 0x80};
    if (write(i2c_fd_, reset_cmd, 2) != 2) {
        return false;
    }
    
    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Re-initialize with outputs disabled
    uint8_t sleep_cmd[2] = {MODE1_REG, 0x10};  // Put in sleep mode
    if (write(i2c_fd_, sleep_cmd, 2) != 2) {
        return false;
    }
    
    // Wait for command to take effect
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    return true;
}

void ServoController::controlThreadFunc() {
    while (running_) {
        Target t;
        {
            std::unique_lock<std::mutex> lock(target_mutex_);
            target_cv_.wait_for(lock, std::chrono::milliseconds(100));
            t = current_target_;
        }
        
        if (t.isValid()) {
            // Map radar angles to servo angles
            float target_yaw = t.getYaw();  // Modified mapping relationship
            
            // Pitch mapping
            float target_pitch = 90 + (100 - t.getPitch());  // From radar pitch to servo pitch
            
            // Apply Kalman filtering
            float filtered_yaw = yaw_filter_.update(target_yaw);
            float filtered_pitch = pitch_filter_.update(target_pitch);
            
            // Smooth movement
            const float smooth_factor = 0.5f;  // Smoothing factor, adjustable
            current_yaw_ = current_yaw_ * (1 - smooth_factor) + filtered_yaw * smooth_factor;
            current_pitch_ = current_pitch_ * (1 - smooth_factor) + filtered_pitch * smooth_factor;
            
            // Set servo angles
            setServoAngle(yaw_channel_, current_yaw_);
            setServoAngle(pitch_channel_, current_pitch_);
            
            std::cout << "[SERVO] Setting servos - Yaw: " << current_yaw_ 
                     << "° Pitch: " << current_pitch_ << "°" << std::endl;
            std::cout << "[RADAR] Target angles - Yaw: " << t.getYaw() 
                     << "° Pitch: " << t.getPitch() << "°" << std::endl;
        }
        
        // Sleep to avoid excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool ServoController::initPCA9685() {
    // Set sleep mode
    uint8_t sleep_cmd[2] = {MODE1_REG, 0x10};
    if (write(i2c_fd_, sleep_cmd, 2) != 2) {
        return false;
    }
    
    // Set prescale for 50Hz (servo frequency)
    uint8_t prescale_cmd[2] = {PRESCALE_REG, 121};  // 50Hz corresponds to prescale = 121
    if (write(i2c_fd_, prescale_cmd, 2) != 2) {
        return false;
    }
    
    // Clear sleep, set auto-increment
    uint8_t wake_cmd[2] = {MODE1_REG, 0x20};
    if (write(i2c_fd_, wake_cmd, 2) != 2) {
        return false;
    }
    
    // Wait for oscillator to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    return true;
}

bool ServoController::setServoAngle(int channel, float angle) {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    // Clamp angle to valid range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Map angle to PWM value
    int pwm_value = static_cast<int>(SERVO_MIN + (angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
    int on = 0;
    int off = pwm_value;
    
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>((on >> 8) & 0xFF),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>((off >> 8) & 0xFF)
    };
    
    // Write PWM values to PCA9685
    if (write(i2c_fd_, buffer, 5) != 5) {
        return false;
    }
    
    return true;
}

bool ServoController::disableChannel(int channel) {
    if (i2c_fd_ < 0) {
        return false;
    }
    
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg,
        0x00, 0x00,   // ON = 0
        0x00, 0x10    // OFF full ON bit (bit 4 of OFF_H = 1) = completely off
    };
    
    // Write to PCA9685 to disable the channel
    if (write(i2c_fd_, buffer, 5) != 5) {
        return false;
    }
    
    return true;
}
