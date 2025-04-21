#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "Target.h"
#include "KalmanFilter.h"

/**
 * @class ServoController
 * @brief Controller for servo motors via PCA9685 PWM controller
 * 
 * This class provides an interface for controlling servo motors 
 * using the PCA9685 PWM controller over I2C.
 */
class ServoController {
public:
    /**
     * @brief Construct a new ServoController object
     * @param i2c_device I2C device path
     * @param pca9685_addr PCA9685 I2C address
     * @param yaw_channel Channel number for yaw servo
     * @param pitch_channel Channel number for pitch servo
     */
    explicit ServoController(const std::string& i2c_device = "/dev/i2c-1",
                           uint8_t pca9685_addr = 0x40,
                           int yaw_channel = 0,
                           int pitch_channel = 1);
    
    /**
     * @brief Destroy the ServoController object
     */
    ~ServoController();

    /**
     * @brief Initialize the servo controller
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the servo control thread
     * @return true if the thread was started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the servo control thread and disable servo outputs
     */
    void stop();
    
    /**
     * @brief Update the target to track
     * @param target New target to track
     */
    void updateTarget(const Target& target);
    
    /**
     * @brief Directly set the servo angles
     * @param yaw_angle Yaw angle in degrees (0-180)
     * @param pitch_angle Pitch angle in degrees (0-180)
     * @return true if successful, false otherwise
     */
    bool setServoAngles(float yaw_angle, float pitch_angle);
    
    /**
     * @brief Disable all servo channels
     * @return true if successful, false otherwise
     */
    bool disableAllChannels();
    
    /**
     * @brief Reset the PCA9685 controller
     * @return true if successful, false otherwise
     */
    bool resetPCA9685();

private:
    /**
     * @brief Control thread function
     */
    void controlThreadFunc();
    
    /**
     * @brief Initialize the PCA9685 PWM controller
     * @return true if successful, false otherwise
     */
    bool initPCA9685();
    
    /**
     * @brief Set angle for a specific servo
     * @param channel Servo channel
     * @param angle Angle in degrees (0-180)
     * @return true if successful, false otherwise
     */
    bool setServoAngle(int channel, float angle);
    
    /**
     * @brief Disable a specific servo channel
     * @param channel Channel to disable
     * @return true if successful, false otherwise
     */
    bool disableChannel(int channel);

    // Member variables
    std::string i2c_device_;    ///< I2C device path
    uint8_t pca9685_addr_;      ///< PCA9685 I2C address
    int i2c_fd_;                ///< I2C file descriptor
    int yaw_channel_;           ///< Channel for yaw servo
    int pitch_channel_;         ///< Channel for pitch servo
    
    std::thread control_thread_;  ///< Servo control thread
    std::atomic<bool> running_;   ///< Flag indicating if the thread is running
    
    Target current_target_;      ///< Current target to track
    std::mutex target_mutex_;    ///< Mutex for protecting target data
    std::condition_variable target_cv_; ///< Condition variable for target updates
    
    KalmanFilter yaw_filter_;    ///< Kalman filter for yaw smoothing
    KalmanFilter pitch_filter_;  ///< Kalman filter for pitch smoothing
    
    float current_yaw_;          ///< Current yaw angle
    float current_pitch_;        ///< Current pitch angle
    
    // Constants for PWM control
    static const int SERVO_MIN = 150;  ///< Minimum PWM value (0 degrees)
    static const int SERVO_MAX = 600;  ///< Maximum PWM value (180 degrees)
    
    // PCA9685 register addresses
    static const uint8_t MODE1_REG = 0x00;      ///< Mode 1 register
    static const uint8_t PRESCALE_REG = 0xFE;   ///< Prescale register
    static const uint8_t LED0_ON_L = 0x06;      ///< LED0 ON low byte register
};

#endif // SERVO_CONTROLLER_H
