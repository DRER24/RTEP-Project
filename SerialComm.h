#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <chrono>
#include "RadarData.h"

/**
 * @class SerialComm
 * @brief Class for serial port communication with the radar
 * 
 * This class handles opening, reading from, and writing to the serial port
 * to communicate with the radar device.
 */
class SerialComm {
public:
    /**
     * @brief Callback function type for radar data
     */
    using RadarDataCallback = std::function<void(const RadarData&)>;
    
    /**
     * @brief Construct a new SerialComm object
     * @param port Serial port device path
     * @param baud Baud rate
     */
    explicit SerialComm(const std::string& port = "/dev/ttyAMA0", int baud = 9600);
    
    /**
     * @brief Destroy the SerialComm object
     */
    ~SerialComm();
    
    /**
     * @brief Open the serial port
     * @return true if successful, false otherwise
     */
    bool open();
    
    /**
     * @brief Close the serial port
     */
    void close();
    
    /**
     * @brief Check if the serial port is open
     * @return true if open, false otherwise
     */
    bool isOpen() const;
    
    /**
     * @brief Start reading and processing data
     * @return true if started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop reading and processing data
     */
    void stop();
    
    /**
     * @brief Register a callback for radar data
     * @param callback Callback function to call when radar data is available
     */
    void registerCallback(RadarDataCallback callback);
    
    /**
     * @brief Convert a hex string to bytes
     * @param s Hex string with space-separated values
     * @return Vector of bytes
     */
    static std::vector<uint8_t> hexToBytes(const std::string& s);

private:
    /**
     * @brief Sender thread function
     */
    void senderThreadFunc();
    
    /**
     * @brief Receiver thread function
     */
    void receiverThreadFunc();
    
    /**
     * @brief Open the serial port with specified settings
     * @param port Port device path
     * @param baud Baud rate
     * @return File descriptor or -1 on error
     */
    int openSerial(const std::string& port, int baud);

    // Member variables
    std::string port_;      ///< Serial port path
    int baud_;              ///< Baud rate
    int serial_fd_;         ///< Serial port file descriptor
    
    std::atomic<bool> running_;  ///< Flag indicating if threads are running
    std::thread sender_thread_;  ///< Thread for sending commands
    std::thread receiver_thread_; ///< Thread for receiving data
    
    std::vector<uint8_t> command_; ///< Command to send to the radar
    std::vector<uint8_t> receive_buffer_; ///< Buffer for received data
    
    RadarDataCallback callback_; ///< Callback for radar data
    
    std::chrono::steady_clock::time_point last_data_time_; ///< Time of last received data
    
    // Constants for frame detection
    static const std::array<uint8_t, 2> SYNC_BYTES;  ///< Sync bytes for frame detection
};

#endif // SERIAL_COMM_H
