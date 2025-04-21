#include "SerialComm.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>

// Initialize static constants
const std::array<uint8_t, 2> SerialComm::SYNC_BYTES = {0x4D, 0x62};

SerialComm::SerialComm(const std::string& port, int baud)
    : port_(port),
      baud_(baud),
      serial_fd_(-1),
      running_(false),
      last_data_time_(std::chrono::steady_clock::now()) {
    
    // Pre-allocate receive buffer
    receive_buffer_.reserve(1024);
    
    // Initialize the command frame
    command_ = hexToBytes("44 62 08 00 10 00 00 00 00 00 00 00");
    
    // Calculate and append checksum
    uint8_t sum = 0;
    for (auto b : command_) {
        sum += b;
    }
    command_.push_back(sum);
    command_.push_back(0x4B);  // End byte
}

SerialComm::~SerialComm() {
    stop();
    close();
}

bool SerialComm::open() {
    if (isOpen()) {
        return true;  // Already open
    }
    
    serial_fd_ = openSerial(port_, baud_);
    return (serial_fd_ >= 0);
}

void SerialComm::close() {
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool SerialComm::isOpen() const {
    return (serial_fd_ >= 0);
}

bool SerialComm::start() {
    if (!isOpen()) {
        std::cerr << "[ERROR] Cannot start communications, serial port not open" << std::endl;
        return false;
    }
    
    if (running_) {
        std::cerr << "[WARNING] Communications already running" << std::endl;
        return false;
    }
    
    // Start sender and receiver threads
    running_ = true;
    sender_thread_ = std::thread(&SerialComm::senderThreadFunc, this);
    receiver_thread_ = std::thread(&SerialComm::receiverThreadFunc, this);
    
    std::cout << "[INFO] Serial communications started" << std::endl;
    return true;
}

void SerialComm::stop() {
    if (!running_) {
        return;
    }
    
    // Signal threads to stop and wait for them
    running_ = false;
    
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }
    
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    std::cout << "[INFO] Serial communications stopped" << std::endl;
}

void SerialComm::registerCallback(RadarDataCallback callback) {
    callback_ = callback;
}

std::vector<uint8_t> SerialComm::hexToBytes(const std::string& s) {
    std::vector<uint8_t> v;
    std::stringstream ss(s);
    std::string b;
    
    while (ss >> b) {
        v.push_back(static_cast<uint8_t>(std::stoul(b, nullptr, 16)));
    }
    
    return v;
}

void SerialComm::senderThreadFunc() {
    while (running_) {
        // Send command to radar
        if (serial_fd_ >= 0) {
            ::write(serial_fd_, command_.data(), command_.size());
        }
        
        // Send commands every 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void SerialComm::receiverThreadFunc() {
    while (running_) {
        // Wait for data using select() with timeout
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(serial_fd_, &rfds);
        
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms timeout
        
        if (select(serial_fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0 && FD_ISSET(serial_fd_, &rfds)) {
            // Data available, read it
            uint8_t buf[256];
            int n = ::read(serial_fd_, buf, sizeof(buf));
            
            if (n > 0) {
                // Append data to receive buffer
                receive_buffer_.insert(receive_buffer_.end(), buf, buf + n);
                last_data_time_ = std::chrono::steady_clock::now();
            }
        }
        
        // Check if we haven't received data for a while
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_data_time_).count() >= 3) {
            std::cerr << "*** 3 seconds without data, check radar connection ***" << std::endl;
            last_data_time_ = now;  // Reset to avoid repeated messages
        }
        
        // Process received data, extract frames
        while (true) {
            // Find sync bytes
            auto it = std::search(receive_buffer_.begin(), receive_buffer_.end(), 
                                 SYNC_BYTES.begin(), SYNC_BYTES.end());
            
            if (it == receive_buffer_.end()) {
                receive_buffer_.clear();
                break;
            }
            
            // Remove data before sync bytes
            receive_buffer_.erase(receive_buffer_.begin(), it);
            
            // Check if we have enough data for header
            if (receive_buffer_.size() < 3) {
                break;
            }
            
            // Get frame length from header
            uint8_t length = receive_buffer_[2];
            size_t frame_len = length + 6;
            
            // Check if we have complete frame
            if (receive_buffer_.size() < frame_len) {
                break;
            }
            
            // Check end byte
            if (receive_buffer_[frame_len - 1] != 0x4A) {
                receive_buffer_.erase(receive_buffer_.begin());
                continue;
            }
            
            // Verify checksum
            uint8_t sum = 0;
            for (size_t i = 0; i < frame_len - 2; ++i) {
                sum += receive_buffer_[i];
            }
            
            if (sum != receive_buffer_[frame_len - 2]) {
                receive_buffer_.erase(receive_buffer_.begin());
                continue;
            }
            
            // Valid frame, parse it
            RadarData radar_data;
            radar_data.parseFrame(receive_buffer_.data(), frame_len);
            
            // Call callback if registered
            if (callback_) {
                callback_(radar_data);
            }
            
            // Remove processed frame from buffer
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frame_len);
        }
    }
}

int SerialComm::openSerial(const std::string& port, int baud) {
    // Open serial port
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "[ERROR] Cannot open serial port: " << port << std::endl;
        return -1;
    }
    
    // Configure serial port
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        std::cerr << "[ERROR] Cannot get serial port attributes" << std::endl;
        ::close(fd);
        return -1;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            std::cerr << "[ERROR] Unsupported baud rate: " << baud << std::endl;
            ::close(fd);
            return -1;
    }
    
    cfsetispeed(&t, speed);
    cfsetospeed(&t, speed);
    
    // 8E1, no flow control
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_cflag |= PARENB;
    t.c_cflag &= ~PARODD;
    t.c_cflag &= ~CSTOPB;
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    t.c_oflag &= ~OPOST;
    
    // Non-blocking read
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        std::cerr << "[ERROR] Cannot set serial port attributes" << std::endl;
        ::close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    return fd;
}
