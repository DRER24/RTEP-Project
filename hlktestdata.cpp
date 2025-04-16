#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>

// HLK-LD6001 expects 22-byte packets
#define PACKET_SIZE 22
#define SERIAL_PORT "/dev/ttyUSB0"

// Parse and display the presence info - simplified to just show detection type
void parsePacket(const std::vector<uint8_t>& data) {
    if (data.size() != 22 || !(data[0] == 0x4D && data[1] == 0x62 && data[2] == 0x10 && data[3] == 0x00)) {
        return;
    }
    
    uint8_t type = data[18];
    uint8_t strength = data[19];
    
    // Current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time);
    
    // Classify the detection type based on the provided switch case
    std::string label;
    switch (type) {
        case 0: label = "None"; break;
        case 1: label = "Human"; break;
        case 2: label = "Multiple People"; break;
        case 3: label = "Obstacle"; break;
        case 4: label = "Strong Object"; break;
        case 5: label = "Something Detected"; break;
        default: label = "Unknown(" + std::to_string(type) + ")"; break;
    }
    
    // Print timestamp and detection type
    std::cout << std::put_time(&tm, "[%H:%M:%S] ") 
              << "Detected: " << label 
              << ", Strength: " << static_cast<int>(strength) << std::endl;
}

// Send a command to the radar
bool sendCommand(int fd, const std::string& command) {
    write(fd, command.c_str(), command.length());
    // Simple delay to allow command processing
    usleep(200000); // 100ms
    return true;
}

// Configure and open the serial port
int setupSerial() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error opening " << SERIAL_PORT << std::endl;
        return -1;
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
    
    return fd;
}

int main() {
    int fd = setupSerial();
    if (fd == -1) return 1;
    
    // Configure the radar for faster scanning (100ms = 10 readings per second)
    sendCommand(fd, "AT+STOP\n");
    sendCommand(fd, "AT+TIME=100\n");  // 100ms scan interval
    sendCommand(fd, "AT+START\n");
    
    // Send the 14-byte init command
    const uint8_t init_cmd[] = {
        0x44, 0x62, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xBE, 0x4B
    };
    write(fd, init_cmd, sizeof(init_cmd));
    
    std::cout << "[INFO] Listening for 10 seconds...\n\n";
    
    std::vector<uint8_t> buffer;
    uint8_t byte;
    int packetsReceived = 0;
    
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::seconds>
           (std::chrono::steady_clock::now() - start).count() < 10) {
        
        // Resend command every 300ms to keep data flowing
        if (std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - start).count() % 300 == 0) {
            write(fd, init_cmd, sizeof(init_cmd));
        }
        
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
                parsePacket(buffer);
                packetsReceived++;
                buffer.clear();
            }
        } else {
            usleep(1000); // 1ms sleep
        }
    }
    
    std::cout << "\n[INFO] Finished. Received " << packetsReceived << " packets." << std::endl;
    
    // Stop the radar
    sendCommand(fd, "AT+STOP\n");
    
    close(fd);
    return 0;
}
