#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <vector>
#include <cmath>
#include <functional>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <csignal>  // Add header file for signal handling

using namespace std;
using clk = chrono::steady_clock;


/* ---------- Radar Protocol Structure ---------- */
struct Target {
    uint8_t id;
    float   dist;
    int8_t  pitch;
    int8_t  yaw;
    float   x;
    float   y;
    bool    valid;  // Indicates whether the target is valid

    // Constructor
    Target() : id(0), dist(0), pitch(0), yaw(0), x(0), y(0), valid(false) {}
};


/* ---------- Global Variables ---------- */
mutex target_mutex;
Target current_target;
atomic<bool> running{true};
condition_variable target_cv;
int i2c_fd = -1;        // Global I2C file descriptor
int serial_fd = -1;     // Global serial port file descriptor
int yaw_channel = 0;    // Horizontal servo channel
int pitch_channel = 1;  // Vertical servo channel
thread sender;          // Global send thread
thread servo_controller; // Global servo control thread

/* ---------- Remaining Code Unchanged ---------- */
// Default I2C address for PCA9685
#define PCA9685_ADDR 0x40

// Register addresses for PCA9685
#define MODE1_REG     0x00
#define PRESCALE_REG  0xFE
#define LED0_ON_L     0x06

// PWM control range
#define SERVO_MIN 150   // Corresponds to minimum angle (about 0°)
#define SERVO_MAX 600   // Corresponds to maximum angle (about 180°)



struct RadarData {
    uint8_t        fault = 0;
    uint8_t        cnt   = 0;
    vector<Target> tgts;
};

/* ---------- Kalman Filter ---------- */
class KalmanFilter {
public:
    KalmanFilter(float process_noise = 0.01f, float measurement_noise = 0.1f, float error = 1.0f) {
        Q = process_noise;       // Process noise covariance
        R = measurement_noise;   // Measurement noise covariance
        P = error;               // Estimation error covariance
        X = 0;                   // Initial state
    }

    float update(float measurement) {
        // Prediction
        P = P + Q;

        // Update
        float K = P / (P + R);  // Kalman gain
        X = X + K * (measurement - X);
        P = (1 - K) * P;

        return X;
    }

private:
    float X;  // State
    float P;  // Estimation error covariance
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
};

/* ---------- Add Signal Handler Function ---------- */
// Function to clean up and release resources
void cleanup() {
    cout << "[INFO] Performing cleanup..." << endl;

    // Set exit flag
    running = false;

    // Wait for threads to end
    if (sender.joinable()) {
        cout << "[INFO] Waiting for command sending thread to finish..." << endl;
        sender.join();
    }

    if (servo_controller.joinable()) {
        cout << "[INFO] Waiting for servo control thread to finish..." << endl;
        servo_controller.join();
    }
    void disableChannel(int file, int channel);
    // Release servo
    if (i2c_fd >= 0) {
        cout << "[INFO] Releasing servo channels..." << endl;
        disableChannel(i2c_fd, yaw_channel);
        disableChannel(i2c_fd, pitch_channel);
        close(i2c_fd);
    }

    // Close serial port
    if (serial_fd >= 0) {
        cout << "[INFO] Closing serial port device..." << endl;
        close(serial_fd);
    }

    cout << "[INFO] Cleanup complete, program exiting normally" << endl;
}

// Signal handler function
void signal_handler(int signal) {
    cout << endl << "[INFO] Received signal " << signal << ", preparing to exit program" << endl;
    cleanup();
    exit(0);  // Exit the program normally
}

/* ---------- Servo control functions ---------- */
// Initialize PCA9685 to 50Hz
void initPCA9685(int file) {
    uint8_t sleep_cmd[2] = {MODE1_REG, 0x10}; // Set to sleep mode
    write(file, sleep_cmd, 2);

    uint8_t prescale_cmd[2] = {PRESCALE_REG, 121}; // For 50Hz, prescale = 121
    write(file, prescale_cmd, 2);

    uint8_t wake_cmd[2] = {MODE1_REG, 0x20}; // Clear sleep, enable auto-increment
    write(file, wake_cmd, 2);

    usleep(500); // Short delay
}

// Set servo angle
void setServoAngle(int file, int channel, float angle) {
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

    write(file, buffer, 5);
}

// Disable channel
void disableChannel(int file, int channel) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg,
        0x00, 0x00,   // ON = 0
        0x00, 0x10    // OFF full ON bit (bit 4 of OFF_H = 1) = fully disable output
    };
    write(file, buffer, 5);
}

/* ---------- Serial port related functions ---------- */
// Open serial port
int open_serial(const string& port, int baud) {
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) { perror("open"); return -1; }

    termios t{};
    tcgetattr(fd, &t);

    speed_t s;
    switch (baud) {
        case 9600:   s = B9600;   break;
        case 19200:  s = B19200;  break;
        case 38400:  s = B38400;  break;
        case 57600:  s = B57600;  break;
        case 115200: s = B115200; break;
        default:
            cerr << "[ERROR] Unsupported baud: " << baud << "\n";
            return -1;
    }
    cfsetispeed(&t, s);
    cfsetospeed(&t, s);

    // 8E1, no flow control
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_cflag |= PARENB;
    t.c_cflag &= ~PARODD;
    t.c_cflag &= ~CSTOPB;
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    t.c_oflag &= ~OPOST;

    // Non-blocking read()
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &t);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// Convert hexadecimal string to bytes
vector<uint8_t> hex2bytes(const string& s) {
    vector<uint8_t> v;
    stringstream ss(s);
    string b;
    while (ss >> b)
        v.push_back(static_cast<uint8_t>(strtoul(b.c_str(), nullptr, 16)));
    return v;
}

// Parse a frame of data
RadarData parse_frame(const uint8_t* p, size_t len) {
    RadarData d;
    d.fault = p[4];
    d.cnt   = p[5];
    for (int i = 0; i < d.cnt; ++i) {
        int base = 4 + 8 + i * 8;
        Target t;
        t.id    = p[base];
        t.dist  = p[base + 1] * 0.1f;
        t.pitch = static_cast<int8_t>(p[base + 2]);
        t.yaw   = static_cast<int8_t>(p[base + 3]);
        t.x     = static_cast<int8_t>(p[base + 6]) * 0.1f;
        t.y     = static_cast<int8_t>(p[base + 7]) * 0.1f;
        t.valid = true;
        d.tgts.push_back(t);
    }
    return d;
}

// Print parsed result
void print_parsed(const RadarData& d) {
    cout << "[PARSED] fault=" << int(d.fault)
         << " targets=" << int(d.cnt) << "\n";
    for (const auto& t : d.tgts) {
        cout << "  ID=" << int(t.id) 
             << ", Dist=" << t.dist << "m"
             << ", Pitch=" << int(t.pitch) << "°"
             << ", Yaw=" << int(t.yaw) << "°"
             << ", X=" << t.x << "m"
             << ", Y=" << t.y << "m" << endl;
    }
}

/* ---------- Target tracking functions ---------- */
// Find the most likely human target
Target find_best_target(const RadarData& data) {
    // If no targets, return invalid target
    if (data.cnt == 0) {
        Target t;
        t.valid = false;
        return t;
    }

    
    // Simple strategy: take the nearest target (can be adjusted as needed)
auto closest = data.tgts[0];
float min_dist = closest.dist;

for (const auto& t : data.tgts) {
    if (t.dist < min_dist) {
        closest = t;
        min_dist = t.dist;
    }
}

return closest;
}

/* ---------- Main program ---------- */
int main(int argc, char* argv[]) {
    // Set signal handlers
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // kill
    signal(SIGSEGV, signal_handler);  // segmentation fault
    signal(SIGABRT, signal_handler);  // abort
    
    // —— Support command-line arguments, with default values ——
    string radar_port;
    int    radar_baud;
    string i2c_device;
    
    if (argc >= 2) {
        radar_port = argv[1];
    } else {
        radar_port = "/dev/ttyAMA0";
    }
    
    if (argc >= 3) {
        radar_baud = stoi(argv[2]);
    } else {
        radar_baud = 9600;
    }
    
    if (argc >= 4) {
        i2c_device = argv[3];
    } else {
        i2c_device = "/dev/i2c-1";
    }
    
    cout << "[INFO] Radar port: " << radar_port << " Baud rate: " << radar_baud << endl;
    cout << "[INFO] Servo I2C device: " << i2c_device << endl;
    
    // Open serial port device
    serial_fd = open_serial(radar_port, radar_baud);
    if (serial_fd == -1) {
        cerr << "[ERROR] Failed to open radar serial device" << endl;
        return 1;
    }
    cout << "[INFO] Radar serial opened successfully" << endl;
    
    // Open I2C device
    i2c_fd = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        cerr << "[ERROR] Failed to open I2C device: " << i2c_device << endl;
        close(serial_fd);
        return 1;
    }
    
    // Set PCA9685 address
    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        cerr << "[ERROR] Failed to set I2C address" << endl;
        close(serial_fd);
        close(i2c_fd);
        return 1;
    }
    
    // Initialize PWM controller
    initPCA9685(i2c_fd);
    cout << "[INFO] PCA9685 initialization complete" << endl;
    
    // Initialize servo position (both yaw and pitch set to 90 degrees, i.e., facing forward)
    setServoAngle(i2c_fd, yaw_channel, 90);
    setServoAngle(i2c_fd, pitch_channel, 90);
    cout << "[INFO] Servo initialization complete, facing forward" << endl;
    
    // Initialize Kalman filters (for yaw and pitch angles)
    KalmanFilter yaw_filter(0.005f, 0.5f, 1.0f);
    KalmanFilter pitch_filter(0.01f, 0.1f, 1.0f);
    
    // Construct and repeatedly send command frame
    const vector<uint8_t> cmd = [] {
        string s = "44 62 08 00 10 00 00 00 00 00 00 00";
        auto v = hex2bytes(s);
        uint8_t sum = 0;
        for (auto b : v) sum += b;
        v.push_back(sum);
        v.push_back(0x4B);
        return v;
    }();
    
    // Create radar command sending thread
    sender = thread([&]{
        while (running) {
            ::write(serial_fd, cmd.data(), cmd.size());
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    });
    
    // Create servo control thread
    servo_controller = thread([&]{
        float current_yaw = 90.0f;    // Current yaw angle of servo (initialized to middle position)
        float current_pitch = 90.0f;  // Current pitch angle of servo (initialized to middle position)
        
        while (running) {
            Target t;
            {
                unique_lock<mutex> lock(target_mutex);
                target_cv.wait_for(lock, chrono::milliseconds(100));
                t = current_target;
            }
            
            if (t.valid) {
                // Adjust mapping
                // Radar yaw ranges from 0-180 degrees, 90 is right, 0 is forward, 180 is back
                // Servo yaw: 90 is forward, 180 is right, 0 is left
                float target_yaw = t.yaw;  // Adjusted mapping

                // Radar pitch: 90 is horizontal, <90 is upward, >90 is downward
                // Servo pitch: 90 is forward, >90 is upward, <90 is downward
                float target_pitch = 90 + (100 - t.pitch);  // Map radar pitch to servo pitch
                
                // Apply Kalman filter
                float filtered_yaw = yaw_filter.update(target_yaw);
                float filtered_pitch = pitch_filter.update(target_pitch);
                
                // Smooth movement
                const float smooth_factor = 0.5f;  // Smoothing factor, can be adjusted
                current_yaw = current_yaw * (1 - smooth_factor) + filtered_yaw * smooth_factor;
                current_pitch = current_pitch * (1 - smooth_factor) + filtered_pitch * smooth_factor;
                
                // Set servo angles
                setServoAngle(i2c_fd, yaw_channel, current_yaw);
                setServoAngle(i2c_fd, pitch_channel, current_pitch);
                
                cout << "[SERVO] Set servo - Yaw: " << current_yaw 
                     << "° Pitch: " << current_pitch << "°" << endl;
                cout << "[RADAR] Target angle - Yaw: " << t.yaw 
                     << "° Pitch: " << t.pitch << "°" << endl;
            }
            
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    });
    
    // Receive buffer & state machine
    vector<uint8_t> rx;
    rx.reserve(1024);
    const array<uint8_t,2> SYNC{0x4D,0x62};
    auto last_data = clk::now();
    
    cout << "[INFO] System startup complete, beginning tracking..." << endl;
    cout << "[INFO] Press Ctrl+C to safely exit the program" << endl;
    
    try {
        while (running) {
            // Use select to wait up to 100ms
            fd_set rfds; 
            FD_ZERO(&rfds); 
            FD_SET(serial_fd, &rfds);
            timeval tv{0, 100000};
            
            if (select(serial_fd+1, &rfds, nullptr, nullptr, &tv) > 0 && FD_ISSET(serial_fd, &rfds)) {
                uint8_t buf[256];
                int n = ::read(serial_fd, buf, sizeof(buf));
                if (n > 0) {
                    rx.insert(rx.end(), buf, buf + n);
                    last_data = clk::now();
                }
            }
            
            // Warn if no data received for 3 seconds
            if (chrono::duration_cast<chrono::seconds>(clk::now() - last_data).count() >= 3) {
                cerr << "*** No data received for 3 seconds, check radar connection ***" << endl;
                last_data = clk::now();
            }
            
            // Frame extraction loop
            while (true) {
                auto it = search(rx.begin(), rx.end(), SYNC.begin(), SYNC.end());
                if (it == rx.end()) { rx.clear(); break; }
                rx.erase(rx.begin(), it);
                if (rx.size() < 3) break;
                uint8_t L = rx[2];
                size_t flen = L + 6;
                if (rx.size() < flen) break;
                if (rx[flen-1] != 0x4A) { rx.erase(rx.begin()); continue; }
                uint8_t sum = 0;
                for (size_t i = 0; i < flen-2; ++i) sum += rx[i];
                if (sum != rx[flen-2]) { rx.erase(rx.begin()); continue; }

                auto radar_data = parse_frame(rx.data(), flen);
                print_parsed(radar_data);
                
                // Find best target and update
                Target best_target = find_best_target(radar_data);
                if (best_target.valid) {
                    {
                        lock_guard<mutex> lock(target_mutex);
                        current_target = best_target;
                    }
                    target_cv.notify_one();
                }
                
                rx.erase(rx.begin(), rx.begin() + flen);
            }
        }
    } 
    catch (const exception& e) {
        cerr << "[ERROR] Exception: " << e.what() << endl;
        cleanup();  // Ensure cleanup even in case of exception
        return 1;
    }
    catch (...) {
        cerr << "[ERROR] Unknown exception" << endl;
        cleanup();  // Ensure cleanup even in case of exception
        return 1;
    }
    
    // Cleanup when program exits normally
    cleanup();
    
    return 0;
}
