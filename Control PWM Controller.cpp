#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cerrno>

static const uint8_t PCA9685_ADDR = 0x40;

static const uint8_t REG_MODE1      = 0x00;
static const uint8_t REG_MODE2      = 0x01;
static const uint8_t REG_SUBADR1    = 0x02;
static const uint8_t REG_SUBADR2    = 0x03;
static const uint8_t REG_SUBADR3    = 0x04;
static const uint8_t REG_PRESCALE   = 0xFE;
static const uint8_t LED0_ON_L      = 0x06;
static const uint8_t LED0_ON_H      = 0x07;
static const uint8_t LED0_OFF_L     = 0x08;
static const uint8_t LED0_OFF_H     = 0x09;

class I2CDev
{
public:
    I2CDev(const char* devPath, uint8_t devAddr)
        : fd_(-1), devAddr_(devAddr)
    {
        fd_ = open(devPath, O_RDWR);
        if(fd_ < 0)
        {
            std::cerr << "Turning on I2C Devices " << devPath << " Failure. " << strerror(errno) << std::endl;
            return;
        }
        if(ioctl(fd_, I2C_SLAVE, devAddr_) < 0)
        {
            std::cerr << "Setting the I2C Address " << (int)devAddr_ << " failure: " << strerror(errno) << std::endl;
            close(fd_);
            fd_ = -1;
        }
    }

    ~I2CDev()
    {
        if(fd_ >= 0) close(fd_);
    }

    bool isOpen() const { return fd_ >= 0; }

    bool writeReg(uint8_t regAddr, uint8_t value)
    {
        if(!isOpen()) return false;
        uint8_t buf[2];
        buf[0] = regAddr;
        buf[1] = value;
        int ret = ::write(fd_, buf, 2);
        return (ret == 2);
    }

    bool readReg(uint8_t regAddr, uint8_t &value)
    {
        