#include "Serial.hpp"
#include "Message.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>


// SerialPort implementation
SerialPort::SerialPort() : fd(-1) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& devicePath, speed_t baudrate) {
    fd = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return false;
    }

    struct termios options;
    std::memset(&options, 0, sizeof(options));

    if (tcgetattr(fd, &options) < 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    return true;
}

void SerialPort::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

int SerialPort::read(std::vector<uint8_t>& buffer, int maxLen) {
    if (fd < 0 || maxLen <= 0) {
        return -1;
    }
    
    // Set non-blocking read with timeout
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms timeout
    
    int select_result = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    
    if (select_result < 0) {
        std::cerr << "Error in select: " << strerror(errno) << std::endl;
        return -1;
    } else if (select_result == 0) {
        // Timeout, no data available
        return 0;
    }
    
    // Data is available, read it
    buffer.resize(maxLen);
    int n = ::read(fd, buffer.data(), maxLen);
    
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now
            buffer.clear();
            return 0;
        } else {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            buffer.clear();
            return -1;
        }
    }
    
    // Resize buffer to actual data size
    buffer.resize(n);
    return n;
}

int SerialPort::write(const std::vector<uint8_t>& buffer) {
    if (fd < 0 || buffer.empty()) {
        return -1;
    }
    
    return ::write(fd, buffer.data(), buffer.size());
}



