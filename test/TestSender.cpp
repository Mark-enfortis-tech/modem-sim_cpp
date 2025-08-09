#include "Message.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class TestSender {
public:
    TestSender() : fd(-1) {}
    
    ~TestSender() {
        close();
    }
    
    bool open(const std::string& devicePath, speed_t baudrate = B9600) {
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
    
    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
    
    std::vector<uint8_t> createTransmitRequest(const std::string& payload, uint8_t tag) {
        return createTransmitRequest(
            std::vector<uint8_t>(payload.begin(), payload.end()), 
            tag
        );
    }
    
    std::vector<uint8_t> createTransmitRequest(const std::vector<uint8_t>& payload, uint8_t tag) {
        size_t payloadSize = std::min(payload.size(), static_cast<size_t>(MAX_PAYLOAD_SIZE));
        std::vector<uint8_t> buffer(19 + payloadSize); // Header + payload + checksum
        
        // Header
        buffer[0] = CMD_START_BYTE;
        
        // Length (excludes start byte, length field, and checksum)
        uint16_t length_field = 15 + payloadSize; // 15 bytes for header fields excluding start byte and length
        buffer[1] = length_field & 0xFF;
        buffer[2] = (length_field >> 8) & 0xFF;
        
        // Type & Opcode
        buffer[3] = MSG_TYPE_TRANSMIT_REQ;
        buffer[4] = OPCODE_TRANSMIT_REQ;
        
        // Tag
        buffer[5] = tag;
        
        // TX Header
        buffer[6] = 0x00;  // TX flags
        buffer[7] = 0x00;  // Data service
        buffer[8] = 0x00;  // Modulation
        buffer[9] = 0x0D;  // TX service
        buffer[10] = 0x00; // Priority
        buffer[11] = 0x00; // CW
        
        // Network ID
        buffer[12] = 0x01; // Network ID LSB
        buffer[13] = 0x00; // Network ID MSB
        
        // Target ID
        buffer[14] = 0x01; // Target ID LSB
        buffer[15] = 0x00; // Target ID MSB
        
        // Target ID Type
        buffer[16] = 0x00; // Short address
        
        // Source Port
        buffer[17] = 0x01;
        
        // Payload
        if (!payload.empty()) {
            std::memcpy(&buffer[18], payload.data(), payloadSize);
        }
        
        // Checksum
        buffer[18 + payloadSize] = Message::calculateChecksum(buffer);
        
        return buffer;
    }
    
    bool sendMessage(const std::vector<uint8_t>& buffer) {
        if (fd < 0) {
            return false;
        }
        
        int n = ::write(fd, buffer.data(), buffer.size());
        return (n == static_cast<int>(buffer.size()));
    }
    
    std::vector<uint8_t> receiveResponse(int timeout_ms = 1000) {
        if (fd < 0) {
            return {};
        }
        
        // Set non-blocking read with timeout
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        
        int select_result = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
        
        if (select_result <= 0) {
            return {}; // Timeout or error
        }
        
        // Data is available, read it
        std::vector<uint8_t> buffer(1024);
        int n = ::read(fd, buffer.data(), buffer.size());
        
        if (n <= 0) {
            return {};
        }
        
        buffer.resize(n);
        return buffer;
    }
    
private:
    int fd;
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port> [message]\n";
        std::cout << "Example: " << argv[0] << " /tmp/vserial1 \"Hello, World!\"\n";
        return 1;
    }
    
    std::string port = argv[1];
    std::string message = (argc > 2) ? argv[2] : "Test message from sender";
    
    std::cout << "Opening serial port " << port << "...\n";
    TestSender sender;
    
    if (!sender.open(port)) {
        std::cerr << "Failed to open serial port " << port << "\n";
        return 1;
    }
    
    std::cout << "Serial port opened successfully\n";
    
    // Create transmit request
    std::vector<uint8_t> buffer = sender.createTransmitRequest(message, 0x01);
    
    // Send the message
    Message::printHexDump("Sending transmit request", buffer);
    if (!sender.sendMessage(buffer)) {
        std::cerr << "Failed to send message\n";
        return 1;
    }
    std::cout << "Sent " << buffer.size() << " bytes\n";
    
    // Wait for responses
    std::cout << "Waiting for responses...\n";
    for (int i = 0; i < 2; i++) {
        std::vector<uint8_t> response = sender.receiveResponse();
        if (!response.empty()) {
            Message::printHexDump("Received response", response);
        } else {
            std::cout << "No response received or error\n";
            break;
        }
        sleep(1);
    }
    
    return 0;
}
