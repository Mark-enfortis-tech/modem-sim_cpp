#include "Message.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <atomic>

class TestReceiver {
public:
    TestReceiver() : fd(-1), running(false) {}
    
    ~TestReceiver() {
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
    
    void stop() {
        running = false;
    }
    
    void run() {
        if (fd < 0) {
            std::cerr << "Serial port not open\n";
            return;
        }
        
        running = true;
        std::cout << "Receiver running, waiting for messages...\n";
        
        while (running) {
            std::vector<uint8_t> buffer = receiveMessage();
            
            if (!buffer.empty()) {
                Message::printHexDump("Received message", buffer);
                
                // Extract payload if it's an intranetwork receive message
                if (buffer.size() >= 5 && 
                    buffer[0] == CMD_START_BYTE && 
                    buffer[3] == MSG_TYPE_INTRANETWORK_RECEIVE && 
                    buffer[4] == OPCODE_INTRANETWORK_RECEIVE) {
                    
                    std::vector<uint8_t> payload;
                    int result = extractPayload(buffer, payload);
                    
                    if (result > 0) {
                        // Try to interpret as text
                        std::string text(payload.begin(), payload.end());
                        std::cout << "Received text: " << text << std::endl;
                    }
                }
            }
            
            // Small delay to prevent CPU hogging
            usleep(10000); // 10ms
        }
        
        std::cout << "Receiver stopped\n";
    }
    
private:
    std::vector<uint8_t> receiveMessage(int timeout_ms = 100) {
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
    
    int extractPayload(const std::vector<uint8_t>& packet, std::vector<uint8_t>& payload) {
        if (packet.size() < 27) {  // Minimum packet size with empty payload
            return -1;
        }
        
        // Verify it's an intranetwork receive message
        if (packet[0] != CMD_START_BYTE || 
            packet[3] != MSG_TYPE_INTRANETWORK_RECEIVE || 
            packet[4] != OPCODE_INTRANETWORK_RECEIVE) {
            return -2;
        }
        
        // Extract length field
        uint16_t msg_len = packet[1] | (packet[2] << 8);
        
        // Verify message length
        if (packet.size() < msg_len + 4) {  // +4 for start byte, length (2 bytes), and checksum
            return -3;
        }
        
        // Verify checksum
        uint8_t expected_cs = packet[packet.size() - 1];
        uint8_t calculated_cs = Message::calculateChecksum(packet);
        if (expected_cs != calculated_cs) {
            return -4;
        }
        
        // Calculate payload length and extract payload
        int payload_len = msg_len - 23;  // 23 bytes for header fields excluding start byte and length
        if (payload_len <= 0) {
            payload.clear();
            return 0;  // No payload
        }
        
        // Copy payload (starts at offset 26)
        payload.assign(packet.begin() + 26, packet.begin() + 26 + payload_len);
        
        return payload_len;
    }
    
    int fd;
    std::atomic<bool> running;
};

// Global receiver instance for signal handler
TestReceiver* g_receiver = nullptr;

// Signal handler for graceful shutdown
void handleSignal(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down...\n";
    if (g_receiver) {
        g_receiver->stop();
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port>\n";
        std::cout << "Example: " << argv[0] << " /tmp/vserial3\n";
        return 1;
    }
    
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::string port = argv[1];
    
    std::cout << "Opening serial port " << port << "...\n";
    TestReceiver receiver;
    g_receiver = &receiver;
    
    if (!receiver.open(port)) {
        std::cerr << "Failed to open serial port " << port << "\n";
        return 1;
    }
    
    std::cout << "Serial port opened successfully\n";
    
    // Run the receiver
    receiver.run();
    
    g_receiver = nullptr;
    return 0;
}
