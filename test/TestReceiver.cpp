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
    TestReceiver() : fd(-1), running(false), currentState(State::WAIT_RX) {}
    
    ~TestReceiver() {
        close();
    }

    // Define the possible states
    enum class State {
        WAIT_RX,
        SEND_ACK,
        WAIT_ACK_RESP1,
        WAIT_ACK_RESP2
    };

    State getCurrentState() const { return currentState; };

    void setCurrentState(State newState) {
        currentState = newState;
    };

   

    std::string getStateName() const {
        switch (currentState) {
            case State::WAIT_RX: return "WAIT_RX";
            case State::SEND_ACK: return "SEND_ACK";
            case State::WAIT_ACK_RESP1: return "WAIT_ACK_RESP1";
            case State::WAIT_ACK_RESP2: return "WAIT_ACK_RESP2";
            default: return "UNKNOWN";
        }
    };

    
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
   
    
    void run() {
        if (fd < 0) {
            std::cerr << "Serial port not open\n";
            return;
        }
        
        running = true;
        std::cout << "Receiver running, waiting for message, " << getStateName() << "...\n";
        
        while (running) {
            switch(getCurrentState()){

                case State::WAIT_RX: {
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
                                setCurrentState(State::SEND_ACK);
                            } else {
                                std::cout << "Invalid extracted payload result: " << result << std::endl;
                            }
                        } else {
                            std::cout << "Invalid buffer size\n";
                        }
                    }
                }
                break;

                case State::SEND_ACK: {
                    std::cout << "State: "<< getStateName() <<"\n";
                    // Create transmit request
                    std::vector<uint8_t> buffer = createTransmitRequest(std::vector<uint8_t>({'A', 'C', 'K'}), 0x01);

                    // Send the message
                    Message::printHexDump("Sending transmit request", buffer);
                    if (!sendMessage(buffer)) {
                        std::cerr << "Failed to send message, changing to State:WAIT_RX\n";
                        setCurrentState(State::WAIT_RX);
                        std::cout << "State: "<< getStateName() <<"\n";
                    }
                    std::cout << "Sent " << buffer.size() << " bytes\n";

                    setCurrentState(State::WAIT_ACK_RESP1);
                    std::cout << "State: "<< getStateName() <<"\n";
                }
                break;

                case State::WAIT_ACK_RESP1: {

                    std::vector<uint8_t> response = receiveResponse();
                    if (!response.empty()) {
                        Message::printHexDump("Received response1", response);
                        setCurrentState(State::WAIT_ACK_RESP2);
                        std::cout << "State: "<< getStateName() <<"\n";
                    } else {
                        std::cout << "No response received or error, changing to State:WAIT_RX\n";
                        setCurrentState(State::WAIT_RX);
                        std::cout << "State: "<< getStateName() <<"\n";
                        break;
                    }
                }
                break;

                case State::WAIT_ACK_RESP2: {

                    std::vector<uint8_t> response = receiveResponse();
                    if (!response.empty()) {
                        Message::printHexDump("Received response2", response);
                        setCurrentState(State::WAIT_RX);
                        std::cout << "State: "<< getStateName() <<"\n";
                    } else {
                        std::cout << "No response received or error, changing to State:WAIT\n";
                        setCurrentState(State::WAIT_RX);
                        std::cout << "State: "<< getStateName() <<"\n";
                        break;
                    }
                }
                break;
            
            
            // Small delay to prevent CPU hogging
            usleep(100); // 10ms
            }
        }
        std::cout << "Receiver stopped\n";
    }
    
private:
    int fd;
    std::atomic<bool> running;
    State currentState;

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
        std::cout << "\nexpected_cs: " << expected_cs << ", calculated_cs: "<< calculated_cs << std::endl;
        
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
        buffer[14] = 0x00; // Target ID LSB
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


/*
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
   
*/
