#include "Serial.hpp"
#include "Message.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>

// MessageQueue implementation
MessageQueue::MessageQueue(size_t maxSize) : maxQueueSize(maxSize) {
}

bool MessageQueue::push(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(queueMutex);
    
    if (messages.size() >= maxQueueSize) {
        return false;
    }
    
    messages.push(QueueMessage(data));  // Changed from Message to QueueMessage
    return true;
}

bool MessageQueue::pop(std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(queueMutex);
    
    if (messages.empty()) {
        return false;
    }
    
    data = messages.front().data;
    messages.pop();
    return true;
}

bool MessageQueue::isEmpty() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    return messages.empty();
}

bool MessageQueue::isFull() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    return messages.size() >= maxQueueSize;
}

size_t MessageQueue::count() const {
    std::lock_guard<std::mutex> lock(queueMutex);
    return messages.size();
}

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

// PlmInterface implementation
PlmInterface::PlmInterface() : nextTag(0) {
}

PlmInterface::~PlmInterface() {
    close();
}

bool PlmInterface::init(const std::string& portPath) {
    if (!serialPort.open(portPath)) {
        std::cerr << "Failed to open serial port " << portPath << std::endl;
        return false;
    }
    
    std::cout << "PLM interface initialized on port " << portPath << std::endl;
    return true;
}

void PlmInterface::close() {
    serialPort.close();
}

bool PlmInterface::sendMessage(const std::vector<uint8_t>& data) {
    return txQueue.push(data);
}

bool PlmInterface::isMessageAvailable() const {
    return !rxQueue.isEmpty();
}

bool PlmInterface::getMessage(std::vector<uint8_t>& data) {
    return rxQueue.pop(data);
}

void PlmInterface::process() {
    // Process incoming messages
    processIncomingMessages();
    
    // Send messages from queue
    if (!txQueue.isEmpty()) {
        sendMessageFromQueue();
    }
}

size_t PlmInterface::txQueueCount() const {
    return txQueue.count();
}

size_t PlmInterface::rxQueueCount() const {
    return rxQueue.count();
}

int PlmInterface::extractPayload(const std::vector<uint8_t>& packet, std::vector<uint8_t>& payload) const {
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

std::vector<uint8_t> PlmInterface::buildTransmitPacket(const std::vector<uint8_t>& payload, uint8_t tag) const {
    std::vector<uint8_t> packet;
    
    if (payload.empty()) {
        return packet;  // Return empty packet on error
    }
    
    // Reserve space for the packet
    packet.resize(19 + payload.size());
    
    // Header
    packet[0] = CMD_START_BYTE;
    
    // Length (excludes start byte, length field, and checksum)
    uint16_t length_field = 15 + payload.size();  // 15 bytes for header fields excluding start byte and length
    packet[1] = length_field & 0xFF;
    packet[2] = (length_field >> 8) & 0xFF;
    
    // Type & Opcode
    packet[3] = MSG_TYPE_TRANSMIT_REQ;
    packet[4] = OPCODE_TRANSMIT_REQ;
    
    // Tag
    packet[5] = tag;
    
    // TX Header
    packet[6] = 0x00;  // TX flags
    packet[7] = 0x00;  // Data service
    packet[8] = 0x00;  // Modulation
    packet[9] = 0x0D;  // TX service
    packet[10] = 0x00; // Priority
    packet[11] = 0x00; // CW
    
    // Network ID
    packet[12] = 0x01; // Network ID LSB
    packet[13] = 0x00; // Network ID MSB
    
    // Target ID
    packet[14] = 0x01; // Target ID LSB
    packet[15] = 0x00; // Target ID MSB
    
    // Target ID Type
    packet[16] = 0x00; // Short address
    
    // Source Port
    packet[17] = 0x01;
    
    // Payload
    if (!payload.empty()) {
        std::copy(payload.begin(), payload.end(), packet.begin() + 18);
    }
    
    // Checksum
    packet[18 + payload.size()] = Message::calculateChecksum(packet);
    
    return packet;
}

bool PlmInterface::sendMessageFromQueue() {
    if (txQueue.isEmpty() || !serialPort.isOpen()) {
        return false;
    }
    
    std::vector<uint8_t> payload;
    
    if (txQueue.pop(payload)) {
        // Build transmit packet
        std::vector<uint8_t> packet = buildTransmitPacket(payload, nextTag++);
        
        if (!packet.empty()) {
            // Send the packet
            int n = serialPort.write(packet);
            
            // Debug output
            std::cout << "Sent packet (" << n << " bytes)" << std::endl;
            Message::printHexDump("Sent packet", packet);
            
            return (n == static_cast<int>(packet.size()));
        }
    }
    
    return false;
}

int PlmInterface::processIncomingMessages() {
    if (!serialPort.isOpen()) {
        return -1;
    }
    
    std::vector<uint8_t> buffer;
    int n = serialPort.read(buffer, 2048);  // Use a reasonable buffer size
    
    // If we received data and it looks like a valid message, process it
    if (n > 0) {
        // Debug output
        std::cout << "Received data (" << n << " bytes)" << std::endl;
        Message::printHexDump("Received data", buffer);
        
        if (!buffer.empty() && buffer[0] == CMD_START_BYTE) {
            // Check if it's a transmit response
            if (buffer.size() >= 8 && 
                (buffer[3] == MSG_TYPE_TRANSMIT_RESP1 || buffer[3] == MSG_TYPE_TRANSMIT_RESP2) && 
                buffer[4] == OPCODE_TRANSMIT_RESP) {
                
                std::cout << "Received transmit response, status: " 
                          << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[5])
                          << ", tag: " 
                          << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[6])
                          << std::dec << std::endl;
            }
            // Check if it's an intranetwork receive message
            else if (buffer.size() >= 5 && buffer[3] == MSG_TYPE_INTRANETWORK_RECEIVE && 
                     buffer[4] == OPCODE_INTRANETWORK_RECEIVE) {
                
                std::vector<uint8_t> payload;
                int payload_len = extractPayload(buffer, payload);
                
                if (payload_len > 0) {
                    std::cout << "Extracted payload (" << payload_len << " bytes)" << std::endl;
                    Message::printHexDump("Extracted payload", payload);
                    
                    rxQueue.push(payload);
                    return payload_len;
                }
            }
        }
    }
    
    return n;
}
