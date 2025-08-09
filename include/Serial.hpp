#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <termios.h>
#include <cstdint>

// QueueMessage structure for message queue (renamed from Message)
struct QueueMessage {
    std::vector<uint8_t> data;
    
    QueueMessage() = default;
    QueueMessage(const std::vector<uint8_t>& data) : data(data) {}
};

// Thread-safe message queue
class MessageQueue {
public:
    MessageQueue(size_t maxSize = 10);
    
    bool push(const std::vector<uint8_t>& data);
    bool pop(std::vector<uint8_t>& data);
    bool isEmpty() const;
    bool isFull() const;
    size_t count() const;
    
private:
    const size_t maxQueueSize;
    std::queue<QueueMessage> messages;  // Using QueueMessage instead of Message
    mutable std::mutex queueMutex;
};

// Serial port class
class SerialPort {
public:
    SerialPort();
    ~SerialPort();
    
    // Prevent copying
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    
    // Open and close
    bool open(const std::string& devicePath, speed_t baudrate = B9600);
    void close();
    
    // Read and write
    int read(std::vector<uint8_t>& buffer, int maxLen);
    int write(const std::vector<uint8_t>& buffer);
    
    // Status
    bool isOpen() const { return fd >= 0; }
    
private:
    int fd;
};

// PLM Interface class
class PlmInterface {
public:
    PlmInterface();
    ~PlmInterface();
    
    // Prevent copying
    PlmInterface(const PlmInterface&) = delete;
    PlmInterface& operator=(const PlmInterface&) = delete;
    
    // Initialize and close
    bool init(const std::string& portPath);
    void close();
    
    // Message operations
    bool sendMessage(const std::vector<uint8_t>& data);
    bool isMessageAvailable() const;
    bool getMessage(std::vector<uint8_t>& data);
    
    // Process communication (should be called regularly)
    void process();
    
    // Queue status
    size_t txQueueCount() const;
    size_t rxQueueCount() const;
    
private:
    // Extract payload from an intranetwork receive packet
    int extractPayload(const std::vector<uint8_t>& packet, std::vector<uint8_t>& payload) const;
    
    // Build a transmit packet from payload
    std::vector<uint8_t> buildTransmitPacket(const std::vector<uint8_t>& payload, uint8_t tag) const;
    
    // Send a message from the TX queue
    bool sendMessageFromQueue();
    
    // Process incoming messages
    int processIncomingMessages();
    
    SerialPort serialPort;
    MessageQueue txQueue;
    MessageQueue rxQueue;
    uint8_t nextTag;
};

#endif // SERIAL_HPP
