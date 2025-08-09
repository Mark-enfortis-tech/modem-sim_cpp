#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <cstdint>
#include <vector>
#include <string>

// Command start byte
constexpr uint8_t CMD_START_BYTE = 0xCA;

// Max payload size per spec
constexpr size_t MAX_PAYLOAD_SIZE = 1760;

// Message types
constexpr uint8_t MSG_TYPE_TRANSMIT_REQ = 0x00;
constexpr uint8_t MSG_TYPE_TRANSMIT_RESP1 = 0x01;
constexpr uint8_t MSG_TYPE_TRANSMIT_RESP2 = 0x02;
constexpr uint8_t MSG_TYPE_INTRANETWORK_RECEIVE = 0x01;

// Message opcodes
constexpr uint8_t OPCODE_TRANSMIT_REQ = 0x60;
constexpr uint8_t OPCODE_TRANSMIT_RESP = 0x60;
constexpr uint8_t OPCODE_INTRANETWORK_RECEIVE = 0x68;

// Status codes
constexpr uint8_t STATUS_OK = 0x00;

class Message {
public:
    // Calculate XOR checksum from Type to end of Payload
    static uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    static uint8_t calculateChecksum(const uint8_t* data, int len);
    
    // Debug function to print message in hex format
    static void printHexDump(const std::string& prefix, const std::vector<uint8_t>& data);
    static void printHexDump(const std::string& prefix, const uint8_t* data, int len);
};

// Transmit Request message class
class TransmitRequest {
public:
    TransmitRequest();
    
    // Parse a transmit request message from a buffer
    bool parse(const std::vector<uint8_t>& buffer);
    bool parse(const uint8_t* buffer, int len);
    
    // Getters
    uint8_t getStartByte() const { return startByte; }
    uint16_t getLength() const { return length; }
    uint8_t getType() const { return type; }
    uint8_t getOpcode() const { return opcode; }
    uint8_t getTag() const { return tag; }
    uint8_t getTxFlags() const { return txFlags; }
    uint8_t getDataService() const { return dataService; }
    uint8_t getModulation() const { return modulation; }
    uint8_t getTxService() const { return txService; }
    uint8_t getPriority() const { return priority; }
    uint8_t getCw() const { return cw; }
    uint16_t getNetId() const { return netId; }
    uint16_t getTargetId() const { return targetId; }
    uint8_t getTargetIdType() const { return targetIdType; }
    uint8_t getSourcePort() const { return sourcePort; }
    const std::vector<uint8_t>& getPayload() const { return payload; }
    
private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t tag;
    uint8_t txFlags;
    uint8_t dataService;
    uint8_t modulation;
    uint8_t txService;
    uint8_t priority;
    uint8_t cw;
    uint16_t netId;
    uint16_t targetId;
    uint8_t targetIdType;
    uint8_t sourcePort;
    std::vector<uint8_t> payload;
};

// Transmit Response message class
class TransmitResponse {
public:
    TransmitResponse(uint8_t type = MSG_TYPE_TRANSMIT_RESP1, 
                    uint8_t status = STATUS_OK, 
                    uint8_t tag = 0);
    
    // Build a transmit response message into a buffer
    std::vector<uint8_t> build() const;
    
    // Setters
    void setType(uint8_t type) { this->type = type; }
    void setStatus(uint8_t status) { this->status = status; }
    void setTag(uint8_t tag) { this->tag = tag; }
    
private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t status;
    uint8_t tag;
};

// Intranetwork Receive Indication message class
class IntranetworkReceive {
public:
    IntranetworkReceive();
    
    // Build an intranetwork receive indication message into a buffer
    std::vector<uint8_t> build() const;
    
    // Create from a transmit request
    void createFromTransmitRequest(const TransmitRequest& req);
    
    // Setters
    void setRxFlags(uint8_t flags) { rxFlags = flags; }
    void setDataService(uint8_t service) { dataService = service; }
    void setModulation(uint8_t mod) { modulation = mod; }
    void setSq(uint8_t sq) { this->sq = sq; }
    void setTxService(uint8_t service) { txService = service; }
    void setPriority(uint8_t priority) { this->priority = priority; }
    void setCw(uint8_t cw) { this->cw = cw; }
    void setRepeated(uint8_t repeated) { this->repeated = repeated; }
    void setTxResult(uint8_t result) { txResult = result; }
    void setNetId(uint16_t id) { netId = id; }
    void setSourceId(uint16_t id) { sourceId = id; }
    void setTargetId(uint16_t id) { targetId = id; }
    void setOriginIdType(uint8_t type) { originIdType = type; }
    void setOriginId(uint16_t id) { originId = id; }
    void setFinalTargetId(uint16_t id) { finalTargetId = id; }
    void setSourcePort(uint8_t port) { sourcePort = port; }
    void setPayload(const std::vector<uint8_t>& data) { payload = data; }
    
private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t rxFlags;
    uint8_t dataService;
    uint8_t modulation;
    uint8_t sq;
    uint8_t txService;
    uint8_t priority;
    uint8_t cw;
    uint8_t repeated;
    uint8_t txResult;
    uint16_t netId;
    uint16_t sourceId;
    uint16_t targetId;
    uint8_t originIdType;
    uint16_t originId;
    uint16_t finalTargetId;
    uint8_t sourcePort;
    std::vector<uint8_t> payload;
};

#endif // MESSAGE_HPP
