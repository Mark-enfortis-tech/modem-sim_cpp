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
    uint8_t getStartByte() const { return startByte; }      //  Command Start
    uint16_t getLength() const { return length; }           //  Length
    uint8_t getType() const { return type; }                //  Type
    uint8_t getOpcode() const { return opcode; }            //  Opcode
    uint8_t getDataService() const { return dataService; }  // Data Service Type
    uint8_t getPriority() const { return priority; }        // Priority
    uint8_t getAckService() const { return ackService; }    // ackService  TODO: missing
    uint8_t getHops() const { return hops; }                // Hops  TODO: missing
    uint8_t getGain() const { return gain; }                // Gain  TODO: missing
    uint16_t getTag() const { return tag; }                 //  Tag  TODO: this is a uint16_t
    uint8_t getEncrypt() const { return encrypt; }          // Encrypt  TODO: missing
    uint8_t getDestPort() const { return destPort; }        // Dest port TODO: missing
    uint16_t getDestAddr() const { return destAddr; }       // Dest address TODO: missing
    const std::vector<uint8_t>& getPayload() const { return payload; }  // Payload
    uint8_t getChecksum() const { return checksum; }        // Checksum  TODO: missing

    
private:
    uint8_t startByte;          //  Command Start
    uint16_t length;            //  Length
    uint8_t type;               //  Type
    uint8_t opcode;             //  Opcode
    uint8_t dataService;        // Data Service Type
    uint8_t priority;           // Priority     
    uint8_t ackService;         // ackService  TODO: missing  
    uint8_t hops;               // Hops  TODO: missing
    uint8_t gain;               // Gain  TODO: missing
    uint16_t tag;               //  Tag  TODO: this is a uint16_t
    uint8_t encrypt;            // Encrypt  TODO: missing
    uint8_t destPort;           // Dest port TODO: missing
    uint16_t destAddr;          // Dest address TODO: missing
    std::vector<uint8_t> payload;   // Payload
    uint8_t checksum;           // Checksum  TODO: missing
 
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
    uint8_t startByte;   // Command start
    uint16_t length;        // Length
    uint8_t type;           // Type
    uint8_t opcode;         // Opcode
    uint8_t status;         // Status
    uint8_t responseNumber; // Response number TODO: missing
    uint8_t result;         // Result TODO: missing             
    uint16_t tag;           // Tag TODO: should be 2B
    uint8_t checksum;       // Checksum  TODO: missing
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
    uint8_t startByte;              // Command Start 
    uint16_t length;                // Length
    uint8_t type;                   // Type
    uint8_t opcode;                 // Opcode

    uint8_t rxType;                 // RxType
    uint8_t dataService;            // Data Service Type
    uint8_t modulation;             // Modulation
    uint8_t sq;                     // Signal Quality
    uint8_t txService;              // Tx Service
    uint8_t priority;               // priority
    uint8_t cw;                     // CW
    uint8_t repeated;               // Repeated
    uint8_t txResult;               // Tx Result

    uint16_t netId;                 // Network ID
    uint16_t sourceId;              // Source ID
    uint16_t targetId;              // Target ID
    uint8_t originIdType;           // Origin ID Type
    uint16_t originId;              // Origin ID
    uint16_t finalTargetId;         // Final Target ID
    uint8_t sourcePort;             // Source/target Ports
    std::vector<uint8_t> payload;
    uint8_t checksum;       // Checksum  TODO: missing
};

#endif // MESSAGE_HPP
