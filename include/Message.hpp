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
    static uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    static uint8_t calculateChecksum(const uint8_t* data, int len);

    static void printHexDump(const std::string& prefix, const std::vector<uint8_t>& data);
    static void printHexDump(const std::string& prefix, const uint8_t* data, int len);
};

// Transmit Request message class
class TransmitRequest {
public:
    TransmitRequest();
    bool parse(const std::vector<uint8_t>& buffer);
    bool parse(const uint8_t* buffer, int len);

    // Getters
    uint8_t getStartByte() const;
    uint16_t getLength() const;
    uint8_t getType() const;
    uint8_t getOpcode() const;
    uint8_t getDataService() const;
    uint8_t getPriority() const;
    uint8_t getAckService() const;
    uint8_t getHops() const;
    uint8_t getGain() const;
    uint16_t getTag() const;
    uint8_t getEncrypt() const;
    uint8_t getDestPort() const;
    uint16_t getDestAddr() const;
    const std::vector<uint8_t>& getPayload() const;
    uint8_t getChecksum() const;
    // For use by IntranetworkReceive (dummy implementations)
    uint16_t getNetId() const;
    uint16_t getTargetId() const;
    uint8_t getSourcePort() const;

private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t dataService;
    uint8_t priority;
    uint8_t ackService;
    uint8_t hops;
    uint8_t gain;
    uint16_t tag;
    uint8_t encrypt;
    uint8_t destPort;
    uint16_t destAddr;
    std::vector<uint8_t> payload;
    uint8_t checksum;
};

// Transmit Response message class
class TransmitResponse {
public:
    TransmitResponse(uint8_t type = MSG_TYPE_TRANSMIT_RESP1,
                    uint8_t status = STATUS_OK,
                    uint8_t tag = 0);

    std::vector<uint8_t> build() const;

    // Setters
    void setType(uint8_t typ);
    void setStatus(uint8_t stat);
    void setTag(uint8_t t);
    void setResponseNumber(uint8_t r);
    void setResult(uint8_t r);

private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t status;
    uint8_t responseNumber;
    uint8_t result;
    uint16_t tag;
    uint8_t checksum;
};

// Intranetwork Receive class
class IntranetworkReceive {
public:
    IntranetworkReceive();

    std::vector<uint8_t> build() const;

    void createFromTransmitRequest(const TransmitRequest& req);

    // Setters
    void setRxFlags(uint8_t flags);
    void setDataService(uint8_t service);
    void setModulation(uint8_t mod);
    void setSq(uint8_t s);
    void setTxService(uint8_t service);
    void setPriority(uint8_t p);
    void setCw(uint8_t c);
    void setRepeated(uint8_t r);
    void setTxResult(uint8_t result);
    void setNetId(uint16_t id);
    void setSourceId(uint16_t id);
    void setTargetId(uint16_t id);
    void setOriginIdType(uint8_t t);
    void setOriginId(uint16_t id);
    void setFinalTargetId(uint16_t id);
    void setSourcePort(uint8_t port);
    void setPayload(const std::vector<uint8_t>& data);

private:
    uint8_t startByte;
    uint16_t length;
    uint8_t type;
    uint8_t opcode;
    uint8_t rxType;
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
    uint8_t checksum;
};

#endif // MESSAGE_HPP