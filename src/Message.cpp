#include "Message.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cstring>

// Message class implementation
uint8_t Message::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t cs = 0;
    for (size_t i = 3; i < data.size(); ++i) {
        cs ^= data[i];
    }
    return cs;
}

uint8_t Message::calculateChecksum(const uint8_t* data, int len) {
    uint8_t cs = 0;
    for (int i = 3; i < len; ++i) {
        cs ^= data[i];
    }
    return cs;
}

void Message::printHexDump(const std::string& prefix, const std::vector<uint8_t>& data) {
    std::cout << prefix << " (" << data.size() << " bytes):";
    for (size_t i = 0; i < data.size(); i++) {
        if (i % 16 == 0) std::cout << "\n" << std::hex << std::setw(4) << std::setfill('0') << i << ": ";
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

void Message::printHexDump(const std::string& prefix, const uint8_t* data, int len) {
    std::cout << prefix << " (" << len << " bytes):";
    for (int i = 0; i < len; i++) {
        if (i % 16 == 0) std::cout << "\n" << std::hex << std::setw(4) << std::setfill('0') << i << ": ";
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

// TransmitRequest class implementation
TransmitRequest::TransmitRequest() 
    : startByte(CMD_START_BYTE), length(0), type(MSG_TYPE_TRANSMIT_REQ), opcode(OPCODE_TRANSMIT_REQ),
      tag(0), txFlags(0), dataService(0), modulation(0), txService(0), priority(0), cw(0),
      netId(0), targetId(0), targetIdType(0), sourcePort(0) {
}

bool TransmitRequest::parse(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < 16 || buffer[0] != CMD_START_BYTE) {
        return false; // Invalid message
    }
    
    return parse(buffer.data(), buffer.size());
}

bool TransmitRequest::parse(const uint8_t* buffer, int len) {
    if (len < 16 || buffer[0] != CMD_START_BYTE) {
        return false; // Invalid message
    }

    // Extract length field
    uint16_t msg_len = buffer[1] | (buffer[2] << 8);
    
    // Verify message length
    if (len < msg_len + 4) { // +4 for start byte, length (2 bytes), and checksum
        return false; // Incomplete message
    }

    // Verify checksum
    uint8_t expected_cs = buffer[len - 1];
    uint8_t calculated_cs = Message::calculateChecksum(buffer, len - 1);
    if (expected_cs != calculated_cs) {
        return false; // Checksum mismatch
    }

    // Parse message fields
    startByte = buffer[0];
    length = msg_len;
    type = buffer[3];
    opcode = buffer[4];
    tag = buffer[5];
    txFlags = buffer[6];
    dataService = buffer[7];
    modulation = buffer[8];
    txService = buffer[9];
    priority = buffer[10];
    cw = buffer[11];
    netId = buffer[12] | (buffer[13] << 8);
    targetId = buffer[14] | (buffer[15] << 8);
    targetIdType = buffer[16];
    sourcePort = buffer[17];
    
    // Calculate payload length and copy payload
    size_t payload_len = msg_len - 15; // 15 bytes for header fields excluding start byte and length
    if (payload_len > MAX_PAYLOAD_SIZE) {
        payload_len = MAX_PAYLOAD_SIZE;
    }
    
    if (payload_len > 0) {
        payload.resize(payload_len);
        std::memcpy(payload.data(), &buffer[18], payload_len);
    } else {
        payload.clear();
    }
    
    return true; // Success
}

// TransmitResponse class implementation
TransmitResponse::TransmitResponse(uint8_t type, uint8_t status, uint8_t tag)
    : startByte(CMD_START_BYTE), length(5), type(type), opcode(OPCODE_TRANSMIT_RESP),
      status(status), tag(tag) {
}

std::vector<uint8_t> TransmitResponse::build() const {
    std::vector<uint8_t> buffer(8); // Fixed size for transmit response
    
    buffer[0] = startByte;
    buffer[1] = length & 0xFF;
    buffer[2] = (length >> 8) & 0xFF;
    buffer[3] = type;
    buffer[4] = opcode;
    buffer[5] = status;
    buffer[6] = tag;
    
    // Calculate and add checksum
    buffer[7] = Message::calculateChecksum(buffer);
    
    return buffer;
}

// IntranetworkReceive class implementation
IntranetworkReceive::IntranetworkReceive()
    : startByte(CMD_START_BYTE), length(0), type(MSG_TYPE_INTRANETWORK_RECEIVE),
      opcode(OPCODE_INTRANETWORK_RECEIVE), rxFlags(0), dataService(0), modulation(0),
      sq(0), txService(0), priority(0), cw(0), repeated(0), txResult(0),
      netId(0), sourceId(0), targetId(0), originIdType(0), originId(0),
      finalTargetId(0), sourcePort(0) {
}

std::vector<uint8_t> IntranetworkReceive::build() const {
    // Calculate total length
    size_t total_len = 26 + payload.size() + 1; // +1 for checksum
    std::vector<uint8_t> buffer(total_len);
    
    buffer[0] = startByte;
    
    // Length (excludes start byte, length field, and checksum)
    uint16_t length_field = 23 + payload.size(); // 23 bytes for header fields excluding start byte and length
    buffer[1] = length_field & 0xFF;
    buffer[2] = (length_field >> 8) & 0xFF;
    
    // Type & Opcode
    buffer[3] = type;
    buffer[4] = opcode;
    
    // RX Header
    buffer[5] = rxFlags;
    buffer[6] = dataService;
    buffer[7] = modulation;
    buffer[8] = sq;
    buffer[9] = txService;
    buffer[10] = priority;
    buffer[11] = cw;
    buffer[12] = repeated;
    buffer[13] = txResult;
    
    // Network and addressing fields
    buffer[14] = netId & 0xFF;
    buffer[15] = (netId >> 8) & 0xFF;
    
    buffer[16] = sourceId & 0xFF;
    buffer[17] = (sourceId >> 8) & 0xFF;
    
    buffer[18] = targetId & 0xFF;
    buffer[19] = (targetId >> 8) & 0xFF;
    
    buffer[20] = originIdType;
    
    buffer[21] = originId & 0xFF;
    buffer[22] = (originId >> 8) & 0xFF;
    
    buffer[23] = finalTargetId & 0xFF;
    buffer[24] = (finalTargetId >> 8) & 0xFF;
    
    buffer[25] = sourcePort;
    
    // Payload
    if (!payload.empty()) {
        std::memcpy(&buffer[26], payload.data(), payload.size());
    }
    
    // Checksum
    buffer[26 + payload.size()] = Message::calculateChecksum(buffer);
    
    return buffer;
}

void IntranetworkReceive::createFromTransmitRequest(const TransmitRequest& req) {
    startByte = CMD_START_BYTE;
    type = MSG_TYPE_INTRANETWORK_RECEIVE;
    opcode = OPCODE_INTRANETWORK_RECEIVE;
    
    // Set RX flags according to spec
    rxFlags = 0x02;           // bits 0:3 = 0010 (received packet destined to me)
    dataService = 0x00;       // unicast
    modulation = 0x00;        // DCSKT0
    sq = 0x1F;                // signal quality = 31d
    txService = 0x0D;         // unacknowledged packet
    priority = 0x00;          // normal
    cw = 0x00;                // default
    repeated = 0x00;          // packet was not retransmitted
    txResult = 0x00;          // result ok
    
    // Network and addressing fields
    netId = req.getNetId();
    sourceId = 0x0000;        // Source ID (simulator)
    targetId = req.getTargetId();
    originIdType = 0x00;      // short address
    originId = 0x0000;        // Origin ID
    finalTargetId = req.getTargetId();
    sourcePort = req.getSourcePort();
    
    // Copy payload
    payload = req.getPayload();
    
    // Calculate length field (excludes start byte, length field, and checksum)
    length = 23 + payload.size();
}
