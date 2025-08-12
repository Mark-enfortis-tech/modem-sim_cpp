#include "Message.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cstring>

// --- Message ---

uint8_t Message::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t cs = 0;
    // Exclude the last byte (the checksum itself)
    for (size_t i = 3; i < data.size() - 1; ++i) {
        cs ^= data[i];
    }
    return cs;
}

uint8_t Message::calculateChecksum(const uint8_t* data, int len) {
    uint8_t cs = 0;
    // Exclude the last byte (the checksum itself)
    for (int i = 3; i < len - 1; ++i) {
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

// --- TransmitRequest ---

TransmitRequest::TransmitRequest()
    : startByte(CMD_START_BYTE), length(0), type(MSG_TYPE_TRANSMIT_REQ), opcode(OPCODE_TRANSMIT_REQ),
      dataService(0), priority(0), ackService(0), hops(0), gain(0), tag(0),
      encrypt(0), destPort(0), destAddr(0), checksum(0)
{}

bool TransmitRequest::parse(const std::vector<uint8_t>& buffer) {
    return parse(buffer.data(), static_cast<int>(buffer.size()));
}

bool TransmitRequest::parse(const uint8_t* buffer, int len) {
    if (len < 16 || buffer[0] != CMD_START_BYTE) {
        return false;
    }
    length     = buffer[1] | (buffer[2] << 8);

    // Total transmitted message size: startByte + 2 length + payload + 1 checksum
    if (len < static_cast<int>(length) + 4) {
        return false;
    }

    startByte  = buffer[0];
    type       = buffer[3];
    opcode     = buffer[4];
    dataService= buffer[5];
    priority   = buffer[6];
    ackService = buffer[7];
    hops       = buffer[8];
    gain       = buffer[9];
    tag        = buffer[10] | (buffer[11] << 8);
    encrypt    = buffer[12];
    destPort   = buffer[13];
    destAddr   = buffer[14] | (buffer[15] << 8);

    size_t headerLen = 16;
    size_t payloadLen = (length > (headerLen - 3) ? length - (headerLen - 3) : 0);
    if (payloadLen > MAX_PAYLOAD_SIZE) payloadLen = MAX_PAYLOAD_SIZE;

    if (payloadLen > 0 && (headerLen + payloadLen) <= static_cast<size_t>(len)) {
        payload.resize(payloadLen);
        std::memcpy(payload.data(), &buffer[headerLen], payloadLen);
    } else {
        payload.clear();
    }

    checksum = buffer[headerLen + payloadLen];

    if (checksum != Message::calculateChecksum(buffer, static_cast<int>(headerLen + payloadLen))) {
        return false;
    }
    return true;
}

uint8_t  TransmitRequest::getStartByte()  const { return startByte; }
uint16_t TransmitRequest::getLength()     const { return length; }
uint8_t  TransmitRequest::getType()       const { return type; }
uint8_t  TransmitRequest::getOpcode()     const { return opcode; }
uint8_t  TransmitRequest::getDataService() const { return dataService; }
uint8_t  TransmitRequest::getPriority()   const { return priority; }
uint8_t  TransmitRequest::getAckService() const { return ackService; }
uint8_t  TransmitRequest::getHops()       const { return hops; }
uint8_t  TransmitRequest::getGain()       const { return gain; }
uint16_t TransmitRequest::getTag()        const { return tag; }
uint8_t  TransmitRequest::getEncrypt()    const { return encrypt; }
uint8_t  TransmitRequest::getDestPort()   const { return destPort; }
uint16_t TransmitRequest::getDestAddr()   const { return destAddr; }
const std::vector<uint8_t>& TransmitRequest::getPayload() const { return payload; }
uint8_t  TransmitRequest::getChecksum()   const { return checksum; }
uint16_t TransmitRequest::getNetId() const { return 0; }
uint16_t TransmitRequest::getTargetId() const { return destAddr; }
uint8_t  TransmitRequest::getSourcePort() const { return destPort; }

// --- TransmitResponse ---

TransmitResponse::TransmitResponse(uint8_t type, uint8_t status, uint8_t tag)
    : startByte(CMD_START_BYTE), length(7), type(type), opcode(OPCODE_TRANSMIT_RESP),
      status(status), responseNumber(0), result(0), tag(tag), checksum(0)
{}

std::vector<uint8_t> TransmitResponse::build() const {
    std::vector<uint8_t> buffer(11);
    buffer[0] = startByte;
    buffer[1] = length & 0xFF;
    buffer[2] = (length >> 8) & 0xFF;
    buffer[3] = type;
    buffer[4] = opcode;
    buffer[5] = status;
    buffer[6] = responseNumber;
    buffer[7] = result;
    buffer[8] = tag & 0xFF;
    buffer[9] = (tag >> 8) & 0xFF;
    buffer[10] = Message::calculateChecksum(buffer);
    return buffer;
}

void TransmitResponse::setType(uint8_t typ)      { this->type = typ; }
void TransmitResponse::setStatus(uint8_t stat)   { this->status = stat; }
void TransmitResponse::setTag(uint8_t t)         { this->tag = t; }
void TransmitResponse::setResponseNumber(uint8_t r) { this->responseNumber = r; }
void TransmitResponse::setResult(uint8_t r)      { this->result = r; }

// --- IntranetworkReceive ---

IntranetworkReceive::IntranetworkReceive()
    : startByte(CMD_START_BYTE), length(0), type(MSG_TYPE_INTRANETWORK_RECEIVE),
      opcode(OPCODE_INTRANETWORK_RECEIVE), rxType(0), dataService(0), modulation(0),
      sq(0), txService(0), priority(0), cw(0), repeated(0), txResult(0),
      netId(0), sourceId(0), targetId(0), originIdType(0), originId(0),
      finalTargetId(0), sourcePort(0), checksum(0)
{}

std::vector<uint8_t> IntranetworkReceive::build() const {
    size_t headerLen = 26;
    size_t totalLen = headerLen + payload.size() + 1;
    std::vector<uint8_t> buffer(totalLen);

    buffer[0] = startByte;
    uint16_t length_field = static_cast<uint16_t>(headerLen - 3 + payload.size());
    buffer[1] = length_field & 0xFF;
    buffer[2] = (length_field >> 8) & 0xFF;
    buffer[3] = type;
    buffer[4] = opcode;
    buffer[5] = rxType;
    buffer[6] = dataService;
    buffer[7] = modulation;
    buffer[8] = sq;
    buffer[9] = txService;
    buffer[10] = priority;
    buffer[11] = cw;
    buffer[12] = repeated;
    buffer[13] = txResult;
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
    if (!payload.empty())
        std::memcpy(&buffer[26], payload.data(), payload.size());

    // Calculate and add checksum locally (not modifying the member variable)
    uint8_t local_checksum = Message::calculateChecksum(buffer);
    buffer[26 + payload.size()] = local_checksum;

    return buffer;
}

void IntranetworkReceive::createFromTransmitRequest(const TransmitRequest& req) {
    startByte = CMD_START_BYTE;
    type = MSG_TYPE_INTRANETWORK_RECEIVE;
    opcode = OPCODE_INTRANETWORK_RECEIVE;
    rxType = 0x02;
    dataService = 0x00;
    modulation = 0x00;
    sq = 0x1F;
    txService = 0x0D;
    priority = 0x00;
    cw = 0x00;
    repeated = 0x00;
    txResult = 0x00;
    netId = 0;
    sourceId = 0x0000;
    targetId = req.getTargetId();
    originIdType = 0x00;
    originId = 0x0000;
    finalTargetId = req.getTargetId();
    sourcePort = req.getSourcePort();
    payload = req.getPayload();
    length = 23 + payload.size();
}

void IntranetworkReceive::setRxFlags(uint8_t flags)         { rxType = flags; }
void IntranetworkReceive::setDataService(uint8_t service)   { dataService = service; }
void IntranetworkReceive::setModulation(uint8_t mod)        { modulation = mod; }
void IntranetworkReceive::setSq(uint8_t s)                  { sq = s; }
void IntranetworkReceive::setTxService(uint8_t service)     { txService = service; }
void IntranetworkReceive::setPriority(uint8_t p)            { priority = p; }
void IntranetworkReceive::setCw(uint8_t c)                  { cw = c; }
void IntranetworkReceive::setRepeated(uint8_t r)            { repeated = r; }
void IntranetworkReceive::setTxResult(uint8_t result)       { txResult = result; }
void IntranetworkReceive::setNetId(uint16_t id)             { netId = id; }
void IntranetworkReceive::setSourceId(uint16_t id)          { sourceId = id; }
void IntranetworkReceive::setTargetId(uint16_t id)          { targetId = id; }
void IntranetworkReceive::setOriginIdType(uint8_t t)        { originIdType = t; }
void IntranetworkReceive::setOriginId(uint16_t id)          { originId = id; }
void IntranetworkReceive::setFinalTargetId(uint16_t id)     { finalTargetId = id; }
void IntranetworkReceive::setSourcePort(uint8_t port)       { sourcePort = port; }
void IntranetworkReceive::setPayload(const std::vector<uint8_t>& data) { payload = data; }