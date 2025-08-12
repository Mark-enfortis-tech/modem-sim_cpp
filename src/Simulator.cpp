#include "Simulator.hpp"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <chrono>

// Static instance pointer for signal handler
PlmSimulator* PlmSimulator::instance = nullptr;

PlmSimulator::PlmSimulator() : running(false), currentState(State::WAIT_TX) {
    // Store instance pointer for signal handler
    instance = this;
}

PlmSimulator::~PlmSimulator() {
    stop();
    instance = nullptr;
}

void PlmSimulator::handleSignal(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down...\n";
    if (instance) {
        instance->stop();
    }
}

void PlmSimulator::stop() {
    running = false;
}


void PlmSimulator::convertTxReqToRxInd(const TransmitRequest& req, IntranetworkReceive& ind) {
    ind.createFromTransmitRequest(req);
}

void PlmSimulator::setCurrentState(State newState) {

    // Update the state
    currentState = newState;
}

std::string PlmSimulator::getStateName() const {
    switch (currentState) {
        case State::WAIT_TX: return "WAIT_TX";
        case State::SEND_RESP1: return "SEND_RESP1";
        case State::SEND_RX: return "SEND_RX";
        case State::SEND_RESP2: return "SEND_RESP2";
        case State::WAIT_ACK: return "WAIT_ACK";
        case State::SEND_ACK_RESP1: return "SEND_ACK_RESP1";
        case State::SEND_ACK_RX: return "SEND_ACK_RX";
        case State::SEND_ACK_RESP2: return "SEND_ACK_RESP2";
        default: return "UNKNOWN";
    }
}


void PlmSimulator::run(const std::string& senderPortPath, const std::string& receiverPortPath) {
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    std::cout << "Power Line Modem Simulator\n";
    std::cout << "Sender port: " << senderPortPath << "\n";
    std::cout << "Receiver port: " << receiverPortPath << "\n";
    
    // Open serial ports
    if (!senderPort.open(senderPortPath)) {
        std::cerr << "Failed to open sender port " << senderPortPath << "\n";
        return;
    }
    
    if (!receiverPort.open(receiverPortPath)) {
        std::cerr << "Failed to open receiver port " << receiverPortPath << "\n";
        senderPort.close();
        return;
    }
    
    std::cout << "Serial ports opened successfully\n";
    std::cout << "Waiting for message, " << getStateName() << "...\n";
    
    // Set running flag
    running = true;
    TransmitRequest req;
    IntranetworkReceive rxInd;

    // for now for ack service, later be able to switch via command
    
    // Main processing loop
    while (running) {

        switch(getCurrentState()){
            case State::WAIT_TX:    // wait for transmission
            
                std::vector<uint8_t> buffer;
                int n = senderPort.read(buffer, MAX_PAYLOAD_SIZE + 100);
                if (n > 0) {
                    // Check if it's a valid message
                    if (!buffer.empty() && buffer[0] == CMD_START_BYTE) {
                        Message::printHexDump("Received from sender", buffer);
                        
                        // Parse the transmit request
                        bool result = req.parse(buffer);
                        
                        if (result && req.getType() == MSG_TYPE_TRANSMIT_REQ && req.getOpcode() == OPCODE_TRANSMIT_REQ) {
                            std::cout << "Valid transmit request received, tag: " 
                                    << std::hex << std::setw(2) << std::setfill('0') 
                                    << static_cast<int>(req.getTag()) << std::dec << "\n";
                            setCurrentState(State::SEND_RESP1);
                            }
                        }
                    }
            break;

            case State::SEND_RESP1: // send response 1 to sender
                std::cout << "State::SEND_RESP1\n";

                TransmitResponse resp1(MSG_TYPE_TRANSMIT_RESP1, STATUS_OK, req.getTag());
                std::vector<uint8_t> respBuffer = resp1.build();
                
                if (!respBuffer.empty()) {
                    Message::printHexDump("Sending response 1", respBuffer);
                    senderPort.write(respBuffer);
                    setCurrentState(State::SEND_RX);
                } else {
                    std::cout << "Error in respBuffer, State::SEND_RESP1, changing to State::WAIT_TX\n";
                    setCurrentState(State::WAIT_TX);
                }
            break;

            case State::SEND_RX:    // send message to receiver
                std::cout << "State::SEND_RX\n";

                // Convert to intranetwork receive indication
                convertTxReqToRxInd(req, rxInd);
                        
                // Build and send intranetwork receive indication to receiver
                std::vector<uint8_t> indBuffer = rxInd.build();
                if (!indBuffer.empty()) {
                        Message::printHexDump("Sending intranetwork receive indication", indBuffer);
                        receiverPort.write(indBuffer);
                        setCurrentState(State::SEND_RESP2);
                } else {
                    std::cout << "Error in indBuffer, State::SEND_RX, changing to State::WAIT_TX\n";
                    setCurrentState(State::WAIT_TX);
                }; 
            break;

            case State::SEND_RESP2: // send response 2 to sender
                std::cout << "State::SEND_RESP2\n";
                
                // Send second response (transmission complete)
                TransmitResponse resp2(MSG_TYPE_TRANSMIT_RESP2, STATUS_OK, req.getTag());
                std::vector<uint8_t> respBuffer = resp2.build();
                
                if (!respBuffer.empty()) {
                    Message::printHexDump("Sending response 2", respBuffer);
                    senderPort.write(respBuffer);
                    std::cout << "Waiting for ACK, State::WAIT_ACK...\n";
                    setCurrentState(State::WAIT_ACK)
                } else {
                    std::cout << "Error in respBuffer, State::SEND_RESP2, changing to State::WAIT_TX\n";
                    setCurrentState(State::WAIT_TX);
                };
            break;

            case State::WAIT_ACK:  // wait for ack message
            
            std::cout << "Recieved ACK\n";
            setCurrentState(State::SEND_ACK_RESP1);
            break;

            case State::SEND_ACK_RESP1:
            // send ack response 1 to receiver
            std::cout << "State::SEND_ACK_RESP1\n";
            setCurrentState(State::SEND_ACK_RX);
            break;

            case State::SEND_ACK_RX:
            // send ack message to sender
            std::cout << "State::SEND_ACK\n";
            setCurrentState(State::SEND_ACK_RESP2);
            break;

            case State::SEND_ACK_RESP2:
            // send ack response 2 to sender
            std::cout << "State::SEND_ACK_RESP2\n";
            std::cout << "State::WAIT_TX\n";
            setCurrentState(State::WAIT_TX);
            break;
        }
        // Read from sender port
        std::vector<uint8_t> buffer;
        int n = senderPort.read(buffer, MAX_PAYLOAD_SIZE + 100);
        
        if (n > 0) {
            // Check if it's a valid message
            if (!buffer.empty() && buffer[0] == CMD_START_BYTE) {
                Message::printHexDump("Received from sender", buffer);
                
                // Parse the transmit request
                TransmitRequest req;
                bool result = req.parse(buffer);
                
                if (result && req.getType() == MSG_TYPE_TRANSMIT_REQ && req.getOpcode() == OPCODE_TRANSMIT_REQ) {
                    std::cout << "Valid transmit request received, tag: " 
                              << std::hex << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(req.getTag()) << std::dec << "\n";
                    
                    // Send first response (acknowledge receipt)
                    TransmitResponse resp1(MSG_TYPE_TRANSMIT_RESP1, STATUS_OK, req.getTag());
                    std::vector<uint8_t> respBuffer = resp1.build();
                    
                    if (!respBuffer.empty()) {
                        Message::printHexDump("Sending response 1", respBuffer);
                        senderPort.write(respBuffer);
                    }
                    
                    // Convert to intranetwork receive indication
                    IntranetworkReceive rxInd;
                    convertTxReqToRxInd(req, rxInd);
                    
                    // Build and send intranetwork receive indication to receiver
                    std::vector<uint8_t> indBuffer = rxInd.build();
                    
                    if (!indBuffer.empty()) {
                        Message::printHexDump("Sending intranetwork receive indication", indBuffer);
                        receiverPort.write(indBuffer);
                        
                        // Send second response (transmission complete)
                        TransmitResponse resp2(MSG_TYPE_TRANSMIT_RESP2, STATUS_OK, req.getTag());
                        respBuffer = resp2.build();
                        
                        if (!respBuffer.empty()) {
                            Message::printHexDump("Sending response 2", respBuffer);
                            senderPort.write(respBuffer);
                        }
                    }
                } else {
                    std::cout << "Invalid transmit request received\n";
                }
            }


        }
        
        // Small delay to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Clean up
    std::cout << "Closing serial ports...\n";
    senderPort.close();
    receiverPort.close();
    std::cout << "Simulator shutdown complete\n";
}
