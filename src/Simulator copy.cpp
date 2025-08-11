#include "Simulator.hpp"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <chrono>

// Static instance pointer for signal handler
PlmSimulator* PlmSimulator::instance = nullptr;

PlmSimulator::PlmSimulator() : running(false) {
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
    std::cout << "Waiting for messages...\n";
    
    // Set running flag
    running = true;

    // for now for ack service, later be able to switch via command
    
    // Main processing loop
    while (running) {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Clean up
    std::cout << "Closing serial ports...\n";
    senderPort.close();
    receiverPort.close();
    std::cout << "Simulator shutdown complete\n";
}
