#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <string>
#include <atomic>
#include "Serial.hpp"
#include "Message.hpp"

class PlmSimulator {
public:
    PlmSimulator();
    ~PlmSimulator();
    
    // Prevent copying
    PlmSimulator(const PlmSimulator&) = delete;
    PlmSimulator& operator=(const PlmSimulator&) = delete;
    
    // Run the simulator
    void run(const std::string& senderPort, const std::string& receiverPort);
    
    // Stop the simulator
    void stop();
    
    // Signal handler for graceful shutdown
    static void handleSignal(int sig);
    
private:
    // Convert transmit request to intranetwork receive indication
    void convertTxReqToRxInd(const TransmitRequest& req, IntranetworkReceive& ind);
    
    SerialPort senderPort;
    SerialPort receiverPort;
    std::atomic<bool> running;
    
    static PlmSimulator* instance;
};

#endif // SIMULATOR_HPP
