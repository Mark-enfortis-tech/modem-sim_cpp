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

    // Define the possible states
    enum class State {
        WAIT_TX,
        SEND_RESP1,
        SEND_RX,
        SEND_RESP2,
        WAIT_ACK,
        SEND_ACK_RESP1,
        SEND_ACK,
        SEND_ACK_RESP2
    };

    
    
    // Prevent copying
    PlmSimulator(const PlmSimulator&) = delete;
    PlmSimulator& operator=(const PlmSimulator&) = delete;


    std::string getStateName() const {
        switch (currentState) {
            case State::WAIT_TX: return "WAIT_TX";
            case State::SEND_RESP1: return "SEND_RESP1";
            case State::SEND_RX: return "SEND_RX";
            case State::WAIT_ACK: return "WAIT_ACK";
            case State::SEND_ACK_RESP1: return "SEND_ACK_RESP1";
            case State::SEND_ACK: return "SEND_ACK";
            case State::SEND_ACK_RESP2: return "SEND_ACK_RESP2";
            default: return "UNKNOWN";
        }
    };
    
    // Run the simulator
    void run(const std::string& senderPort, const std::string& receiverPort);
    
    // Stop the simulator
    void stop();
    
    // Signal handler for graceful shutdown
    static void handleSignal(int sig);

    State getCurrentState() const { return currentState; };
    void setCurrentState(State newState);
    std::string getStateName() const;
    
private:
    // Convert transmit request to intranetwork receive indication
    void convertTxReqToRxInd(const TransmitRequest& req, IntranetworkReceive& ind);
    
    SerialPort senderPort;
    SerialPort receiverPort;
    std::atomic<bool> running;
    
    static PlmSimulator* instance;
    State currentState;

    // Helper function to get state name
    std::string getStateName(State state) const;
};

#endif // SIMULATOR_HPP
