#include "Simulator.hpp"
#include <iostream>
#include <string>

void printUsage(const std::string& programName) {
    std::cout << "Usage: " << programName << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -s, --sender PORT    Specify sender serial port (default: /dev/ttyS0)\n";
    std::cout << "  -r, --receiver PORT  Specify receiver serial port (default: /dev/ttyS1)\n";
    std::cout << "  -h, --help           Display this help message\n";
    std::cout << "\nExample:\n";
    std::cout << "  " << programName << " -s /dev/ttyUSB0 -r /dev/ttyUSB1\n";
}

int main(int argc, char *argv[]) {
    std::string senderPort = "/dev/ttyS0";
    std::string receiverPort = "/dev/ttyS1";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if ((arg == "-s" || arg == "--sender") && i + 1 < argc) {
            senderPort = argv[++i];
        } else if ((arg == "-r" || arg == "--receiver") && i + 1 < argc) {
            receiverPort = argv[++i];
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }
    
    // Create and run the simulator
    PlmSimulator simulator;
    simulator.run(senderPort, receiverPort);
    
    return 0;
}
