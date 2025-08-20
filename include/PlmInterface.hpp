#ifndef PLMINTERFACE_HPP
#define PLMINTERFACE_HPP

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "PlmTxHandler.hpp"
#include "PlmRxHandler.hpp"
#include "json.hpp"

// For convenience
using json = nlohmann::json;
using MessageBuffer = std::vector<uint8_t>;

class PlmInterface {
public:
    PlmInterface(const std::string& configFile = "config.json") 
        : configFilePath(configFile), isRunning(false) {
        initializePLM();
    }

    struct Status {
        bool isRunning;
        bool portOpen;
        std::string rxState;
        std::string txState;
        size_t pendingTxMessages;
        size_t pendingRxMessages;
    };

    // Interface control
    bool start() {
        isRunning = true;
        
        // Open TX serial port
        if (!plmTxHandler.open(txDevicePath, baudRate)) {
            std::cerr << "Failed to open TX port\n";
            return false;
        }
        
        // Start TX handler thread
        plmTxHandler.start();
        
        // Open RX serial port  
        if (!plmRxHandler.open(rxDevicePath, baudRate)) {
            std::cerr << "Failed to open RX port\n";
            plmTxHandler.stop();  // Clean up TX if RX fails
            return false;
        }
        
        // Start RX handler thread
        plmRxHandler.start();
        
        std::cout << "Both handlers started successfully\n";
        return true;
    }

    void stop() {
        if (!isRunning) return;
        
        plmTxHandler.stop();
        plmRxHandler.stop();
        isRunning = false;
    }

    void queueTxMessage(const MessageBuffer& payload) {
        plmTxHandler.queueMessage(payload);  // Fixed: removed type declaration
    }

    bool hasReceivedRxMessage() {
        return plmRxHandler.getQueueSize() > 0;  // Simplified
    }

    MessageBuffer getRxMessage() {  // Fixed: proper function syntax
        if (hasReceivedRxMessage()) {
            return plmRxHandler.getMessage();
        }
        return MessageBuffer(); // Return empty buffer if no message
    }

private:
    PlmTxHandler plmTxHandler;
    PlmRxHandler plmRxHandler;
    int baudRate;
    bool isRunning;
    std::string txDevicePath;
    std::string rxDevicePath;
    std::string configFilePath;
    json jsonConfig;

    void readJsonFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        file >> jsonConfig;  // parse JSON directly
    } 

    void initializePLM() {
        readJsonFile(configFilePath);
        
        // Extract configuration values from JSON
        if (jsonConfig.contains("txDevicePath")) {
            txDevicePath = jsonConfig["txDevicePath"];
        }
        if (jsonConfig.contains("rxDevicePath")) {
            rxDevicePath = jsonConfig["rxDevicePath"];
        }
        if (jsonConfig.contains("baudRate")) {
            baudRate = jsonConfig["baudRate"];
        }
        
        // Add validation as needed
        if (txDevicePath.empty() || rxDevicePath.empty()) {
            throw std::runtime_error("Invalid device paths in configuration");
        }
    }
}; // Fixed: added closing brace and semicolon

#endif // PLMINTERFACE_HPP
