#include "../include/PlmInterface.hpp"
#include "../include/Message.hpp"  // For Message class methods
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <atomic>

using MessageBuffer = std::vector<uint8_t>;

class TestReceiver {
public:
    TestReceiver() : running(false), currentState(State::WAIT_RX) {}
    
    ~TestReceiver() {}

    PlmInterface plmInterface;

    // Define the possible states
    enum class State {
        WAIT_RX,
        PROC_RX,
        SEND_ACK
    };

    State getCurrentState() const { return currentState; }

    void setCurrentState(State newState) {
        currentState = newState;
    }

    // Create ACK message with "ACK" as ASCII payload
    MessageBuffer createAckMessage() {
        // Create ASCII "ACK" payload
        std::string ackString = "ACK";
        std::vector<uint8_t> payload(ackString.begin(), ackString.end());
        size_t payloadSize = payload.size();
        
        std::vector<uint8_t> buffer(17 + payloadSize); // Header + payload + checksum

        // Header
        buffer[0] = CMD_START_BYTE;
        
        // Length (excludes start byte, length field, and checksum)
        uint16_t length_field = 15 + payloadSize;
        buffer[1] = length_field & 0xFF;
        buffer[2] = (length_field >> 8) & 0xFF;
        
        // Type & Opcode
        buffer[3] = MSG_TYPE_TRANSMIT_REQ;
        buffer[4] = OPCODE_TRANSMIT_REQ;
        
        buffer[5] = 0x01;   // Data service type
        buffer[6] = 0x00;   // Priority
        buffer[7] = 0x00;   // Ack service
        buffer[8] = 0x01;   // Hops
        buffer[9] = 0x01;   // Gain

        buffer[10] = 0x01;  // Tag (LSB)
        buffer[11] = 0x00;  // Tag (MSB)
                
        buffer[12] = 0x00;  // Encrypt
        buffer[13] = 0x00;  // Target port

        buffer[14] = 0x01;  // Target ID LSB
        buffer[15] = 0x00;  // Target ID MSB

        // Payload - copy the "ACK" string
        std::memcpy(&buffer[16], payload.data(), payloadSize);

        // Checksum
        buffer[16 + payloadSize] = Message::calculateChecksum(buffer);

        return buffer;
    }

    std::string getStateName() const {
        switch (currentState) {
            case State::WAIT_RX: return "WAIT_RX";
            case State::PROC_RX: return "PROC_RX";
            case State::SEND_ACK: return "SEND_ACK";
            default: return "UNKNOWN";
        }
    }

    // Send ACK message via PlmInterface
    bool sendAckMessage(const std::vector<uint8_t>& buffer) {
        plmInterface.queueTxMessage(buffer);
        return true;
    }

    // Process received message and extract payload
    void processReceivedMessage(const MessageBuffer& message) {
        if (message.empty()) {
            std::cout << "Received empty message\n";
            return;
        }

        Message::printHexDump("Received message", message);
        
        // Extract payload if it's a valid message
        std::vector<uint8_t> payload;
        int result = extractPayload(message, payload);
        
        if (result > 0) {
            // Try to interpret as text
            std::string text(payload.begin(), payload.end());
            std::cout << "Received payload as text: '" << text << "'\n";
            std::cout << "Payload size: " << result << " bytes\n";
        } else if (result == 0) {
            std::cout << "Message received with no payload\n";
        } else {
            std::cout << "Invalid message format, error code: " << result << "\n";
        }
    }
   
    void stop() {
        running = false;
        plmInterface.stop();  // Also stop the PLM interface
    }

    void run() {
        // Start the PLM interface first
        if (!plmInterface.start()) {
            std::cerr << "Failed to start PLM interface\n";
            return;
        }

        running = true;
        std::cout << "Receiver running, waiting for messages, " << getStateName() << "...\n";
        setCurrentState(State::WAIT_RX);

        while (running) {
            switch(getCurrentState()) {
                case State::WAIT_RX: {
                    std::cout << "State: " << getStateName() << " - Monitoring for incoming messages...\n";
                    
                    // Check if there are received messages
                    if (plmInterface.hasReceivedRxMessage()) {
                        std::cout << "Message detected! Changing to PROC_RX state\n";
                        setCurrentState(State::PROC_RX);
                    }
                    
                    // Small delay to prevent CPU hogging while waiting
                    usleep(100000); // 100ms
                }
                break;

                case State::PROC_RX: {
                    std::cout << "State: " << getStateName() << " - Processing received message\n";
                    
                    // Get the received message
                    currentRxMessage = plmInterface.getRxMessage();
                    
                    // Process the received message
                    processReceivedMessage(currentRxMessage);
                    
                    // Create ACK response
                    currentAckMessage = createAckMessage();
                    
                    std::cout << "ACK message created, changing to SEND_ACK state\n";
                    setCurrentState(State::SEND_ACK);
                }
                break;

                case State::SEND_ACK: {
                    std::cout << "State: " << getStateName() << " - Sending ACK message\n";
                    
                    // Send the ACK message
                    sendAckMessage(currentAckMessage);
                    
                    Message::printHexDump("Sending ACK message", currentAckMessage);
                    std::cout << "Placed " << currentAckMessage.size() << " bytes ACK into tx queue\n";
                    
                    std::cout << "ACK sent, returning to WAIT_RX state\n";
                    setCurrentState(State::WAIT_RX);
                }
                break;
            }
            
            // Small delay to prevent CPU hogging
            usleep(1000); // 1ms
        }
        
        std::cout << "Receiver stopped\n";
    }
    
private:
    std::atomic<bool> running;
    State currentState;
    MessageBuffer currentRxMessage;
    MessageBuffer currentAckMessage;

    // Extract payload from received message
    int extractPayload(const std::vector<uint8_t>& packet, std::vector<uint8_t>& payload) {
        if (packet.size() < 27) {  // Minimum packet size with empty payload
            return -1;
        }
        
        // Verify it's an intranetwork receive message
        if (packet[0] != CMD_START_BYTE || 
            packet[3] != MSG_TYPE_INTRANETWORK_RECEIVE || 
            packet[4] != OPCODE_INTRANETWORK_RECEIVE) {
            return -2;
        }
        
        // Extract length field
        uint16_t msg_len = packet[1] | (packet[2] << 8);
        
        // Verify message length
        if (packet.size() < msg_len + 4) {  // +4 for start byte, length (2 bytes), and checksum
            return -3;
        }
        
        // Verify checksum
        uint8_t expected_cs = packet[packet.size() - 1];
        uint8_t calculated_cs = Message::calculateChecksum(packet);
        if (expected_cs != calculated_cs) {
            return -4;
        }
        
        // Calculate payload length and extract payload
        int payload_len = msg_len - 23;  // 23 bytes for header fields excluding start byte and length
        if (payload_len <= 0) {
            payload.clear();
            return 0;  // No payload
        }
        
        // Copy payload (starts at offset 26)
        payload.assign(packet.begin() + 26, packet.begin() + 26 + payload_len);
        
        return payload_len;
    }
};

// Global receiver instance for signal handler
TestReceiver* g_receiver = nullptr;

// Signal handler for graceful shutdown
void handleSignal(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down...\n";
    if (g_receiver) {
        g_receiver->stop();
    }
}

int main(int argc, char *argv[]) {
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    TestReceiver receiver;
    g_receiver = &receiver;

    std::cout << "Initial State: " << receiver.getStateName() << "\n";
    
    // Run the receiver
    receiver.run();
    
    g_receiver = nullptr;
    return 0;
}
