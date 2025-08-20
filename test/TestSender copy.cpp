#include "../include/PlmInterface.hpp"
#include "../include/Message.hpp"
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


class TestSender {
public:
    TestSender() {}
    
    ~TestSender() {}
    

    PlmInterface plmInterface;

     // Define the possible states
    enum class State {
        WAIT,
        SEND_DATA,
        WAIT_ACK
    };

    State getCurrentState() const { return currentState; };

    void setCurrentState(State newState) {
        currentState = newState;
    };



   MessageBuffer createCurrentTxMessage(const std::vector<uint8_t>& payload) {
        size_t payloadSize = payload.size();
        std::vector<uint8_t> buffer(19 + payloadSize); // Header + payload + checksum

        // Header
        buffer[0] = CMD_START_BYTE;
        
        // Length (excludes start byte, length field, and checksum)
        uint16_t length_field = 15 + payloadSize; // 15 bytes for header fields excluding start byte and length
        buffer[1] = length_field & 0xFF;
        buffer[2] = (length_field >> 8) & 0xFF;
        
        // Type & Opcode
        buffer[3] = MSG_TYPE_TRANSMIT_REQ;
        buffer[4] = OPCODE_TRANSMIT_REQ;
        
        buffer[5] = 0x01;   // Data service type, intranetwork, unicast
        buffer[6] = 0x00;   // Priority
        buffer[7] = 0x00;   // ack service
        buffer[8] = 0x01;   // Hops
        buffer[9] = 0x01;   // gain

        buffer[10] = 0x01;  // tag (LSB)
        buffer[11] = 0x00;  // tag (MSB)
                
        buffer[12] = 0x00; // Encrypt
        buffer[13] = 0x00; // Target port

        buffer[14] = 0x01; // Target ID LSB
        buffer[15] = 0x00; // Target ID MS

        // payload
        buffer[16] = payload;

        // Checksum
        buffer[16 + payloadSize] = Message::calculateChecksum(buffer);

        return buffer;
    }


    std::string getStateName() const {
        switch (currentState) {
            case State::WAIT: return "WAIT";
            case State::SEND_DATA: return "SEND_DATA";
            case State::WAIT_ACK: return "WAIT_ACK";
            default: return "UNKNOWN";
        }
    };



    bool sendMessage(const MessageBuffer& payload) {
        plmInterface.queueTxMessage(payload);
    }
    
   
    void stop() {
        running = false;
    }

    void run() {

        running = true;
        std::cout << "Sender running, waiting for message, " << getStateName() << "...\n";
        uint8_t counter = 0;
        setCurrentState(State::WAIT);

        
        while (running) {
            switch(getCurrentState()){
                case State::WAIT: {
                    std::cout << "State: "<< getStateName() <<"\n";
                    currentTxMessage = getCurrentMessage(counter++);
                    setCurrentState(State::SEND_TX);
                    sleep(1);
                }
                break;

                case State::SEND_DATA: {
                    std::cout << "State: "<< getStateName() <<"\n";
                    
                    // Send the message
                    sendMessage(currentTxMessage); 

                    Message::printHexDump("Sending transmit request", currentTxMessage);
                    std::cout << "Placed " << currentTxMessage.size() << " bytes into queue\n";
                    setCurrentState(State::WAIT);
                }
                break;


                case State::WAIT_ACK: {
                    std::cout << "State: "<< getStateName() <<"\n";

                    if (plmInterface.getRxQueueSize() > 0){
                        currentRxMessage = plmInterface.getQueuedRxMessage();
                        std::cout << "Received ACK" << std::endl;
                        Message::printHexDump("Sending transmit request", currentRxMessage);
                        setCurrentState(State::WAIT);
                        std::cout << "State: "<< getStateName() <<"\n";
                    }
                }
                break;
            }
            
            // Small delay to prevent CPU hogging
            usleep(500); // 10ms

        }
    }
    
private:

    std::atomic<bool> running;
    State currentState;
    MessageBuffer currentTxMessage;
    MessageBuffer currentRxMessage;


    
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
TestSender* g_sender = nullptr;

// Signal handler for graceful shutdown
void handleSignal(int sig) {
    std::cout << "\nReceived signal " << sig << ", shutting down...\n";
    if (g_sender) {
        g_sender->stop();
    }
}




int main(int argc, char *argv[]) {

    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    
    TestSender sender;
    g_sender = &sender;

    std::cout << "State: "<< sender.getStateName() <<"\n";
    
    // Run the sendeer
    sender.run();
    
    g_sender = nullptr;

    return 0;
}


