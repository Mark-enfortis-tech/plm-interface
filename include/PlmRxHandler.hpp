#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <atomic>
#include <termios.h>
#include <errno.h>
#include <functional>

#include "PlmMessage.hpp"

using MessageBuffer = std::vector<uint8_t>;

class PlmRxHandler {
    public:
        PlmRxHandler(){}
        
        ~PlmRxHandler(){}

        // Define the possible states
        enum class State {
            WAIT,
            PROC_MSG,
            SEND_ACK,
            WAIT_RESP1,
            WAIT_RESP2
        };

        // State management
        //ok
        State getCurrentState() const;
        //ok
        void setCurrentState(State newState);
        //ok
        std::string getStateName();
        //ok
        void setCurrentMessage(std::string);

        // serial port management
        //ok
        bool open(const std::string& devicePath, speed_t);
        //ok
        void close();
        //ok
        std::vector<uint8_t> PlmRxHandler::receiveSerialMessage(int timeout_ms = 1000);

        // session and target 
        //ok
        void stop();
        //ok
        void run();

        // this is done by PlmTxHandler
        //ok
        void setTargetPort(uint8_t _targetPort);
        //ok
        void setTargetID(uint16_t _targetID)


        //ok
        MessageBuffer getMessage();    // used by PlmInterface
        //ok
        int getQueueSize();            // used by PlmInterface
        
        // status information
        //ok
        uint8_t getDataServiceType();
        //ok
        uint8_t getSignalQuality();
        //ok
        uint8_t getTxServiceType();
        //ok
        uint8_t getPriority();
        //ok
        uint8_t getCW();
        //ok
        uint8_t getTxResult();

        
        // network stats
        //ok
        uint16_t getNetworkID();
        //ok
        uint16_t getSourceID();
        //ok
        uint16_t getTargetID();
        //ok
        uint8_t getOriginIDAddrType();
        //ok
        uint16_t getOriginID();
        //ok
        uint16_t getFinalID();
        //ok
        uint8_t getSourcePort();
        //ok
        uint8_t getTargetPort();

        // inter-thread messaging
        // ok
        void setTxHandler(PlmTxHandler* txHandler);
        
    private:
        int outputFd;
        std::atomic<bool> running;
        State currentState;

        // Message queues and synchronization
        std::queue<MessageBuffer>& rxQueue;
        std::mutex& rxQueueMutex;

        // Thread management
        std::thread rxThread;

        // Current transmission context
        MessageBuffer currentPayload;
        MessageBuffer currentRxMessage;

        // status information
        uint8_t dataServiceType;
        uint8_t signalQuality;
        uint8_t txServiceType;
        uint8_t priority;
        uint8_t cw;
        uint8_t txResult;
        
        // network stats
        uint16_t networkID;
        uint16_t sourceID;
        uint16_t targetID;
        uint8_t originIDAddrType;
        uint16_t originID;
        uint16_t finalID;
        uint8_t sourcePort;
        uint8_t targetPort;

        // inter-thread messaging
        PlmTxHandler* m_txHandler;
        std::mutex m_txHandlerMutex;


        // Message reception and processing
        //ok
        int decodePacket();
        //ok
        void pushMessage(MessageBuffer);
        //ok
        int processRxIndMessage();

        // Transmission management

        // Utility methods
   
    
}