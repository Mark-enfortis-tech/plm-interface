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

class PlmTxHandler {
    public:
        PlmTxHandler(){}
        
        ~PlmTxHandler(){}

        // Define the possible states
        enum class State {
            WAIT,
            PROC_MSG,
            SEND_TX,
            WAIT_RESP1,
            WAIT_RESP2,
            WAIT_ACK
        };

        // State management
        //ok
        State getCurrentState() const;
        //ok
        void setCurrentState(State newState);
        //ok
        std::string getStateName() const;
        //ok
        // serial port management
        bool open(const std::string& devicePath, int baudRate);
        //ok
        void close();
        //ok
        // session and target 
        void stop();
        //ok
        void start();

        // modem init
        //ok
        void setTargetPort(uint8_t _targetPort);
        //ok
        uint8_t getTargetPort();
        //ok
        void setTargetID(uint16_t _targetID);
        //ok
        uint16_t getTargetID();
        //ok
        void setTag(uint16_t);

        // queue 
        //ok
        void queueMessage(const MessageBuffer& payload);
        //ok
        MessageBuffer getQueuedMessage();
        //ok
        int getQueueSize()
        //ok
        // inter-thread messaging
        void handleItcMessage(const PlmMessage& message);
        
        
    private:
        int fd;
        std::atomic<bool> running;
        State currentState;
        bool isResp1Msg = false;
        bool isResp2Msg = false;
        //ok
        void run();

        // Message queues and synchronization
        std::queue<MessageBuffer> txQueue;
        std::queue<MessageBuffer> rxQueue;
        std::mutex txQueueMutex;
        std::mutex rxQueueMutex;
        
        
        // Thread management
        std::thread txThread;
        std::thread rxThread;
        std::mutex responseMutex;

        // Current transmission context
        MessageBuffer currentPayload;
        MessageBuffer currentTxMessage;
        MessageBuffer currentRxMessage;
        uint16_t currentTag;
        int retryCount;
        static constexpr int MAX_RETRIES = 3;
        static constexpr int RESPONSE_TIMEOUT_MS = 5000;
        uint8_t targetPort;
        uint16_t targetID;
        
        // Message creation and sending
        //ok
        bool sendMessage(const std::vector<uint8_t>& buffer);
        //ok
        MessageBuffer createTxMessage(uint16_t tag);

        // Transmission management

        // Utility methods
        //ok
        uint16_t generateTag();

        // inter-thread messaging
        //TODO: get code from ninja.
        std::function<void(const PlmMessage&)> m_messageCallback;
        std::mutex m_callbackMutex;
    
}