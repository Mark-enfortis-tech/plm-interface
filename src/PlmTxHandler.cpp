#include "./include/PlmTxHandler.hpp"
//ok
PlmTxHandler::State PlmTxHandler::getCurrentState() const { 
    return currentState; 
}
//ok
void PlmTxHandler::setCurrentState(State newState) {
        currentState = newState;
    };
//ok
std::string PlmTxHandler::getStateName() const {
    switch (currentState) {
        case State::WAIT: return "WAIT";
        case State::PROC_MSG: return "PROC_MSG";
        case State::SEND_TX: return "SEND_TX";
        case State::WAIT_RESP1: return "WAIT_RESP1";
        case State::WAIT_RESP2: return "WAIT_RESP2";
        default: return "UNKNOWN";
    }
};
//ok
void PlmTxHandler::run() {
    if (fd < 0) {
        std::cerr << "Serial port not open\n";
        return;
    }
    
    std::cout << "Sender running, waiting for message, " << getStateName() << "...\n";
    
    while (running) {
        switch(getCurrentState()){
            case State::WAIT: {

                // check queue
                //  if not empty, change state to PROC_MSG
                if (getQueueSize() > 0 ){
                    setCurrentState(State::PROC_MSG);
                }
                // Small delay to prevent CPU hogging
                sleep(1); // 1s
            }
            break;

            case State::PROC_MSG: {
                std::cout << "State: "<< getStateName() <<"\n";

                // pop queue
                currentPayload = std::move(getMessage());

                // create message
                uint16_t _newTag = generateTag();
                currentTxMessage = std::move(createTxMessage(_newTag));
    
                setCurrentState(State::SEND_TX);
            }
            break;

            case State::SEND_TX: {
                std::cout << "State: "<< getStateName() <<"\n";

                // Send the message
                Message::printHexDump("Sending transmit request", currentTxMessage);
                if (!sendMessage(currentTxMessage)) {
                    std::cerr << "Failed to send message, changing to State:WAIT\n";
                    setCurrentState(State::WAIT);
                }
                std::cout << "Sent " << currentTxMessage.size() << " bytes\n";

                setCurrentState(State::WAIT_RESP1);
            }
            break;

            // NOTE: resp1 and resp2 messages are received by PlmRxHandler which will invoke a direct callback
            // to the PlmTxHandler::handleRxRespMessage(const PlmMessage& message). This method will fetch
            // the rxMessage from the PlmMessage param and set a flag either isResp1 or isResp2 depending on 
            // the response type.
            case State::WAIT_RESP1: {
                std::cout << "State: "<< getStateName() <<"\n";
                if (isResp1Msg.load()){
                    std::lock_guard<std::mutex> lock(responseMutex);
                    Message::printHexDump("Received response1", currentRxMessage);
                    setCurrentState(State::WAIT_RESP2);
                    isResp1Msg = false;
                }
            }
            break;

            case State::WAIT_RESP2: {
                std::cout << "State: "<< getStateName() <<"\n";
                if (isResp2Msg.load()){
                    std::lock_guard<std::mutex> lock(responseMutex);
                    Message::printHexDump("Received response2", currentRxMessage);
                    setCurrentState(State::WAIT);
                    isResp2Msg = false;
                }
            }
            break;

        }
        
        // Small delay to prevent CPU hogging
        usleep(100); // 10ms

    }
}

//ok
// Descr: This method is a called by a callback signal from the PlmRxHandler whenever a resp1 or resp2 message is received.
// This method will fetch the rxMessage from the PlmMessage param and set a flag either isResp1 or isResp2 
// depending on the response type.
void PlmTxHandler::handleRxRespMessage(const PlmMessage& message) {
    std::lock_guard<std::mutex> lock(responseMutex);
    
    if (message.type == MSG_RESP1) {
        currentRxMessage = message.data;
        isResp2Msg = false;
        isResp1Msg = true;  // Set this last
    } else if (message.type == MSG_RESP2) {
        currentRxMessage = message.data;
        isResp1Msg = false;
        isResp2Msg = true;  // Set this last
    }
}
//ok
bool PlmTxHandler::open(const std::string& devicePath, int baudRate) {
    fd = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return false;
    }

    struct termios options;
    std::memset(&options, 0, sizeof(options));

    if (tcgetattr(fd, &options) < 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    return true;
}
//ok
void PlmTxHandler::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}
//ok
bool PlmTxHandler::sendMessage(const std::vector<uint8_t>& buffer) {
    if (fd < 0) {
        return false;
    }
    
    int n = ::write(fd, buffer.data(), buffer.size());
    return (n == static_cast<int>(buffer.size()));
}
//ok
void PlmTxHandler::start() {
    if (fd < 0) {
        std::cerr << "Serial port not open. Call open() first.\n";
        return;
    }
    
    if (running) {
        std::cout << "PlmTxHandler already running\n";
        return;
    }
    
    running = true;
    txThread = std::thread(&PlmTxHandler::run, this);
    std::cout << "PlmTxHandler thread started\n";
}
//ok
void PlmTxHandler::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    
    if (txThread.joinable()) {
        txThread.join();
    }
    
    std::cout << "PlmTxHandler thread stopped\n";
}
//ok
MessageBuffer PlmTxHandler::getQueuedMessage(){
    std::lock_guard<std::mutex> lock(txQueueMutex);  // MUST add this
    if (txQueue.empty()) {
        return MessageBuffer();
    }
    MessageBuffer msg = txQueue.front();
    txQueue.pop();
    return msg;
};
//ok
void PlmTxHandler::queueMessage(const MessageBuffer& payload){
    std::lock_guard<std::mutex> lock(txQueueMutex);  // MUST add this
    txQueue.push(MessageBuffer);
};
//ok
int PlmTxHandler::getQueueSize(){
    std::lock_guard<std::mutex> lock(txQueueMutex);  // MUST add this
    return txQueue.size();
};
//ok
MessageBuffer PlmTxHandler::createTxMessage(uint16_t tag) {
    size_t payloadSize = std::min(currentPayload.size(), static_cast<size_t>(MAX_PAYLOAD_SIZE));
    MessageBuffer buffer(17 + payloadSize); // Header + payload + checksum

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

    // Tag (LSB/MSB)
    buffer[10] = tag & 0xFF;
    buffer[11] = (tag >> 8) & 0xFF;

    buffer[12] = 0x00;  // Encrypt
    
    buffer[13] = getTargetPort();  // Target port

    uint16_t _targetID = getTargetID();
    buffer[14] = _targetID & 0xFF;           // Target ID LSB
    buffer[15] = (_targetID >> 8) & 0xFF;    // Target ID MSB

    // Payload
    if (!currentPayload.empty()) {
        std::memcpy(&buffer[16], currentPayload.data(), payloadSize);
    }

    // Checksum
    buffer[16 + payloadSize] = Message::calculateChecksum(buffer);

    return buffer;
}
//ok
void PlmTxHandler::setTargetPort(uint8_t _targetPort){
    targetPort = _targetPort;
}
//ok
uint8_t PlmTxHandler::getTargetPort(){
    return (targetPort);
}
//ok
void PlmTxHandler::setTargetID(uint16_t _targetID){
    targetID = _targetID;
}
//ok
uint16_t PlmTxHandler::getTargetID(){
    return (targetID);
}
//ok
uint16_t PlmTxHandler::generateTag() {
    return currentTag++;
}

// used by PlmInterface 
//ok
void PlmTxHandler::setTag(uint16_t _tag) {
    currentTag = _tag;
}
    



/*
int PlmTxHandler::extractPayload(const std::vector<uint8_t>& packet, std::vector<uint8_t>& payload) {
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



MessageBuffer PlmTxHandler::receiveSerialMessage(int timeout_ms = 100) {
    if (fd < 0) {
        return {};
    }
    
    // Set non-blocking read with timeout
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    int select_result = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    
    if (select_result <= 0) {
        return {}; // Timeout or error
    }
    
    // Data is available, read it
    std::vector<uint8_t> buffer(1024);
    int n = ::read(fd, buffer.data(), buffer.size());
    
    if (n <= 0) {
        return {};
    }
    
    buffer.resize(n);
    return buffer;
}



            // TODO: If necessary can place a WAIT_ACK state here and pause tx operations
            // until ACK is received from the transmitting end. For now just continue.

            // case State::WAIT_ACK: {
            //     std::cout << "State: "<< getStateName() <<"\n";
            //     std::vector<uint8_t> buffer(1024);

            //     std::vector<uint8_t> response = receiveResponse();
            //     if (!response.empty()) {
            //         Message::printHexDump("Received ACK", response);

            //         // Extract payload if it's an intranetwork receive message
            //         if (buffer.size() >= 5 && 
            //             buffer[0] == CMD_START_BYTE && 
            //             buffer[3] == MSG_TYPE_INTRANETWORK_RECEIVE && 
            //             buffer[4] == OPCODE_INTRANETWORK_RECEIVE) {
                        
            //             std::vector<uint8_t> payload;
            //             int result = extractPayload(buffer, payload);
                        
            //             if (result > 0) {
            //                 // Try to interpret as text
            //                 std::string text(payload.begin(), payload.end());
            //                 std::cout << "Received text: " << text << std::endl;
            //             }
            //         }

            //         setCurrentState(State::WAIT);
            //         std::cout << "\n\n\nState: "<< getStateName() <<"\n";
            //     } else {
            //         std::cout << "No response received or error, changing to State:WAIT\n";
            //         setCurrentState(State::WAIT);
            //         break;
            //     }
            // }
            // break;



std::vector<uint8_t> PlmTxHandler::createTransmitRequest(const std::string& payload, uint8_t tag) {
    return createTransmitRequest(
        std::vector<uint8_t>(payload.begin(), payload.end()), 
        tag
    );
}

std::vector<uint8_t> PlmTxHandler::createTransmitRequest(const std::vector<uint8_t>& payload, uint8_t tag) {
    size_t payloadSize = std::min(payload.size(), static_cast<size_t>(MAX_PAYLOAD_SIZE));
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
    
    // Payload
    if (!payload.empty()) {
        std::memcpy(&buffer[16], payload.data(), payloadSize);
    }
    
    // Checksum
    buffer[16 + payloadSize] = Message::calculateChecksum(buffer);
    
    return buffer;
}


*/