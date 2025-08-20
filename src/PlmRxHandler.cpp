#include "../include/PlmRxHandler.hpp"
//ok
void PlmRxHandler::run() {
    if (fd < 0) {
        std::cerr << "Serial port not open\n";
        return;
    }
    
    running = true;
    std::cout << "Sender running, waiting for message, " << getStateName() << "...\n";
    
    while (running) {
        switch(getCurrentState()){
            case State::WAIT: {

                // read new messages
                MessageBuffer buffer = receiveMessage();
                if (!buffer.empty()){
                    currentRxMessage = std::move(buffer);
                    setCurrentState(State::PROC_MSG);
                }
            }
            break;

            case State::PROC_MSG: {
                std::cout << "State: "<< getStateName() <<"\n";

                // decode packet;
                result = decodePacket();
                
                if (result > 0) {
                    // Try to interpret as text
                    std::string text(payload.begin(), payload.end());
                    std::cout << "Received text: " << text << std::endl; 
                } else {
                    std::cout << "Invalid extracted payload result: " << result << std::endl;
                }
        
                setCurrentState(State::WAIT);
                std::cout << "State: "<< getStateName() <<"\n";
            }
            break;
        }
        
        // Small delay to prevent CPU hogging
        usleep(100); // 10ms

    }
}
//ok
void PlmRxHandler::processRxIndMessage(){

    int retVal = 0;
    
    // Extract length field
    uint16_t msg_len = currentRxMessage[1] | (currentRxMessage[2] << 8);

    // calc checksum
    uint8_t expected_cs = currentRxMessage[currentRxMessage.size() - 1];
    uint8_t calculated_cs = Message::calculateChecksum(currentRxMessage);

    // Calculate currentPayload length and extract currentPayload
    int currentPayload_len = msg_len - 23;  // 23 bytes for header fields excluding start byte and length
    
    // verify message length
    if (currentRxMessage.size() < msg_len + 4) {  // +4 for start byte, length (2 bytes), and checksum
        retVal = -3;
    }

    // Verify checksum
    else if (expected_cs != calculated_cs) {
        retVal = -4;
    }
    
    // verify missing payload
    else if (currentPayload_len <= 0) {
        currentPayload.clear();
        retVal = 0;  // No currentPayload
    }

    // process payload
    else {
            // data service type
        dataServiceType = currentRxMessage[6];

        // signal quality
        signalQuality = currentRxMessage[8];

        // priority
        priority =  currentRxMessage[10];

        // cw
        cw = currentRxMessage[11];

        //tx result
        txResult= currentRxMessage[13];

        // networkID
        networkID = static_cast<uint16_t>(currentRxMessage[14])
                | (static_cast<uint16_t>(currentRxMessage[15]) << 8);

        // sourceID
        sourceID = static_cast<uint16_t>(currentRxMessage[16])
                | (static_cast<uint16_t>(currentRxMessage[17]) << 8);

        // targetID
        targetID = static_cast<uint16_t>(currentRxMessage[18])
                | (static_cast<uint16_t>(currentRxMessage[19]) << 8);

        //originID address type
        originIDAddrType = currentRxMessage[20];

        // originID
        originID = static_cast<uint16_t>(currentRxMessage[21])
                | (static_cast<uint16_t>(currentRxMessage[22]) << 8);

        // finalID
        finalID = static_cast<uint16_t>(currentRxMessage[23])
                | (static_cast<uint16_t>(currentRxMessage[24]) << 8);

        // source port bits 0::3
        sourcePort = currentRxMessage[25] & 0x0F;

        // target port bits 0::3
        targetPort = (currentRxMessage[25] >> 4) & 0x0F;

        // Copy currentPayload (starts at offset 26)
        currentPayload.assign(currentRxMessage.begin() + 26, currentRxMessage.begin() + 26 + currentPayload_len);
        
        retVal = payload_len;

    }

    return retVal;
}
//ok
void PlmRxHandler::setTxHandler(PlmTxHandler* txHandler) {
    std::lock_guard<std::mutex> lock(m_txHandlerMutex);
    m_txHandler = txHandler;
}
//ok
PlmRxHandler::State PlmRxHandler::getCurrentState() const { return currentState; };
//ok
void PlmRxHandler::setCurrentState(State newState) {
        currentState = newState;
    };
//ok
std::string PlmRxHandler::getStateName() const {
    switch (currentState) {
        case State::WAIT: return "WAIT";
        case State::PROC_MSG: return "PROC_MSG";
        default: return "UNKNOWN";
    }
};
//ok
void PlmRxHandler::setCurrentMessage(std::string){
    message = _message;
}
//ok
bool PlmRxHandler::open(const std::string& devicePath, speed_t) {
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

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

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
void PlmRxHandler::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}
//ok
void PlmRxHandler::stop() {
    running = false;
}
//ok
std::vector<uint8_t> PlmRxHandler::receiveSerialMessage(int timeout_ms = 100) {
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
//ok
int PlmRxHandler::decodePacket() {
    int retVal = 0

    // check if it's a resp1 message
    if (currentRxMessage[0] == CMD_START_BYTE && currentRxMessage[3] == MSG_TYPE_TRANSMIT_RESP1){
        retVal = 1;

        // send message to PlmTxHandler
        std::lock_guard<std::mutex> lock(m_txHandlerMutex);
        if (m_txHandler) {
            PlmMessage message;
            message.type = MSG_RESP1;
            message.data = currentRxMessage;
            m_txHandler->handleItcMessage(&message);
        }
    
        // check if it's a resp2 message
    } else if (currentRxMessage[0] == CMD_START_BYTE && currentRxMessage[3] == MSG_TYPE_TRANSMIT_RESP2){
        retVal = 1;

        // send message to PlmTxHandler
        std::lock_guard<std::mutex> lock(m_txHandlerMutex);
        if (m_txHandler) {
            PlmMessage message;
            message.type = MSG_RESP2;
            message.data = currentRxMessage;
            m_txHandler->handleItcMessage(&message);
        }
    }
    
    // check if it's an intranetwork receive message
    else if (currentRxMessage[0] != CMD_START_BYTE || 
        currentRxMessage[3] == MSG_TYPE_INTRANETWORK_RECEIVE || 
        currentRxMessage[4] == OPCODE_INTRANETWORK_RECEIVE) {
            retVal = processRxIndMessage();
            // if message is ok, push to stack
            if (retVal > 0){
                pushMessage(currentPayload);
            }   
    }
    else {
        retval = -1;
    }

    return retVal;
}
//ok
int PlmRxHandler::processRxIndMessage(){
    int retVal = 0;

    // data service type
    dataServiceType = currentRxMessage[6];

    // signal quality
    signalQuality = currentRxMessage[8];

    // priority
    priority =  currentRxMessage[10];

    // cw
    cw = currentRxMessage[11];

    //tx result
    txResult= currentRxMessage[13];

    // networkID
    networkID = static_cast<uint16_t>(currentRxMessage[14])
               | (static_cast<uint16_t>(currentRxMessage[15]) << 8);

    // sourceID
    sourceID = static_cast<uint16_t>(currentRxMessage[16])
               | (static_cast<uint16_t>(currentRxMessage[17]) << 8);

    // targetID
    targetID = static_cast<uint16_t>(currentRxMessage[18])
               | (static_cast<uint16_t>(currentRxMessage[19]) << 8);

    //originID address type
    originIDAddrType = currentRxMessage[20];

    // originID
    originID = static_cast<uint16_t>(currentRxMessage[21])
               | (static_cast<uint16_t>(currentRxMessage[22]) << 8);

    // finalID
    finalID = static_cast<uint16_t>(currentRxMessage[23])
               | (static_cast<uint16_t>(currentRxMessage[24]) << 8);

    // source port bits 0::3
    sourcePort = currentRxMessage[25] & 0x0F;

    // target port bits 0::3
    targetPort = (currentRxMessage[25] >> 4) & 0x0F;

    // Calculate currentPayload length and extract currentPayload
    int currentPayload_len = msg_len - 23;  // 23 bytes for header fields excluding start byte and length

    // Verify checksum
    uint8_t expected_cs = currentRxMessage[currentRxMessage.size() - 1];
    uint8_t calculated_cs = Message::calculateChecksum(currentRxMessage);
    if (expected_cs != calculated_cs) {
        retVal = -4;
    }
    // Verify payload length
    else if (currentPayload_len <= 0) {
        currentPayload.clear();
        retVal = 0;  // No currentPayload
    } else 
    // all is well extract payload
    {
    // Copy currentPayload (starts at offset 26)
        currentPayload.assign(currentRxMessage.begin() + 26, currentRxMessage.begin() + 26 + currentPayload_len);
        retVal = currentPayload_len;  
    }
    
    return retVal;

}
//ok
// used internally by PROC_MSG to push message for external access
void PlmRxHandler::pushMessage(MessageBuffer){
    rxQueue.push(MessageBuffer);
}
//ok
// used by PlmInterface for application to fetch payloads
MessageBuffer PlmRxHandler::getMessage(){
   return( rxQueue.pop());
}
//ok
uint8_t PlmRxHandler::getDataServiceType(){
    return (dataServiceType);
}
//ok
uint8_t PlmRxHandler::getSignalQuality(){
    return (signalQuality);
}
//ok
uint8_t PlmRxHandler::getTxServiceType(){
    return(txServiceType);
}
//ok
uint8_t PlmRxHandler::getPriority(){
    return(priority);
}
//ok
uint8_t PlmRxHandler::getCW(){
    return(cw);
}
//ok
uint8_t PlmRxHandler::getTxResult(){
    return(txResult);
}
//ok
uint16_t PlmRxHandler::getNetworkID(){
    return(networkID);
}
//ok
uint16_t PlmRxHandler::getSourceID(){
    return(sourceID);
}
//ok
uint16_t PlmRxHandler::getTargetID(){
    return(targetID);
}
//ok
uint8_t PlmRxHandler::getOriginIDAddrType(){
    return(getOriginIDAddrType);
}
//ok
uint16_t PlmRxHandler::getOriginID(){
    return(originID);
}
//ok
uint16_t PlmRxHandler::getFinalID(){
    return(finalID);
}
//ok
uint8_t PlmRxHandler::getSourcePort(){
    return(sourcePort);
}
//ok
uint8_t PlmRxHandler::getTargetPort(){
    return(targetPort);
}
//ok
int PlmRxHandler::getQueueSize(){
    return (rxQueue.size());
}
//ok
void PlmRxHandler::setTargetID(uint16_t _targetID){
    targetID = _targetID;
}
//ok
void PlmRxHandler::setTargetPort(uint8_t _targetPort){
    targetPort = _targetPort;
}




    
/*
int PlmRxHandler::decodePacket() {

    // check if it's a resp1 message
    if (currentRxMessage[0] == CMD_START_BYTE && currentRxMessage[3] == MSG_TYPE_TRANSMIT_RESP1){
        isResp1Msg = true;
        isResp2Msg = false;
        currentRxMessage = message;
    
        // check if it's a resp2 message
    } else if (currentRxMessage[0] == CMD_START_BYTE && currentRxMessage[3] == MSG_TYPE_TRANSMIT_RESP2){
        isResp1Msg = false;
        isResp2Msg = true;
        currentRxMessage = message;
    }
    
    // check if it's an intranetwork receive message
    else if (currentRxMessage[0] != CMD_START_BYTE || 
        currentRxMessage[3] == MSG_TYPE_INTRANETWORK_RECEIVE || 
        currentRxMessage[4] == OPCODE_INTRANETWORK_RECEIVE) {
            processRxIndMessage()
        
    }
    
    // Extract length field
    uint16_t msg_len = currentRxMessage[1] | (currentRxMessage[2] << 8);
    
    // Verify message length
    if (currentRxMessage.size() < msg_len + 4) {  // +4 for start byte, length (2 bytes), and checksum
        return -3;
    }

    // data service type
    dataServiceType = currentRxMessage[6];

    // signal quality
    signalQuality = currentRxMessage[8];

    // priority
    priority =  currentRxMessage[10];

    // cw
    cw = currentRxMessage[11];

    //tx result
    txResult= currentRxMessage[13];

    // networkID
    networkID = static_cast<uint16_t>(currentRxMessage[14])
               | (static_cast<uint16_t>(currentRxMessage[15]) << 8);

    // sourceID
    sourceID = static_cast<uint16_t>(currentRxMessage[16])
               | (static_cast<uint16_t>(currentRxMessage[17]) << 8);

    // targetID
    targetID = static_cast<uint16_t>(currentRxMessage[18])
               | (static_cast<uint16_t>(currentRxMessage[19]) << 8);

    //originID address type
    originIDAddrType = currentRxMessage[20];

    // originID
    originID = static_cast<uint16_t>(currentRxMessage[21])
               | (static_cast<uint16_t>(currentRxMessage[22]) << 8);

    // finalID
    finalID = static_cast<uint16_t>(currentRxMessage[23])
               | (static_cast<uint16_t>(currentRxMessage[24]) << 8);

    // source port bits 0::3
    sourcePort = currentRxMessage[25] & 0x0F;

    // target port bits 0::3
    targetPort = (currentRxMessage[25] >> 4) & 0x0F;

    // Verify checksum
    uint8_t expected_cs = currentRxMessage[currentRxMessage.size() - 1];
    uint8_t calculated_cs = Message::calculateChecksum(currentRxMessage);
    if (expected_cs != calculated_cs) {
        return -4;
    }
    
    // Calculate currentPayload length and extract currentPayload
    int currentPayload_len = msg_len - 23;  // 23 bytes for header fields excluding start byte and length
    if (currentPayload_len <= 0) {
        currentPayload.clear();
        return 0;  // No currentPayload
    }
    
    // Copy currentPayload (starts at offset 26)
    currentPayload.assign(currentRxMessage.begin() + 26, currentRxMessage.begin() + 26 + currentPayload_len);
    
    return payload_len;
}


*/

