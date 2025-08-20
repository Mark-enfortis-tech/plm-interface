// PlmInterface.h
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <vector>

#define MSG_RESP1 0X01
#define MSG_RESP2 0X02

struct PlmMessage {
    int type;
    std::vector<uint8_t> data;
    // Add other fields as needed
};




