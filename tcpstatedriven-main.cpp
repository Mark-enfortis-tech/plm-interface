#include "include/json.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <chrono>


// For convenience
using json = nlohmann::json;

enum ServerState { CONNECTED, DISCONNECTED };

struct Target {
    std::string ip;
    int port;
    size_t expected_bytes;
    int socket_fd;
    int foo;
    ServerState state;
    std::vector<uint8_t> buffer;
};

// Function to read JSON from file
json read_json_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    json j;
    file >> j;  // parse JSON directly
    return j;
}

int main() {

    try {
        json data = read_json_file("serverConfig.json");

        std::cout << "svr1Addr: " << data["svr1Addr"] << std::endl;
        std::cout << "svr1Port: " << data["svr1Port"] << std::endl;
        std::cout << "svr1DataBytes: " << data["svr1DataBytes"] << std::endl;
        std::cout << "svr2Addr: " << data["svr2Addr"] << std::endl;
        std::cout << "svr2Port: " << data["svr2Port"] << std::endl;
        std::cout << "svr2DataBytes: " << data["svr2DataBytes"] << std::endl;
        std::cout << "svr3Addr: " << data["svr3Addr"] << std::endl;
        std::cout << "svr3Port: " << data["svr3Port"] << std::endl;
        std::cout << "svr3DataBytes: " << data["svr3DataBytes"] << std::endl;
        std::cout << "svr4Addr: " << data["svr4Addr"] << std::endl;
        std::cout << "svr4Port: " << data["svr4Port"] << std::endl;
        std::cout << "svr4DataBytes: " << data["svr4DataBytes"] << std::endl;


        std::vector<Target> targets = {
            {data["svr1Addr"], data["svr1Port"] , data["svr1DataBytes"], -1, {}, DISCONNECTED, {}},
            {data["svr2Addr"], data["svr2Port"] , data["svr2DataBytes"], -1, {}, DISCONNECTED, {}},
            {data["svr3Addr"], data["svr3Port"] , data["svr3DataBytes"], -1, {}, DISCONNECTED, {}},
            {data["svr4Addr"], data["svr4Port"] , data["svr4DataBytes"], -1, {}, DISCONNECTED, {}}
        };

        std::cout << "Targets:\n";
        for (size_t i = 0; i < targets.size(); ++i) {
            std::cout << "Target " << i + 1 << ": "
                << targets[i].ip << ":" 
                << targets[i].port 
                << " (" << targets[i].expected_bytes << " bytes)"
                << " State: " << (targets[i].state == CONNECTED ? "CONNECTED" : "DISCONNECTED")
                << std::endl;
        }

        std::thread tcp_thread(tcp_client_thread, std::ref(targets));
        tcp_thread.join();

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }


    return 0;
}