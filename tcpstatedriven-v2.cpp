#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

//////////////////// new //////////////////
#include "json.hpp"                 

enum ServerState { CONNECTED, DISCONNECTED };

//////////////////// new //////////////////
using json = nlohmann::json;        

struct Target {
    std::string ip;
    int port;
    size_t expected_bytes;
    int socket_fd;
    sockaddr_in addr;
    ServerState state;
    std::vector<uint8_t> buffer;
};

std::mutex buffer_mutex;
std::vector<uint8_t> combined_buffer;
std::vector<uint8_t> recv_buffer(20);

bool setup_connection(Target& target) {
    target.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (target.socket_fd < 0) return false;

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    setsockopt(target.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    target.addr.sin_family = AF_INET;
    target.addr.sin_port = htons(target.port);
    inet_pton(AF_INET, target.ip.c_str(), &target.addr.sin_addr);

    if (connect(target.socket_fd, (sockaddr*)&target.addr, sizeof(target.addr)) < 0) {
        close(target.socket_fd);
        target.socket_fd = -1;
        target.state = DISCONNECTED;
        return false;
    }

    target.state = CONNECTED;
    return true;
}

bool has_data_error(const std::vector<uint8_t>& buffer, size_t threshold = 40) {
    size_t zero_count = 0;
    for (uint8_t byte : buffer) {
        if (byte == 0) zero_count++;
    }
    return zero_count > threshold;
}

void send_to_serial(const std::vector<uint8_t>& data) {
    std::cout << "[Serial TX] ";
    for (auto byte : data) std::cout << std::hex << (int)byte << " ";
    std::cout << std::endl;
}

void receive_from_serial(std::vector<uint8_t>& buffer) {
    buffer.clear();
    for (int i = 0; i < 20; ++i) buffer.push_back(0xC0 + i); // Dummy data
    std::cout << "[Serial RX] ";
    for (auto byte : buffer) std::cout << std::hex << (int)byte << " ";
    std::cout << std::endl;
}

void tcp_client_thread(std::vector<Target>& targets) {
    enum State { TRANSMIT, RECEIVE };
    State current_state = TRANSMIT;
    int scan_count = 0;
    const std::string ping_msg = "ping";

    const size_t total_bytes = 80 + 52 + 52 + 52;
    combined_buffer.resize(total_bytes, 0);

    while (true) {
        auto cycle_start = std::chrono::high_resolution_clock::now();

        if (current_state == TRANSMIT) {
            std::vector<std::string> diagnostics;
            size_t offset = 0;

            for (auto& target : targets) {
                if (target.state == DISCONNECTED) {
                    if (!setup_connection(target)) {
                        std::fill(combined_buffer.begin() + offset,
                                  combined_buffer.begin() + offset + target.expected_bytes, 0);
                        diagnostics.push_back(target.ip + ": offline");
                        offset += target.expected_bytes;
                        continue;
                    }
                }

                send(target.socket_fd, ping_msg.c_str(), ping_msg.size(), 0);
                std::vector<uint8_t> buffer(target.expected_bytes, 0);
                ssize_t received = recv(target.socket_fd, buffer.data(), buffer.size(), 0);

                if (received > 0) {
                    buffer.resize(target.expected_bytes);
                    target.buffer = buffer;
                    std::copy(buffer.begin(), buffer.end(), combined_buffer.begin() + offset);

                    if (has_data_error(buffer)) {
                        diagnostics.push_back("DATA ERROR from " + target.ip);
                    } else {
                        diagnostics.push_back(target.ip + ": OK");
                    }
                } else {
                    close(target.socket_fd);
                    target.socket_fd = -1;
                    target.state = DISCONNECTED;
                    std::fill(combined_buffer.begin() + offset,
                              combined_buffer.begin() + offset + target.expected_bytes, 0);
                    diagnostics.push_back(target.ip + ": recv error");
                }

                offset += target.expected_bytes;
            }

            {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                send_to_serial(combined_buffer);
            }

            for (const auto& diag : diagnostics) std::cout << "[Diag] " << diag << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            current_state = RECEIVE;
        } else {
            {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                receive_from_serial(recv_buffer);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            current_state = TRANSMIT;
        }

        auto cycle_end = std::chrono::high_resolution_clock::now();
        auto cycle_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start).count();
        std::cout << "[Cycle " << scan_count << "] Duration: " << cycle_duration << " ms\n";
        scan_count++;
    }
}


//////////////////// new //////////////////
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

//////////////////// new //////////////////
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
