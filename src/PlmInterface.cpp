// #include "../include/PlmInterface.hpp"
#include <fstream>
#include <iostream>
#include "../include/json.hpp"

// For convenience
using json = nlohmann::json;

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
        json data = read_json_file("../config.json");

        // Example: print full JSON
        std::cout << data.dump(4) << std::endl;

        int networkID = data["networkID"];
        std::cout << "NetworkID: " << data["networkID"] << std::endl;
        std::cout << "txPath: " << data["serialPortTx"] << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }

    return 0;
}