#include "PlmInterface.hpp"
#include <iostream>
#include <string>

void printUsage(const std::string& programName) {
    std::cout << "Usage: " << programName << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -s, --sender PORT    Specify sender serial port (default: /dev/ttyS0)\n";
    std::cout << "  -r, --receiver PORT  Specify receiver serial port (default: /dev/ttyS1)\n";
    std::cout << "  -h, --help           Display this help message\n";
    std::cout << "\nExample:\n";
    std::cout << "  " << programName << " -s /dev/ttyUSB0 -r /dev/ttyUSB1\n";
}


int main(int argc, char *argv[]) {
    std::string senderPort = "/dev/ttyS0";
    std::string receiverPort = "/dev/ttyS1";

}