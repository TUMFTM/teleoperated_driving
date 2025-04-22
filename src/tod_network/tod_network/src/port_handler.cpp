/**
 * @file port_handler.hpp
 * @author Your Name
 * @brief This file contains the implementation of a `DynamicPortNumberManager` class that manages the allocation and release of dynamic port numbers within a specified range. It ensures thread-safe operations using a mutex to protect the shared state of allocated ports. The class includes functionality to persist the state of allocated ports to a file and load it from a file for restoring state between program executions.
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
 #include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <mutex> 
#include <set>
#include <algorithm>
#include <optional>

namespace tod_network {
    class DynamicPortNumberManager {
        // Assuming the rest of the class is defined as before
        std::set<int> allocatedPorts;
        int rangeStart, rangeEnd;
        std::mutex mtx;
    public:
        DynamicPortNumberManager(int start, int end)
            : rangeStart(start), rangeEnd(end) {}

        // Tries to find an available port, marks it as allocated, and returns it
        std::optional<int> allocatePort() {
            std::lock_guard<std::mutex> lock(mtx);
            for (int port = rangeStart; port <= rangeEnd; ++port) {
                if (allocatedPorts.find(port) == allocatedPorts.end()) {
                    // Port is available
                    allocatedPorts.insert(port);
                    return port;
                }
            }
            return {}; // No available port found
        }

        // Releases a previously allocated port, making it available again
        bool releasePort(int port) {
            auto it = allocatedPorts.find(port);
            if (it != allocatedPorts.end()) {
                allocatedPorts.erase(it);
                return true;
            }
            return false; // Port was not allocated
        }

        // Additional functionality to persist state
        void saveState(const std::string& filename) {
            std::lock_guard<std::mutex> lock(mtx);
            std::ofstream outFile(filename);
            for (int port : allocatedPorts) {
                outFile << port << std::endl;
            }
        }

        void loadState(const std::string& filename) {
            std::ifstream inFile(filename);
            std::string line;
            while (std::getline(inFile, line)) {
                std::istringstream iss(line);
                int port;
                if (iss >> port) {
                    allocatedPorts.insert(port);
                }
            }
        }
        void clearFile(const std::string& filename) 
        {
            std::ofstream ofs;
            ofs.open(filename, std::ofstream::out | std::ofstream::trunc); // Open and truncate the file
            if (!ofs.is_open()) {
                // Handle error
                std::cerr << "Failed to open file for clearing" << std::endl;
                return;
            }
            ofs.close(); // Closing the file automatically after clearing
        }
    };
} //namespace tod_network