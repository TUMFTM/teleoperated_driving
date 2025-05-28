
/**
 * @file debug.hpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <cxxabi.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <array>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace tod_gl {

#if !defined(NDEBUG)
#define PRINT_STACK_TRACE(message)                                                                 \
    do {                                                                                           \
        std::cerr << message << std::endl;                                                         \
        void* array[500];                                                                          \
        int size = backtrace(array, 500);                                                          \
        char** messages = backtrace_symbols(array, size);                                          \
        std::cerr << "Stack trace:" << std::endl;                                                  \
        for (int i = 1; i < size && messages != NULL; ++i) {                                       \
            std::string msg(messages[i]);                                                          \
            std::string::size_type pos = msg.find("(");                                            \
            std::string mangled_name = (pos == std::string::npos) ? "" : msg.substr(pos + 1);      \
            std::string::size_type end = mangled_name.find("+");                                   \
            if (end != std::string::npos)                                                          \
                mangled_name = mangled_name.substr(0, end);                                        \
            int status;                                                                            \
            char* demangled_name = abi::__cxa_demangle(mangled_name.c_str(), NULL, NULL, &status); \
            std::stringstream ss;                                                                  \
            ss << "addr2line -e " << msg.substr(0, pos - 1) << " " << array[i];                    \
            std::string cmd = ss.str();                                                            \
            std::array<char, 2048> buffer;                                                         \
            std::string result;                                                                    \
            std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);        \
            if (!pipe) {                                                                           \
                std::cerr << "popen() failed!" << std::endl;                                       \
            } else {                                                                               \
                while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {               \
                    result += buffer.data();                                                       \
                }                                                                                  \
            }                                                                                      \
            if (!result.empty() && result[result.length() - 1] == '\n') {                          \
                result.erase(result.length() - 1);                                                 \
            }                                                                                      \
            std::cerr << "[bt]: (" << i << ") " << msg << std::endl;                               \
            if (demangled_name) {                                                                  \
                std::cerr << "    demangled: " << demangled_name << std::endl;                     \
                free(demangled_name);                                                              \
            }                                                                                      \
            if (!result.empty() && result != "??:0") {                                             \
                std::cerr << "    location: " << result << std::endl;                              \
            }                                                                                      \
        }                                                                                          \
        free(messages);                                                                            \
    } while (0)
#else
#define PRINT_STACK_TRACE(message)
#endif

} // namespace tod_gl