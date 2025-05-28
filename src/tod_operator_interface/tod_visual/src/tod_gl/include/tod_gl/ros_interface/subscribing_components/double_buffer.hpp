/**
 * @file double_buffer.hpp
 * @brief Simple implementation of a double buffer for the ros abstraction @link https://en.wikipedia.org/wiki/Multiple_buffering#Double_buffering_in_computer_graphics
 * @copyright TUMFTM 2024
 **/

#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <ostream>

namespace tod_gl {
template <typename T>
class LockFreeDoubleBuffer {
  protected:
    struct alignas(64) Buffer {  // Align to cache line to prevent false sharing
        T data;
        std::atomic<bool> ready{false};
    };

    std::array<Buffer, 2> buffers;
    std::atomic<int> writeIndex{0};

  public:
    LockFreeDoubleBuffer() = default;
    virtual ~LockFreeDoubleBuffer() = default;

    LockFreeDoubleBuffer(const LockFreeDoubleBuffer& other) {
        std::cout << " Copy DoubleBuffer " << std::endl;
        buffers[0].data = other.buffers[0].data;
        buffers[1].data = other.buffers[1].data;
        buffers[0].ready.store(other.buffers[0].ready.load());
        buffers[1].ready.store(other.buffers[1].ready.load());
        writeIndex.store(other.writeIndex.load());
    }

    LockFreeDoubleBuffer(LockFreeDoubleBuffer&& other) noexcept {
        buffers[0].data = std::move(other.buffers[0].data);
        buffers[1].data = std::move(other.buffers[1].data);
        buffers[0].ready.store(other.buffers[0].ready.load());
        buffers[1].ready.store(other.buffers[1].ready.load());
        writeIndex.store(other.writeIndex.load());
    }

    LockFreeDoubleBuffer& operator=(const LockFreeDoubleBuffer& other) {
        if (this != &other) {
            buffers[0].data = other.buffers[0].data;
            buffers[1].data = other.buffers[1].data;
            buffers[0].ready.store(other.buffers[0].ready.load());
            buffers[1].ready.store(other.buffers[1].ready.load());
            writeIndex.store(other.writeIndex.load());
        }
        return *this;
    }

    LockFreeDoubleBuffer& operator=(LockFreeDoubleBuffer&& other) noexcept {
        if (this != &other) {
            buffers[0].data = std::move(other.buffers[0].data);
            buffers[1].data = std::move(other.buffers[1].data);
            buffers[0].ready.store(other.buffers[0].ready.load());
            buffers[1].ready.store(other.buffers[1].ready.load());
            writeIndex.store(other.writeIndex.load());
        }
        return *this;
    }

    friend void swap(LockFreeDoubleBuffer& first, LockFreeDoubleBuffer& second) noexcept {
        using std::swap;
        swap(first.buffers, second.buffers);
        int firstWriteIndex = first.writeIndex.load();
        int secondWriteIndex = second.writeIndex.load();
        first.writeIndex.store(secondWriteIndex);
        second.writeIndex.store(firstWriteIndex);
    }

    void write(const T& newData) {
        int currentWriteIndex = writeIndex.load(std::memory_order_relaxed);
        int nextWriteIndex = 1 - currentWriteIndex;

        buffers[nextWriteIndex].data = newData;
        buffers[nextWriteIndex].ready.store(true, std::memory_order_release);

        writeIndex.store(nextWriteIndex, std::memory_order_release);
    }

    T read() const {
        int currentReadIndex = 1 - writeIndex.load(std::memory_order_acquire);

        while (!buffers[currentReadIndex].ready.load(std::memory_order_acquire)) {
            currentReadIndex = 1 - writeIndex.load(std::memory_order_acquire);
        }

        return buffers[currentReadIndex].data;
    }

    const T* readPtr() const {
        int currentReadIndex = 1 - writeIndex.load(std::memory_order_acquire);

        while (!buffers[currentReadIndex].ready.load(std::memory_order_acquire)) {
            currentReadIndex = 1 - writeIndex.load(std::memory_order_acquire);
        }

        return &buffers[currentReadIndex].data;
    }
};
}  // namespace tod_gl