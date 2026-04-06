#pragma once

#include <gmock/gmock.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "rix/ipc/interfaces/io.hpp"

class MockIO : public rix::ipc::interfaces::IO {
   public:
    MockIO(size_t capacity) : capacity(capacity), write_end_open(true), read_end_open(true) {
        ON_CALL(*this, read).WillByDefault([this](uint8_t *dst, size_t len) -> ssize_t {
            size_t p = buffer.size();
            // Simulate the behavior of a POSIX FIFO
            if (p == 0) {
                if (write_end_open) {
                    if (nonblocking) {
                        // Return 0 if there is no data, the write end is open, and we are in nonblocking mode
                        return 0;
                    } else {
                        // Block if there is no data, the write end is open, and we are in blocking mode
                        wait_for_readable(rix::util::Duration::safe_forever());
                    }
                } else {
                    // Return 0 if the write end is closed
                    return 0;
                }
            }
            len = std::min(p, len);
            std::memcpy(dst, buffer.data(), len);
            buffer.erase(buffer.begin(), buffer.begin() + len);
            return len;
        });
        ON_CALL(*this, write).WillByDefault([this](const uint8_t *src, size_t len) -> ssize_t {
            if (!read_end_open) return -1;
            size_t remaining = this->capacity - buffer.size();
            if (remaining == 0) {
                if (nonblocking) {
                    // Return 0 if there is no room in the buffer and we are in non-blocking mode
                    return 0;
                } else {
                    // Block if there is no room in the buffer and we are in blocking mode
                    wait_for_writable(rix::util::Duration::safe_forever());
                    remaining = this->capacity - buffer.size();
                }
            }
            len = std::min(remaining, len);
            // Assume that space is always available and writes are atomic
            const size_t offset = buffer.size();
            buffer.resize(offset + len);
            std::memcpy(buffer.data() + offset, src, len);
            cv.notify_all();
            return len;
        });
        ON_CALL(*this, wait_for_writable).WillByDefault([this](const rix::util::Duration &d) -> bool {
            auto pred = [&]() -> bool { return read_end_open && buffer.size() < this->capacity; };
            if (pred()) return true;
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait_for(lock, d.get(), pred);
            return pred();
        });
        ON_CALL(*this, wait_for_readable).WillByDefault([this](const rix::util::Duration &d) -> bool {
            auto pred = [&]() -> bool { return buffer.size() > 0 || (!write_end_open && buffer.empty()); };
            if (pred()) return true;
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait_for(lock, d.get(), pred);
            return pred();
        });
        ON_CALL(*this, set_nonblocking).WillByDefault([this](bool status) -> void { nonblocking = status; });
        ON_CALL(*this, is_nonblocking).WillByDefault([this]() -> bool { return nonblocking; });
    }

    MOCK_METHOD(ssize_t, read, (uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(ssize_t, write, (const uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(bool, wait_for_writable, (const rix::util::Duration &duration), (const, override));
    MOCK_METHOD(bool, wait_for_readable, (const rix::util::Duration &duration), (const, override));
    MOCK_METHOD(void, set_nonblocking, (bool status), (override));
    MOCK_METHOD(bool, is_nonblocking, (), (const, override));

    void close_write_end() { 
        write_end_open = false; 
        cv.notify_all();
    }
    void close_read_end() { read_end_open = false; }

    const std::vector<uint8_t> &get_buffer() const { return buffer; };

   private:
    std::vector<uint8_t> buffer;
    std::mutex mtx;
    std::condition_variable cv;
    size_t capacity;
    bool nonblocking;
    bool write_end_open;
    bool read_end_open;
};
