#pragma once

#include <gmock/gmock.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "mock_io.hpp"
#include "rix/ipc/interfaces/connection.hpp"

class MockConnection : public rix::ipc::interfaces::Connection {
   public:
    MockConnection(std::shared_ptr<MockIO> out, std::shared_ptr<MockIO> in,
                   std::shared_ptr<std::array<rix::ipc::Endpoint, 2>> endpoints)
        : nonblocking(false), out(out), in(in), endpoints(endpoints) {
        ON_CALL(*this, read).WillByDefault([this](uint8_t *dst, size_t len) -> ssize_t {
            if (!nonblocking) {
                this->in->wait_for_readable(rix::util::Duration::safe_forever());
            }
            return this->in->read(dst, len);
        });
        ON_CALL(*this, write)
            .WillByDefault([this](const uint8_t *src, size_t len) -> ssize_t {
                if (!nonblocking) {
                    this->out->wait_for_writable(rix::util::Duration::safe_forever());
                }
                return this->out->write(src, len);
            });
        ON_CALL(*this, wait_for_writable)
            .WillByDefault([this](const rix::util::Duration &d) -> bool {
                return this->out->wait_for_writable(d);
            });
        ON_CALL(*this, wait_for_readable)
            .WillByDefault([this](const rix::util::Duration &d) -> bool {
                return this->in->wait_for_readable(d);
            });

        ON_CALL(*this, set_nonblocking).WillByDefault([this](bool status) -> void {
            nonblocking = status;
        });
        ON_CALL(*this, is_nonblocking).WillByDefault([this]() -> bool {
            return nonblocking;
        });

        ON_CALL(*this, remote_endpoint).WillByDefault([this]() -> rix::ipc::Endpoint {
            return this->endpoints->at(1);
        });
        ON_CALL(*this, local_endpoint).WillByDefault([this]() -> rix::ipc::Endpoint {
            return this->endpoints->at(0);
        });
        ON_CALL(*this, ok).WillByDefault([this]() -> bool { return true; });
    }

    // IO interface methods
    MOCK_METHOD(ssize_t, read, (uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(ssize_t, write, (const uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(bool, wait_for_writable, (const rix::util::Duration &duration),
                (const, override));
    MOCK_METHOD(bool, wait_for_readable, (const rix::util::Duration &duration),
                (const, override));
    MOCK_METHOD(void, set_nonblocking, (bool status), (override));
    MOCK_METHOD(bool, is_nonblocking, (), (const, override));

    // Connection interface methods
    MOCK_METHOD(rix::ipc::Endpoint, remote_endpoint, (), (const, override));
    MOCK_METHOD(rix::ipc::Endpoint, local_endpoint, (), (const, override));
    MOCK_METHOD(bool, ok, (), (const, override));

   private:
    bool nonblocking;
    std::shared_ptr<MockIO> out;  // Points to a MockClient::in
    std::shared_ptr<MockIO> in;   // Points to a MockClient::out
    std::shared_ptr<std::array<rix::ipc::Endpoint, 2>> endpoints;  // 0: local, 1: remote
};