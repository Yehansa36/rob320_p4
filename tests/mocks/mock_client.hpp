#pragma once

#include <gmock/gmock.h>

#include <condition_variable>
#include <mutex>
#include <random>
#include <vector>

#include "mock_io.hpp"
#include "mock_server.hpp"
#include "rix/ipc/interfaces/client.hpp"

using ::testing::NiceMock;

class MockClient : public rix::ipc::interfaces::Client {
   public:
    using ServerMap = std::shared_ptr<std::map<rix::ipc::Endpoint, NiceMock<MockServer>*>>;
    static inline ServerMap server_map{};
    static inline std::shared_ptr<std::mutex> server_mutex{};
    static inline size_t capacity{1024};
    static inline std::string address{""};

    MockClient()
        : nonblocking(false), is_connecting(false), connected(false) {
        endpoints = std::make_shared<std::array<rix::ipc::Endpoint, 2>>();
        endpoints->at(1).address = address;
        endpoints->at(1).port = std::rand();
        in = std::make_shared<testing::NiceMock<MockIO>>(capacity);
        out = std::make_shared<testing::NiceMock<MockIO>>(capacity);

        ON_CALL(*this, read).WillByDefault([this](uint8_t *dst, size_t len) -> ssize_t {
            if (!nonblocking) {
                in->wait_for_readable(rix::util::Duration::safe_forever());
            }
            return in->read(dst, len);
        });
        ON_CALL(*this, write).WillByDefault([this](const uint8_t *src, size_t len) -> ssize_t {
            if (!nonblocking) {
                out->wait_for_writable(rix::util::Duration::safe_forever());
            }
            return out->write(src, len);
        });
        ON_CALL(*this, wait_for_writable).WillByDefault([this](const rix::util::Duration &d) -> bool {
            return out->wait_for_writable(d);
        });
        ON_CALL(*this, wait_for_readable).WillByDefault([this](const rix::util::Duration &d) -> bool {
            return in->wait_for_readable(d);
        });

        ON_CALL(*this, set_nonblocking).WillByDefault([this](bool status) -> void { nonblocking = status; });
        ON_CALL(*this, is_nonblocking).WillByDefault([this]() -> bool { return nonblocking; });

        ON_CALL(*this, remote_endpoint).WillByDefault([this]() -> rix::ipc::Endpoint { return endpoints->at(0); });
        ON_CALL(*this, local_endpoint).WillByDefault([this]() -> rix::ipc::Endpoint { return endpoints->at(1); });
        ON_CALL(*this, ok).WillByDefault([this]() -> bool { return true; });

        ON_CALL(*this, connect).WillByDefault([this](const rix::ipc::Endpoint &endpoint) -> bool {
            if (connected) {
                return false;
            }

            if (nonblocking && is_connecting) {
                connected = wait_for_connect(rix::util::Duration(0));
                is_connecting = !connected;
                return connected;
            }

            server_mutex->lock();
            auto it = server_map->find(endpoint);
            if (it == server_map->end()) return false;
            server = it->second;
            server_mutex->unlock();

            server->make_connection(endpoints, in, out);
            is_connecting = true;

            if (!nonblocking) {
                return connected = wait_for_connect(rix::util::Duration::safe_forever());
            }
            return false;
        });
        ON_CALL(*this, wait_for_connect).WillByDefault([this](const rix::util::Duration &d) -> bool {
            if (connected) return true;
            if (!is_connecting) return false;
            return server->wait_for_connection(endpoints->at(1), d);
        });
        ON_CALL(*this, reset).WillByDefault([this](){
            endpoints = std::make_shared<std::array<rix::ipc::Endpoint, 2>>();
            endpoints->at(1).address = "mock_client_address";
            endpoints->at(1).port = std::rand();
            in = std::make_shared<testing::NiceMock<MockIO>>(capacity);
            out = std::make_shared<testing::NiceMock<MockIO>>(capacity);
            connected = false;
            is_connecting = false;
        });
    }

    // IO interface methods
    MOCK_METHOD(ssize_t, read, (uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(ssize_t, write, (const uint8_t *buffer, size_t len), (const, override));
    MOCK_METHOD(bool, wait_for_writable, (const rix::util::Duration &duration), (const, override));
    MOCK_METHOD(bool, wait_for_readable, (const rix::util::Duration &duration), (const, override));
    MOCK_METHOD(void, set_nonblocking, (bool status), (override));
    MOCK_METHOD(bool, is_nonblocking, (), (const, override));

    // Connection interface methods
    MOCK_METHOD(rix::ipc::Endpoint, remote_endpoint, (), (const, override));
    MOCK_METHOD(rix::ipc::Endpoint, local_endpoint, (), (const, override));
    MOCK_METHOD(bool, ok, (), (const, override));

    // Client interface methods
    MOCK_METHOD(bool, connect, (const rix::ipc::Endpoint &endpoint), (override));
    MOCK_METHOD(bool, wait_for_connect, (const rix::util::Duration &duration), (const, override));
    MOCK_METHOD(void, reset, (), (override));

   private:
    bool nonblocking;
    bool is_connecting;
    bool connected;
    NiceMock<MockServer> *server;
    std::shared_ptr<MockIO> out;                                   // Points to a MockConnection::in
    std::shared_ptr<MockIO> in;                                    // Points to a MockConnection::out
    std::shared_ptr<std::array<rix::ipc::Endpoint, 2>> endpoints;  // 0: remote, 1: local
};