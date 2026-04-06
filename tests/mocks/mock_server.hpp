#pragma once

#include <gmock/gmock.h>

#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <vector>

#include "mock_connection.hpp"
#include "mock_io.hpp"
#include "rix/ipc/interfaces/server.hpp"

using ::testing::NiceMock;

class MockServer : public rix::ipc::interfaces::Server {
   public:
    using ServerMap = std::shared_ptr<std::map<rix::ipc::Endpoint, NiceMock<MockServer> *>>;
    static inline ServerMap server_map{};
    static inline std::shared_ptr<std::mutex> server_mutex{};

    MockServer(const rix::ipc::Endpoint &endpoint) : endpoint(endpoint), nonblocking(false) {
        server_mutex->lock();
        server_map->insert({endpoint, (NiceMock<MockServer>*)(this)});
        server_mutex->unlock();
        ON_CALL(*this, accept)
            .WillByDefault([this](std::weak_ptr<rix::ipc::interfaces::Connection> &connection) -> bool {
                if (!nonblocking) {
                    wait_for_accept(rix::util::Duration::safe_forever());
                }

                std::lock_guard<std::mutex> guard(accept_notifier.mtx);
                if (accept_queue.empty()) {
                    return false;
                }
                auto cb = accept_queue.front();
                accept_queue.pop();
                cb.endpoints->at(0).address = this->endpoint.address;
                cb.endpoints->at(0).port = std::rand();
                auto ptr = std::make_shared<testing::NiceMock<MockConnection>>(cb.in, cb.out, cb.endpoints);
                connections.insert(ptr);
                connection = ptr;

                rix::ipc::Endpoint ep(cb.endpoints->at(1));
                auto &cn = client_notifiers[ep];
                std::lock_guard<std::mutex> cn_guard(cn.mtx);
                cn.cond = true;
                cn.cv.notify_all();

                return true;
            });
        ON_CALL(*this, wait_for_accept).WillByDefault([this](const rix::util::Duration &d) -> bool {
            // Waits for a call to make_connection by the client
            std::unique_lock<std::mutex> lock(accept_notifier.mtx);
            if (!accept_queue.empty()) return true;
            return accept_notifier.cv.wait_for(lock, d.get(), [&]() { return !accept_queue.empty(); });
        });
        ON_CALL(*this, close).WillByDefault([this](const std::weak_ptr<rix::ipc::interfaces::Connection> &connection) {
            auto ptr = connection.lock();
            if (ptr) {
                connections.erase(ptr);
            }
        });
        ON_CALL(*this, ok).WillByDefault([this]() -> bool { return true; });
        ON_CALL(*this, local_endpoint).WillByDefault([this]() -> rix::ipc::Endpoint { return this->endpoint; });
        ON_CALL(*this, set_nonblocking).WillByDefault([this](bool status) -> void { this->nonblocking = status; });
        ON_CALL(*this, is_nonblocking).WillByDefault([this]() -> bool { return this->nonblocking; });
    }

    ~MockServer() {
        server_mutex->lock();
        server_map->erase(endpoint);
        server_mutex->unlock();
    }

    MOCK_METHOD(bool, accept, (std::weak_ptr<rix::ipc::interfaces::Connection> & connection), (override));
    MOCK_METHOD(void, close, (const std::weak_ptr<rix::ipc::interfaces::Connection> &connection), (override));
    MOCK_METHOD(bool, ok, (), (const, override));
    MOCK_METHOD(rix::ipc::Endpoint, local_endpoint, (), (const, override));
    MOCK_METHOD(bool, wait_for_accept, (rix::util::Duration duration), (const, override));
    MOCK_METHOD(void, set_nonblocking, (bool status), (override));
    MOCK_METHOD(bool, is_nonblocking, (), (const, override));

    void make_connection(std::shared_ptr<std::array<rix::ipc::Endpoint, 2>> endpoints, std::shared_ptr<MockIO> &out,
                         std::shared_ptr<MockIO> &in) {
        std::lock_guard<std::mutex> guard(accept_notifier.mtx);
        client_notifiers[endpoints->at(1)];
        accept_queue.push({in, out, endpoints});
        accept_notifier.cv.notify_all();
    }

    bool wait_for_connection(const rix::ipc::Endpoint &endpoint, const rix::util::Duration &d) {
        // Waits for a call to accept by the server
        Notifier *cn;
        {
            std::lock_guard<std::mutex> guard(accept_notifier.mtx);
            cn = &client_notifiers[endpoint];
        }
        std::unique_lock<std::mutex> lock(cn->mtx);
        if (cn->cond) return true;
        auto tp = rix::util::Clock::now().time_since_epoch() + d.get();
        return cn->cv.wait_for(lock, d.get(), [&]() -> bool { return cn->cond; });
    }

   private:
    std::set<std::shared_ptr<rix::ipc::interfaces::Connection>> connections;

    struct Notifier {
        std::mutex mtx;
        std::condition_variable cv;
        bool cond = false;
    };
    std::map<rix::ipc::Endpoint, Notifier> client_notifiers;  // Key type is client local endpoint, this is used to
                                                              // notify a client when its connection has been accepted

    struct ConnectionBuffer {
        std::shared_ptr<MockIO> out;                                   // Points to a MockClient::in
        std::shared_ptr<MockIO> in;                                    // Points to a MockClient::out
        std::shared_ptr<std::array<rix::ipc::Endpoint, 2>> endpoints;  // 0: connection local (client remote), 1:
                                                                       // connection remote (client local)
    };
    std::queue<ConnectionBuffer> accept_queue;
    Notifier accept_notifier;

    rix::ipc::Endpoint endpoint;
    bool nonblocking;
};