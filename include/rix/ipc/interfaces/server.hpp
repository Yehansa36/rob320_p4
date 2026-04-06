#pragma once
#include <memory>

#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/interfaces/connection.hpp"

namespace rix {
namespace ipc {
namespace interfaces {

class Server {
   public:
    Server() = default;
    Server(const Server &other) = default;
    Server &operator=(const Server &other) = default;
    virtual ~Server() = default;

    virtual bool accept(std::weak_ptr<Connection> &connection) = 0;
    virtual bool wait_for_accept(rix::util::Duration duration) const = 0;
    virtual void close(const std::weak_ptr<Connection> &connection) = 0;
    virtual bool ok() const = 0;
    virtual Endpoint local_endpoint() const = 0;
    virtual void set_nonblocking(bool status) = 0;
    virtual bool is_nonblocking() const = 0;
};

}  // namespace interfaces
}  // namespace ipc
}  // namespace rix