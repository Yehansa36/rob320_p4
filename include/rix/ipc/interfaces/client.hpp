#pragma once

#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/interfaces/connection.hpp"

namespace rix {
namespace ipc {
namespace interfaces {

class Client : public Connection {
   public:
    Client() = default;
    Client(const Client &other) = default;
    Client &operator=(const Client &other) = default;
    virtual ~Client() = default;

    bool is_connected() const { return wait_for_connect(rix::util::Duration(0.0)); }

    virtual bool connect(const Endpoint &endpoint) = 0;
    virtual bool wait_for_connect(const rix::util::Duration &duration) const = 0;
    virtual void reset() = 0;

};

}  // namespace interfaces
}  // namespace ipc
}  // namespace rix