#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <functional>

#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/interfaces/io.hpp"

namespace rix {
namespace ipc {
namespace interfaces {

class Connection : public IO {
   public:
    Connection() = default;
    Connection(const Connection &other) = default;
    Connection &operator=(const Connection &other) = default;
    ~Connection() = default;

    virtual Endpoint remote_endpoint() const = 0;
    virtual Endpoint local_endpoint() const = 0;
    virtual bool ok() const = 0;
};

}  // namespace interfaces
}  // namespace ipc
}  // namespace rix