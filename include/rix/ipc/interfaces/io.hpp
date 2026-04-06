#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "rix/util/time.hpp"

namespace rix {
namespace ipc {
namespace interfaces {

class IO {
   public:
    IO() = default;
    IO(const IO &other) = default;
    IO &operator=(const IO &other) = default;
    virtual ~IO() = default;

    bool is_writable() const { return wait_for_writable(rix::util::Duration(0.0)); }
    bool is_readable() const { return wait_for_readable(rix::util::Duration(0.0)); }

    virtual ssize_t read(uint8_t *buffer, size_t len) const = 0;
    virtual ssize_t write(const uint8_t *buffer, size_t len) const = 0;
    virtual bool wait_for_writable(const rix::util::Duration &duration) const = 0;
    virtual bool wait_for_readable(const rix::util::Duration &duration) const = 0;
    virtual void set_nonblocking(bool status) = 0;
    virtual bool is_nonblocking() const = 0;
};

}  // namespace interfaces
}  // namespace ipc
}  // namespace rix