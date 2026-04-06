#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "rix/util/time.hpp"

namespace rix {
namespace ipc {
namespace interfaces {

class Notification {
   public:
    Notification() = default;
    Notification(const Notification &other) = default;
    Notification &operator=(const Notification &other) = default;
    virtual ~Notification() = default;

    bool is_ready() const { return wait(rix::util::Duration(0.0)); }
    virtual bool raise() const = 0;
    virtual bool wait(const rix::util::Duration &duration) const = 0;
};

}  // namespace interfaces
}  // namespace ipc
}  // namespace rix