#pragma once

#include "rix/ipc/interfaces/notification.hpp"

namespace rix {
namespace core {
namespace interfaces {

class Spinner {
   public:
    Spinner() = default;
    Spinner(const Spinner &other) = default;
    Spinner &operator=(const Spinner &other) = default;
    virtual ~Spinner() = default;

    void spin() {
        while (ok()) spin_once();
    }

    void spin(std::shared_ptr<rix::ipc::interfaces::Notification> notif) {
        while (ok()) {
            if (notif->is_ready()) {
                shutdown();
                continue;
            }
            spin_once();
        }
    }

    /**
     * @brief Returns true if loop should continue, false if loop should stop.
     *
     */
    virtual void spin_once() = 0;

    /**
     * @brief Returns true if shutdown has not been called and the constructor
     * created the object without error.
     *
     */
    virtual bool ok() const = 0;

    /**
     * @brief Shuts down the object. ok() will return false after this call.
     *
     */
    virtual void shutdown() = 0;
};

}  // namespace interfaces
}  // namespace core
}  // namespace rix