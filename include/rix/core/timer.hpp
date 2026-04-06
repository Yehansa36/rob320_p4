#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include "rix/core/common.hpp"
#include "rix/core/interfaces/spinner.hpp"

namespace rix {
namespace core {

class Timer : public interfaces::Spinner {
   public:
    struct Event {
        rix::util::Time last_expected;
        rix::util::Time last_real;
        rix::util::Time current_expected;
        rix::util::Time current_real;
        rix::util::Duration last_duration;
    };

    using Callback = std::function<void(const Event &event)>;

    Timer(const rix::util::Duration &duration, Callback callback);
    Timer(const Timer &) = delete;
    Timer &operator=(const Timer &) = delete;
    ~Timer();

    /**
     * @brief Returns true if the Subscriber has not been shut down.
     *
     */
    virtual bool ok() const override;

    /**
     * @brief Stops the spin loop. ok() will return false after this function
     * is called.
     *
     */
    virtual void shutdown() override;

    /**
     * @brief A single iteration of the timer loop. The callback will only
     * be called if the timer duration has passed since its last calling.
     *
     */
    virtual void spin_once() override;

    /**
     * @brief Set the callback for the timer.
     *
     * @param callback The callback function to be invoked.
     *
     */
    void set_callback(Callback callback);

    /**
     * @brief Returns the callback for this timer.
     *
     * @return Callback
     */
    Callback get_callback() const;

   private:
    rix::util::Duration duration_;
    Event event_;
    Callback callback_;
    std::mutex callback_mutex_;
    std::atomic<bool> shutdown_flag_;
};

}  // namespace core
}  // namespace rix