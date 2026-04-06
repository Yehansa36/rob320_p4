#pragma once

#include <assert.h>
#include <signal.h>
#include <unistd.h>

#include <functional>

#include "rix/ipc/interfaces/notification.hpp"
#include "rix/ipc/pipe.hpp"

namespace rix {
namespace ipc {

/**
 * @class Signal
 * @brief Asynchronous event handler for POSIX reliable signals.
 *
 */
class Signal : public interfaces::Notification {
   public:
    /**
     * @brief Construct a new Signal object. This function will throw a
     * `std::invalid_argument` error if `signum` is less than 1 or greater than
     * 32 or if another Signal object with the same value already exists.
     *
     * @param signum The signal number (must be between 1 and 32)
     */
    Signal(int signum);

    /**
     * @brief Destroy the Signal object. If the Signal is in a valid state,
     * resets the signal to default behavior.
     *
     */
    virtual ~Signal();

    /**
     * @brief Copy constructor is deleted because we cannot have multiple
     * references attempt to handle the same signal.
     */
    Signal(const Signal &other) = delete;

    /**
     * @brief Assignment operator is deleted because we cannot have multiple
     * references attempt to handle the same signal.
     */
    Signal &operator=(const Signal &other) = delete;

    /**
     * @brief Move constructor is allowed because the moved Signal will be put
     * in an invalid state.
     *
     * @param other
     */
    Signal(Signal &&other);

    /**
     * @brief Move assignment operator is allowed becuase the moved Signal will
     * be put into an invalid state. If the destination Signal is valid, then
     * the resource will be freed before assigning the new Signal.
     *
     * @param other
     * @return Signal&
     */
    Signal &operator=(Signal &&other);

    /**
     * @brief Raise the signal in the current process. If the Signal is in an
     * invalid state, returns `false` immediately. Returns `true` if raise
     * system call was successful.
     *
     */
    virtual bool raise() const;

    /**
     * @brief Send the signal to the process specified by `pid`. If the Signal
     * is in an invalid state, returns `false` immediately. Returns `true` if
     * kill system call was successful.
     *
     * @param pid The ID of the receiving process
     */
    bool kill(pid_t pid) const;

    /**
     * @brief Returns the numerical value of the Signal, or -1 if the Signal is
     * in an invalid state.
     *
     */
    int signum() const;

    /**
     * @brief Wait until the signal is received, or until the specified duration
     * elapses. If the Signal is in an invalid, returns `false` immediately.
     * Otherwise, returns `true` if the signal arrived within the specified
     * duration.
     *
     * @param d The maximum duration to wait for the signal to arrive.
     *
     */
    virtual bool wait(const rix::util::Duration &d) const;

   private:
    /**
     * @brief SignalNotifier struct contains a pipe and an initialization flag.
     * The pipe should be written to from within a signal handler function.
     *
     */
    struct SignalNotifier {
        SignalNotifier() : is_init(false) {};
        std::array<Pipe, 2> pipe; /**< 0: read end, 1: write end */
        bool is_init;             /**< false if SignalNotifier has not been initialized */
    };

    /**
     * @brief Static array of SignalNotifier structs. Elements of this array
     * will only be initialized once a Signal object is created with a signal
     * number that corresponds to an element in this array.
     *
     */
    static std::array<SignalNotifier, 32> notifier;

    /**
     * @brief The signal handler. This must be a static function because the
     * Standard C Library signal API requires the function to return void and
     * have a single integer argument. If this were a member function, it would
     * have an implicit Signal* argument. This implies that any data accessed
     * or modified by the handler must also be global or static.
     *
     * @warning This function must only invoke async-signal-safe functions. For
     * information on signal safety, consult:
     * https://www.man7.org/linux/man-pages/man7/signal-safety.7.html.
     *
     * @param signum The number of the received signal.
     */
    static void handler(int signum);

    int signum_;
};

}  // namespace ipc
}  // namespace rix