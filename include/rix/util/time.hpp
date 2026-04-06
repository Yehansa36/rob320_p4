#pragma once

#include <chrono>
#include <cmath>
#include <string>

#include "rix/msg/standard/Duration.hpp"
#include "rix/msg/standard/Time.hpp"

namespace rix {
namespace util {

using Clock = std::chrono::system_clock;

class Duration;  // Forward declaration

class Time {
   public:
    using Type = std::chrono::time_point<Clock, std::chrono::nanoseconds>;
    static Time now();
    static Time max() { return Time(Clock::time_point::max()); }
    static Time min() { return Time(Clock::time_point::min()); }

    Time();
    Time(const Type &time_point);
    Time(double seconds);
    explicit Time(int32_t seconds, int32_t nanoseconds);
    explicit Time(const rix::msg::standard::Time &msg);

    Time(const Time &other);
    Time &operator=(const Time &other);

    Time operator+(const Duration &other) const;
    Time operator-(const Duration &other) const;
    Duration operator-(const Time &other) const;

    Time &operator+=(const Duration &other);
    Time &operator-=(const Duration &other);
    bool operator==(const Time &other) const;
    bool operator!=(const Time &other) const;
    bool operator<(const Time &other) const;
    bool operator<=(const Time &other) const;
    bool operator>(const Time &other) const;
    bool operator>=(const Time &other) const;

    std::string to_string(bool local_time = false) const;
    rix::msg::standard::Time to_msg();

    enum RoundType { FLOOR = 0, CEIL, NEAREST };

    int64_t to_seconds(RoundType type = RoundType::FLOOR) const;
    int64_t to_milliseconds(RoundType type = RoundType::FLOOR) const;
    int64_t to_microseconds(RoundType type = RoundType::FLOOR) const;
    int64_t to_nanoseconds() const;

    const Type &get() const;
    Type &get();

   private:
    Type tp;
};

class Duration {
   public:
    using Type = std::chrono::nanoseconds;

    static Duration max() { return Duration(std::chrono::nanoseconds::max()); }
    static Duration min() { return Duration(std::chrono::nanoseconds::min()); }

    /**
     * @brief Duration to be used to represent "forever" without causing integer
     * overflow from addition to a time point near now. Currently is 10 years.
     * 
     * @return Duration 
     */
    static Duration safe_forever() { return Duration(std::chrono::nanoseconds(315'360'000'000'000'000)); } // 10 years in nanoseconds

    Duration();
    explicit Duration(const rix::msg::standard::Duration &msg);
    Duration(const Type &duration);
    Duration(double seconds);
    explicit Duration(int32_t seconds, int32_t nanoseconds);

    Duration(const Duration &other);
    Duration &operator=(const Duration &other);

    Time operator+(const Time &other) const;
    Duration operator+(const Duration &other) const;
    Duration operator-(const Duration &other) const;
    Duration operator*(double factor) const;
    Duration operator/(double factor) const;
    Duration operator*(int factor) const;
    Duration operator/(int factor) const;
    Duration operator-() const;
    Duration &operator+=(const Duration &other);
    Duration &operator-=(const Duration &other);
    Duration &operator*=(double factor);
    Duration &operator/=(double factor);
    Duration &operator*=(int factor);
    Duration &operator/=(int factor);
    bool operator==(const Duration &other) const;
    bool operator!=(const Duration &other) const;
    bool operator<(const Duration &other) const;
    bool operator<=(const Duration &other) const;
    bool operator>(const Duration &other) const;
    bool operator>=(const Duration &other) const;

    rix::msg::standard::Duration to_msg();

    int64_t to_seconds(Time::RoundType type = Time::RoundType::FLOOR) const;
    int64_t to_milliseconds(Time::RoundType type = Time::RoundType::FLOOR) const;
    int64_t to_microseconds(Time::RoundType type = Time::RoundType::FLOOR) const;
    int64_t to_nanoseconds() const;

    const Type &get() const;
    Type &get();

   private:
    Type d;
};

/**
 * @brief Sleep for a given duration
 * @param duration The duration to sleep for.
 */
void sleep_for(const Duration &duration);

/**
 * @brief Sleep until a given time
 * @param time The time to sleep until.
 */
void sleep_until(const Time &time);

/**
 * @brief A class for measuring time.
 */
class Timer {
   public:
    /**
     * @brief Constructs a Timer object.
     */
    Timer();

    Timer(const Timer &other);
    Timer &operator=(const Timer &other);

    /**
     * @brief Starts the timer.
     */
    void start();

    /**
     * @brief Stops the timer.
     */
    void stop();

    /**
     * @brief Gets the elapsed duration.
     * @return The elapsed duration.
     */
    Duration get() const;

   private:
    Time start_;  //< The start time.
    Time end_;    //< The end time.
};

/**
 * @brief A class for setting the rate of a loop.
 *
 * @example shared_mutex.cpp
 * @example tcp_client.cpp
 * @example tcp_client_noblock.cpp
 * @example udp_sender_multicast.cpp
 * @example udp_receiver.cpp
 * @example web_client.cpp
 */
class Rate {
   public:
    static inline double min_frequency() { return (1e9 / std::chrono::nanoseconds::max().count()); }
    static inline double max_frequency() { return 1e9; }
    static inline Duration min_period() { return Duration(0, 1); }
    static inline Duration max_period() { return Duration(std::chrono::nanoseconds::max()); }

    Rate();
    /**
     * @brief Constructs a Rate object.
     * @param frequency The frequency in hertz.
     */
    explicit Rate(double frequency);  //(Done By Waj) Handle case of 0 frequency (infinite duration)
    explicit Rate(Duration period);   //(Done By Waj) Handle case of 0 duration (infinite frequency)

    Rate(const Rate &other);
    Rate &operator=(const Rate &other);

    /**
     * @brief Sleeps for the time required to maintain the rate.
     * @return True if the sleep duration was met, false otherwise.
     */
    bool sleep();

    Duration period() const;
    void set_period(const Duration &period);
    double frequency() const;
    void set_frequency(double frequency);

   private:
    Duration period_;
    Time start_;
};

}  // namespace util
}  // namespace rix