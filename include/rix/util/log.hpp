#pragma once

#ifndef RIX_UTIL_LOG_LEVEL
#define RIX_UTIL_LOG_LEVEL 1
#endif

#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <mutex>

#include "rix/util/time.hpp"

namespace rix {
namespace util {

namespace detail {

/**
 * @brief TeeBuffer class. This is used to write data to multiple streams at
 * once. This is used by the Log class to write data to both stdout and a log
 * file at the same time.
 *
 */
class TeeBuffer : public std::streambuf {
   public:
    TeeBuffer(std::vector<std::streambuf *> targets);

    int overflow(int c) override;
    int sync() override;
    void add(std::streambuf *target);

   private:
    std::vector<std::streambuf *> targets_;
};

inline TeeBuffer::TeeBuffer(std::vector<std::streambuf *> targets) : targets_(targets) {}
inline int TeeBuffer::overflow(int c) {
    if (c != EOF) {
        for (auto target : targets_) {
            if (target->sputc(c) == EOF) {
                return EOF;
            }
        }
    }
    return 0;
}
inline int TeeBuffer::sync() {
    int result = 0;
    for (auto target : targets_) {
        if (target->pubsync() == -1) {
            result = -1;
        }
    }
    return result;
}
inline void TeeBuffer::add(std::streambuf *target) { targets_.push_back(target); }

/**
 * @brief TeeStream class. This is used to write data to multiple streams at
 * once. This is used by the Log class to write data to both stdout and a log
 * file at the same time.
 *
 */
class TeeStream : public std::ostream {
   public:
    TeeStream(const TeeBuffer &tee_buffer);

   private:
    TeeBuffer tee_buffer_;
};

inline TeeStream::TeeStream(const TeeBuffer &tee_buffer) : std::ostream(&tee_buffer_), tee_buffer_(tee_buffer) {}

/**
 * @brief NullBuffer class. This is used as a fake stream that writes
 * no data to stdout/stderr without accumulating data in a stream.
 *
 */
class NullBuffer : public std::streambuf {
   public:
    int overflow(int c) override;
};

inline int NullBuffer::overflow(int c) { return c; }

}  // namespace detail

/**
 * @brief A class for logging messages with different log levels.
 */
class Log {
   public:
    /**
     * @brief Log level enum. These values are used as template parameters to
     * the LogStream class. They will be checked against the value of
     * RIX_UTIL_LOG_LEVEL at compile time to determine if data should be sent
     * to stdout/stderr.
     *
     */
    enum Level { DEBUG, INFO, WARN, ERROR, FATAL };

   private:
    /**
     * @brief static declaration of NullBuffer used by LogStream objects to
     * "write" to when the RIX_UTIL_LOG_LEVEL < level.
     *
     */
    inline static detail::NullBuffer null_buffer{};
    inline static std::ofstream logFile{};
    inline static detail::TeeBuffer tee_buffer{std::vector<std::streambuf *>{std::cout.rdbuf()}};
    inline static std::mutex mutex{};

    /**
     * @brief LogStream class. This class has a << operator that will append
     * header information to the data that is input to the stream. The level
     * template parameter is used to determine if data should be logged at
     * compile time. This minimizes runtime overhead when data should not be
     * logged.
     *
     * @tparam level
     */
    template <Level level>
    class LogStream {
       public:
        template <typename T>
        std::ostream &operator<<(const T &val);

        inline static std::ostream null_stream{&Log::null_buffer};
        inline static std::ostream tee_stream{&Log::tee_buffer};
        inline static std::mutex &mutex{Log::mutex};

        inline static std::string create_header(const Time &t);
        inline static std::string create_plain_header(const Time &t);
    };

   public:
    inline static void init(const std::string &name, bool logToFile = false);

    /**
     * The public LogStream objects. These are used to log inforamtion at the
     * corresponding level. If RIX_UTIL_LOG_LEVEL is greater than the template
     * level parameter, then no data will be logged when used.
     *
     */
    inline static LogStream<Level::DEBUG> debug;
    inline static LogStream<Level::INFO> info;
    inline static LogStream<Level::WARN> warn;
    inline static LogStream<Level::ERROR> error;
    inline static LogStream<Level::FATAL> fatal;

   private:
    inline static const std::string reset_color = "\033[0m";
    inline static const std::string bold = "\x1b[1m";
    inline static const std::string unbold = "\x1b[22m";
    inline static std::string name;
    inline static bool is_init{false};

    inline static std::string get_color_code(Level level);
    inline static std::string get_level_string(Level level);
};

template <Log::Level level>
template <typename T>
inline std::ostream &Log::LogStream<level>::operator<<(const T &val) {
    if (level < RIX_UTIL_LOG_LEVEL) {
        return null_stream;
    }

    std::lock_guard<std::mutex> guard(mutex);
    std::string header = create_header(Time::now());
    return tee_stream << header << val;
}

template <Log::Level level>
inline std::string Log::LogStream<level>::create_header(const Time &t) {
    std::stringstream ss;

    // Date field
    ss << "[" << t.to_string() << "] ";

    // Level field
    std::string level_str = "[" + bold + get_color_code(level) + get_level_string(level) + reset_color + "] ";
    ss << std::setw(21) << std::left << level_str;

    // Name field
    if (is_init) {
        ss << "[" << bold << name << unbold << "] ";
    }

    return ss.str();
}

template <Log::Level level>
inline std::string Log::LogStream<level>::create_plain_header(const Time &t) {
    std::stringstream ss;

    // Date field
    ss << "[" << t.to_string() << "] ";

    // Level field
    std::string level_str = "[" + get_level_string(level) + "] ";
    ss << std::setw(8) << std::left << level_str;

    // Name field
    if (is_init) {
        ss << "[" << name << "] ";
    }

    return ss.str();
}

inline void Log::init(const std::string &name, bool logToFile) {
    if (is_init) {
        return;
    }
    Log::name = name;
    if (logToFile) {
        const char *homeDir;
        if ((homeDir = getenv("HOME")) == NULL) {
            homeDir = getpwuid(getuid())->pw_dir;
        }
        std::string dirName = std::string(homeDir) + "/.rix/log/";
        struct stat st;
        if (stat(dirName.c_str(), &st) < 0) {
            if (mkdir(dirName.c_str(), S_IRWXU) < 0) {
                std::string err = strerror(errno);
                logToFile = false;
                is_init = true;
                return;
            }
        }
        logFile.open(dirName + name + "_" + std::to_string(Time::now().to_nanoseconds()) + ".log");
        tee_buffer.add(logFile.rdbuf());
    }
    is_init = true;
}

inline std::string Log::get_color_code(Level level) {
    switch (level) {
        case Level::DEBUG:
            return "\033[36m";  // Cyan
        case Level::INFO:
            return "\033[32m";  // Green
        case Level::WARN:
            return "\033[33m";  // Yellow
        case Level::ERROR:
            return "\033[31m";  // Red
        case Level::FATAL:
            return "\033[35m";  // Magenta
        default:
            return "\033[0m";  // Reset
    }
}

inline std::string Log::get_level_string(Level level) {
    switch (level) {
        case Level::DEBUG:
            return "DEBUG";  // Cyan
        case Level::INFO:
            return "INFO";  // Green
        case Level::WARN:
            return "WARN";  // Yellow
        case Level::ERROR:
            return "ERROR";  // Red
        case Level::FATAL:
            return "FATAL";  // Magenta
        default:
            return "";  // Reset
    }
}

}  // namespace util
}  // namespace rix