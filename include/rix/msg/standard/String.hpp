#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"

namespace rix {
namespace msg {
namespace standard {

class String : public Message {
  public:
    std::string data;

    String() = default;
    String(const String &other) = default;
    ~String() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x607fa99bddbe73e0ULL, 0x32708827f11f4800ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix