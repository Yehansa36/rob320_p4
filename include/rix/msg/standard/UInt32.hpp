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

class UInt32 : public Message {
  public:
    uint32_t data;

    UInt32() = default;
    UInt32(const UInt32 &other) = default;
    ~UInt32() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x720d747016f03643ULL, 0x50fda753f6f4e461ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix