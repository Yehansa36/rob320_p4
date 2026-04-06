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

class Int16 : public Message {
  public:
    int16_t data;

    Int16() = default;
    Int16(const Int16 &other) = default;
    ~Int16() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x407381f3637b2776ULL, 0xfbef4993d27d03d5ULL};
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