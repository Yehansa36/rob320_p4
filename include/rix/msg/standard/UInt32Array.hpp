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

class UInt32Array : public Message {
  public:
    std::vector<uint32_t> data;

    UInt32Array() = default;
    UInt32Array(const UInt32Array &other) = default;
    ~UInt32Array() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number_vector(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xaefe9199dbb27e93ULL, 0xd8ccb06b7c8ad5d4ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number_vector(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number_vector(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix