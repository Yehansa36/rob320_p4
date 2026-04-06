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

class UInt8Array : public Message {
  public:
    std::vector<uint8_t> data;

    UInt8Array() = default;
    UInt8Array(const UInt8Array &other) = default;
    ~UInt8Array() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number_vector(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x982050f13f28b0b5ULL, 0xb92dba42d84dbc13ULL};
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