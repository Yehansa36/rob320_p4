#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Quaternion.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class QuaternionStamped : public Message {
  public:
    standard::Header header;
    geometry::Quaternion quaternion;

    QuaternionStamped() = default;
    QuaternionStamped(const QuaternionStamped &other) = default;
    ~QuaternionStamped() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_message(quaternion);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x54fe0096d6e750e6ULL, 0xcbd823cbff521c71ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_message(dst, offset, quaternion);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_message(quaternion, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix