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
namespace geometry {

class Twist2D : public Message {
  public:
    float vx;
    float vy;
    float wz;

    Twist2D() = default;
    Twist2D(const Twist2D &other) = default;
    ~Twist2D() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(vx);
        size += size_number(vy);
        size += size_number(wz);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xd87fbd8e8f8e4037ULL, 0x2cc37c82e4a30bffULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, vx);
        serialize_number(dst, offset, vy);
        serialize_number(dst, offset, wz);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(vx, src, size, offset)) { return false; };
        if (!deserialize_number(vy, src, size, offset)) { return false; };
        if (!deserialize_number(wz, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix