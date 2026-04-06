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

class Pose2D : public Message {
  public:
    float x;
    float y;
    float theta;

    Pose2D() = default;
    Pose2D(const Pose2D &other) = default;
    ~Pose2D() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(x);
        size += size_number(y);
        size += size_number(theta);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xe6921cbc9b9f8376ULL, 0x5ab0f8a78d45b88fULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, x);
        serialize_number(dst, offset, y);
        serialize_number(dst, offset, theta);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(x, src, size, offset)) { return false; };
        if (!deserialize_number(y, src, size, offset)) { return false; };
        if (!deserialize_number(theta, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix