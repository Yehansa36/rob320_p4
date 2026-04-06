#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Point.hpp"
#include "rix/msg/geometry/Quaternion.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Pose : public Message {
  public:
    geometry::Point position;
    geometry::Quaternion orientation;

    Pose() = default;
    Pose(const Pose &other) = default;
    ~Pose() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(position);
        size += size_message(orientation);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x5e8de7739d32edb5ULL, 0xb8858a25a14cc5a9ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, position);
        serialize_message(dst, offset, orientation);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(position, src, size, offset)) { return false; };
        if (!deserialize_message(orientation, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix