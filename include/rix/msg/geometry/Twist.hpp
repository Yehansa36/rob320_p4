#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Twist : public Message {
  public:
    geometry::Vector3 linear;
    geometry::Vector3 angular;

    Twist() = default;
    Twist(const Twist &other) = default;
    ~Twist() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(linear);
        size += size_message(angular);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x312d32d77af3700aULL, 0xed8db0e780d13e74ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, linear);
        serialize_message(dst, offset, angular);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(linear, src, size, offset)) { return false; };
        if (!deserialize_message(angular, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix