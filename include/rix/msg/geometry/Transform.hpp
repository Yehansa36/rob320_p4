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
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Transform : public Message {
  public:
    geometry::Vector3 translation;
    geometry::Quaternion rotation;

    Transform() = default;
    Transform(const Transform &other) = default;
    ~Transform() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(translation);
        size += size_message(rotation);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x7c6b915aa032ce4aULL, 0xcb20e26fd99a8832ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, translation);
        serialize_message(dst, offset, rotation);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(translation, src, size, offset)) { return false; };
        if (!deserialize_message(rotation, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix