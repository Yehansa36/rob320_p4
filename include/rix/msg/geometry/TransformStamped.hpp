#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class TransformStamped : public Message {
  public:
    standard::Header header;
    std::string child_frame_id;
    geometry::Transform transform;

    TransformStamped() = default;
    TransformStamped(const TransformStamped &other) = default;
    ~TransformStamped() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_string(child_frame_id);
        size += size_message(transform);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x5e276c2aba550001ULL, 0xbeb45a3d7e3c5100ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_string(dst, offset, child_frame_id);
        serialize_message(dst, offset, transform);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_string(child_frame_id, src, size, offset)) { return false; };
        if (!deserialize_message(transform, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix