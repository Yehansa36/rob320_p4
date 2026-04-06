#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Pose2D.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Pose2DStamped : public Message {
  public:
    standard::Header header;
    geometry::Pose2D pose;

    Pose2DStamped() = default;
    Pose2DStamped(const Pose2DStamped &other) = default;
    ~Pose2DStamped() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_message(pose);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xe069dc53bf23a3f1ULL, 0x60fb73a105836636ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_message(dst, offset, pose);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_message(pose, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix