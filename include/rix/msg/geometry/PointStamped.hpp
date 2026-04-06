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
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class PointStamped : public Message {
  public:
    standard::Header header;
    geometry::Point point;

    PointStamped() = default;
    PointStamped(const PointStamped &other) = default;
    ~PointStamped() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_message(point);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x28b887e9284a56c3ULL, 0xd334f1acfc391d26ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_message(dst, offset, point);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_message(point, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix