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
#include "rix/msg/sensor/ChannelFloat.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace sensor {

class PointCloud : public Message {
  public:
    standard::Header header;
    std::vector<geometry::Point> points;
    std::vector<sensor::ChannelFloat> channels;

    PointCloud() = default;
    PointCloud(const PointCloud &other) = default;
    ~PointCloud() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_message_vector(points);
        size += size_message_vector(channels);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x8bca33deedbad17dULL, 0xc1e284d18537d76aULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_message_vector(dst, offset, points);
        serialize_message_vector(dst, offset, channels);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_message_vector(points, src, size, offset)) { return false; };
        if (!deserialize_message_vector(channels, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix