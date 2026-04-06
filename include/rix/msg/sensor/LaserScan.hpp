#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace sensor {

class LaserScan : public Message {
  public:
    standard::Header header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;

    LaserScan() = default;
    LaserScan(const LaserScan &other) = default;
    ~LaserScan() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(header);
        size += size_number(angle_min);
        size += size_number(angle_max);
        size += size_number(angle_increment);
        size += size_number(time_increment);
        size += size_number(scan_time);
        size += size_number(range_min);
        size += size_number(range_max);
        size += size_number_vector(ranges);
        size += size_number_vector(intensities);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xc035b0c601092d80ULL, 0x3a3fb222f8dbb35bULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, header);
        serialize_number(dst, offset, angle_min);
        serialize_number(dst, offset, angle_max);
        serialize_number(dst, offset, angle_increment);
        serialize_number(dst, offset, time_increment);
        serialize_number(dst, offset, scan_time);
        serialize_number(dst, offset, range_min);
        serialize_number(dst, offset, range_max);
        serialize_number_vector(dst, offset, ranges);
        serialize_number_vector(dst, offset, intensities);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(header, src, size, offset)) { return false; };
        if (!deserialize_number(angle_min, src, size, offset)) { return false; };
        if (!deserialize_number(angle_max, src, size, offset)) { return false; };
        if (!deserialize_number(angle_increment, src, size, offset)) { return false; };
        if (!deserialize_number(time_increment, src, size, offset)) { return false; };
        if (!deserialize_number(scan_time, src, size, offset)) { return false; };
        if (!deserialize_number(range_min, src, size, offset)) { return false; };
        if (!deserialize_number(range_max, src, size, offset)) { return false; };
        if (!deserialize_number_vector(ranges, src, size, offset)) { return false; };
        if (!deserialize_number_vector(intensities, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix