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
namespace sensor {

class ChannelFloat : public Message {
  public:
    std::string name;
    std::vector<float> data;

    ChannelFloat() = default;
    ChannelFloat(const ChannelFloat &other) = default;
    ~ChannelFloat() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(name);
        size += size_number_vector(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xc97782d7135864feULL, 0x5d5322310654455cULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, name);
        serialize_number_vector(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(name, src, size, offset)) { return false; };
        if (!deserialize_number_vector(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix