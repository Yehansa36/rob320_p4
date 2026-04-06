#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/sensor/JointState.hpp"
#include "rix/msg/standard/Time.hpp"

namespace rix {
namespace msg {
namespace sensor {

class JS : public Message {
  public:
    standard::Time stamp;
    std::vector<sensor::JointState> joint_states;

    JS() = default;
    JS(const JS &other) = default;
    ~JS() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message(stamp);
        size += size_message_vector(joint_states);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x77f3eba99ee8ea7aULL, 0x3696f4f8df4b9ee0ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message(dst, offset, stamp);
        serialize_message_vector(dst, offset, joint_states);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message(stamp, src, size, offset)) { return false; };
        if (!deserialize_message_vector(joint_states, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix