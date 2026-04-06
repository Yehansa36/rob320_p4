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

class JointState : public Message {
  public:
    std::string name;
    double position;
    double velocity;
    double effort;

    JointState() = default;
    JointState(const JointState &other) = default;
    ~JointState() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(name);
        size += size_number(position);
        size += size_number(velocity);
        size += size_number(effort);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x431aef87adbd38f1ULL, 0x85684cf3334910d0ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, name);
        serialize_number(dst, offset, position);
        serialize_number(dst, offset, velocity);
        serialize_number(dst, offset, effort);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(name, src, size, offset)) { return false; };
        if (!deserialize_number(position, src, size, offset)) { return false; };
        if (!deserialize_number(velocity, src, size, offset)) { return false; };
        if (!deserialize_number(effort, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix