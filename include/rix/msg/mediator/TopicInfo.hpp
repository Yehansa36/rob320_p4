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
namespace mediator {

class TopicInfo : public Message {
  public:
    std::string name;
    std::array<uint64_t, 2> message_hash;

    TopicInfo() = default;
    TopicInfo(const TopicInfo &other) = default;
    ~TopicInfo() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(name);
        size += size_number_array(message_hash);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x8e64137b16fde8adULL, 0xe27995f7c4f32229ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, name);
        serialize_number_array(dst, offset, message_hash);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(name, src, size, offset)) { return false; };
        if (!deserialize_number_array(message_hash, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix