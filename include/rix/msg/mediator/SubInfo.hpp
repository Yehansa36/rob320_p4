#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/mediator/Endpoint.hpp"
#include "rix/msg/mediator/TopicInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SubInfo : public Message {
  public:
    uint64_t id;
    uint64_t node_id;
    uint8_t protocol;
    mediator::TopicInfo topic_info;
    mediator::Endpoint endpoint;

    SubInfo() = default;
    SubInfo(const SubInfo &other) = default;
    ~SubInfo() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(id);
        size += size_number(node_id);
        size += size_number(protocol);
        size += size_message(topic_info);
        size += size_message(endpoint);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x6e39a9407d6ad75bULL, 0xaadb5b7b4a7af6feULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, id);
        serialize_number(dst, offset, node_id);
        serialize_number(dst, offset, protocol);
        serialize_message(dst, offset, topic_info);
        serialize_message(dst, offset, endpoint);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(id, src, size, offset)) { return false; };
        if (!deserialize_number(node_id, src, size, offset)) { return false; };
        if (!deserialize_number(protocol, src, size, offset)) { return false; };
        if (!deserialize_message(topic_info, src, size, offset)) { return false; };
        if (!deserialize_message(endpoint, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix