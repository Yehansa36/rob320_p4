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

namespace rix {
namespace msg {
namespace mediator {

class NodeInfo : public Message {
  public:
    std::string name;
    uint64_t id;
    uint64_t machine_id;
    uint8_t protocol;
    mediator::Endpoint endpoint;

    NodeInfo() = default;
    NodeInfo(const NodeInfo &other) = default;
    ~NodeInfo() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(name);
        size += size_number(id);
        size += size_number(machine_id);
        size += size_number(protocol);
        size += size_message(endpoint);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x4fade8c3f2179b5eULL, 0x9d84b6c6e3719a8aULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, name);
        serialize_number(dst, offset, id);
        serialize_number(dst, offset, machine_id);
        serialize_number(dst, offset, protocol);
        serialize_message(dst, offset, endpoint);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(name, src, size, offset)) { return false; };
        if (!deserialize_number(id, src, size, offset)) { return false; };
        if (!deserialize_number(machine_id, src, size, offset)) { return false; };
        if (!deserialize_number(protocol, src, size, offset)) { return false; };
        if (!deserialize_message(endpoint, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix