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

class Endpoint : public Message {
  public:
    uint16_t port;
    std::string address;

    Endpoint() = default;
    Endpoint(const Endpoint &other) = default;
    ~Endpoint() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(port);
        size += size_string(address);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x90624384b177200dULL, 0xe9ef37ae73ed079fULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, port);
        serialize_string(dst, offset, address);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(port, src, size, offset)) { return false; };
        if (!deserialize_string(address, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix