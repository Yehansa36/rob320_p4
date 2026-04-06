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

class Status : public Message {
  public:
    uint64_t id;
    uint8_t error;

    Status() = default;
    Status(const Status &other) = default;
    ~Status() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(id);
        size += size_number(error);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x7fb8fd2b37424f48ULL, 0xab387927011d828bULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, id);
        serialize_number(dst, offset, error);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(id, src, size, offset)) { return false; };
        if (!deserialize_number(error, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix