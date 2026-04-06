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

class Operation : public Message {
  public:
    uint32_t len;
    uint8_t opcode;

    Operation() = default;
    Operation(const Operation &other) = default;
    ~Operation() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(len);
        size += size_number(opcode);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x86699b36c0a476cbULL, 0x3538e0260620e597ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, len);
        serialize_number(dst, offset, opcode);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(len, src, size, offset)) { return false; };
        if (!deserialize_number(opcode, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix