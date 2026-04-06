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
namespace standard {

class StringArray : public Message {
  public:
    std::vector<std::string> data;

    StringArray() = default;
    StringArray(const StringArray &other) = default;
    ~StringArray() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string_vector(data);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xf4562dc36be2b181ULL, 0x5db9d16dfcb0f9d1ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string_vector(dst, offset, data);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string_vector(data, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace standard
} // namespace msg
} // namespace rix