#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Inertia : public Message {
  public:
    double mass;
    geometry::Vector3 center_of_mass;
    double ixx;
    double ixy;
    double ixz;
    double iyy;
    double iyz;
    double izz;

    Inertia() = default;
    Inertia(const Inertia &other) = default;
    ~Inertia() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(mass);
        size += size_message(center_of_mass);
        size += size_number(ixx);
        size += size_number(ixy);
        size += size_number(ixz);
        size += size_number(iyy);
        size += size_number(iyz);
        size += size_number(izz);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xab914168d3ceee44ULL, 0x4f58ffe82b66ee05ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, mass);
        serialize_message(dst, offset, center_of_mass);
        serialize_number(dst, offset, ixx);
        serialize_number(dst, offset, ixy);
        serialize_number(dst, offset, ixz);
        serialize_number(dst, offset, iyy);
        serialize_number(dst, offset, iyz);
        serialize_number(dst, offset, izz);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(mass, src, size, offset)) { return false; };
        if (!deserialize_message(center_of_mass, src, size, offset)) { return false; };
        if (!deserialize_number(ixx, src, size, offset)) { return false; };
        if (!deserialize_number(ixy, src, size, offset)) { return false; };
        if (!deserialize_number(ixz, src, size, offset)) { return false; };
        if (!deserialize_number(iyy, src, size, offset)) { return false; };
        if (!deserialize_number(iyz, src, size, offset)) { return false; };
        if (!deserialize_number(izz, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix