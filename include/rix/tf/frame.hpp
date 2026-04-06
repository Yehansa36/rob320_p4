#pragma once

#include <string>
#include <vector>

#include "rix/robot/msg_util.hpp"
#include "rix/tf/transform_buffer.hpp"

namespace rix {
namespace tf {

struct Frame {
    Frame();
    Frame(const std::string &name, const TransformBuffer &buffer);
    Frame(const std::string &name, const rix::util::Duration &duration);
    Frame(const Frame &other);
    Frame &operator=(const Frame &other);

    TransformBuffer buffer;
    std::string name;
};

}  // namespace tf
}  // namespace rix