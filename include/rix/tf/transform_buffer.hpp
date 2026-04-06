#pragma once

#include <deque>

#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/standard/Time.hpp"

#include "rix/util/time.hpp"

namespace rix {
namespace tf {

class TransformBuffer {
   public:
    TransformBuffer();
    TransformBuffer(const rix::util::Duration &duration);
    TransformBuffer(const TransformBuffer &other);
    TransformBuffer &operator=(const TransformBuffer &other);

    size_t size() const;
    bool empty() const;
    void clear();

    void insert(const rix::util::Time &time, const rix::msg::geometry::Transform &transform);
    bool get(const rix::util::Time &time, rix::msg::geometry::Transform &transform) const;

    const std::deque<std::pair<rix::util::Time, rix::msg::geometry::Transform>> &data() const;

    rix::util::Duration duration() const;
    void set_duration(const rix::util::Duration &duration);

   private:
    rix::util::Duration duration_;
    std::deque<std::pair<rix::util::Time, rix::msg::geometry::Transform>> buffer_;
};

}  // namespace tf
}  // namespace rix