#pragma once

#include "rix/core/node.hpp"
#include "rix/core/publisher.hpp"
#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace tf {

class TransformBroadcaster {
   public:
    TransformBroadcaster(rix::core::Node &node, const std::string &topic = "/tf",
                         const rix::ipc::Endpoint &endpoint = rix::ipc::Endpoint("127.0.0.1", 0));
    TransformBroadcaster(const TransformBroadcaster &other);
    TransformBroadcaster &operator=(const TransformBroadcaster &other);

    void send(const rix::msg::geometry::TransformStamped &transform);
    void send(const rix::msg::geometry::TF &tf);

    bool ok() const;

   private:
    std::shared_ptr<rix::core::Publisher> publisher_;
};

}  // namespace tf
}  // namespace rix