#include "rix/tf/transform_broadcaster.hpp"

namespace rix {
namespace tf {

/**< TODO: Implement the constructor */
TransformBroadcaster::TransformBroadcaster(rix::core::Node &node, const std::string &topic,
                                           const rix::ipc::Endpoint &endpoint)
    : publisher_(node.create_publisher<rix::msg::geometry::TF>(topic, endpoint)) {
    if (!publisher_->ok()) {
        rix::util::Log::error << "TransformBroadcaster: Failed to create publisher" << std::endl;
    }
}

TransformBroadcaster::TransformBroadcaster(const TransformBroadcaster &other) : publisher_(other.publisher_) {}

TransformBroadcaster &TransformBroadcaster::operator=(const TransformBroadcaster &other) {
    if (this != &other) {
        publisher_ = other.publisher_;
    }
    return *this;
}

void TransformBroadcaster::send(const rix::msg::geometry::TransformStamped &transform) {
    rix::msg::geometry::TF tf;
    tf.transforms.push_back(transform);
    publisher_->publish(tf);
}

void TransformBroadcaster::send(const rix::msg::geometry::TF &tf) { publisher_->publish(tf); }

bool TransformBroadcaster::ok() const { return publisher_->ok(); }

}  // namespace tf
}  // namespace rix