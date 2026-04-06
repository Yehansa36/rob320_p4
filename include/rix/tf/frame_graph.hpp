#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"
#include "rix/tf/frame.hpp"
#include "rix/util/log.hpp"
#include "rix/util/time.hpp"

namespace rix {
namespace tf {

// This class is for storing and interpolating frames in a graph structure.
class FrameGraph {
   public:
    // Traverses from a leaf to the root
    class Iterator {
        friend class FrameGraph;

       public:
        Iterator(const Iterator &other);
        Iterator &operator=(const Iterator &other);

        Iterator &operator--();
        Iterator operator--(int);
        bool operator==(const Iterator &other) const;
        bool operator!=(const Iterator &other) const;
        const Frame &operator*() const;
        const Frame *operator->() const;
        bool operator<(const Iterator &other) const;
        bool operator>(const Iterator &other) const;
        bool operator<=(const Iterator &other) const;
        bool operator>=(const Iterator &other) const;

       private:
        Iterator(const FrameGraph &graph, int index);
        const FrameGraph &graph_;
        int index_;
    };

    FrameGraph(const std::string &root, const rix::util::Duration &duration);
    FrameGraph(const FrameGraph &other);
    FrameGraph &operator=(const FrameGraph &other);

    bool exists(const std::string &name) const;
    std::vector<std::string> get_leaves() const;

    Iterator get_root() const;
    Iterator find(const std::string &name) const;
    Iterator find_nearest_ancestor(const std::string &frame_a, const std::string &frame_b) const;
    Iterator find_nearest_ancestor(Iterator frame_a, Iterator frame_b) const;
    Iterator end() const;

    /**
     * @brief Updates the FrameGraph with the transforms in the TF message
     *
     * @param tf The TF message to update the FrameGraph with
     * @return true if all transforms were successfully update, false if at
     * least one transform failed
     */
    bool update(const rix::msg::geometry::TF &tf);

    /**
     * @brief Updates the FrameGraph with the transform
     *
     * @param transform The transform to update the FrameGraph with
     * @return true if the transform was successfully udpated
     */
    bool update(const rix::msg::geometry::TransformStamped &transform);

    bool get_transform(const std::string &target_frame, const std::string &source_frame, rix::util::Time time,
                       rix::msg::geometry::TransformStamped &transform) const;

   private:
    std::vector<std::vector<int>> graph_;
    std::vector<Frame> frames_;
    std::unordered_map<std::string, int> name_to_index_;
    rix::util::Duration duration_;
};

}  // namespace tf
}  // namespace rix