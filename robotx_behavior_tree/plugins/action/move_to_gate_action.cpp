// Copyright (c) 2022, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hermite_path_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_msgs/msg/task_object.hpp"
#include "robotx_behavior_tree/action_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace robotx_behavior_tree
{
class MoveToGateAction : public ActionROS2Node
{
public:
  MoveToGateAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config), buffer_(get_clock()), listener_(buffer_)
  {
    declare_parameter("goal_tolerance", 1.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_pub_gate_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

private:
  rclcpp::TimerBase::SharedPtr update_position_timer_;
  float distance_;
  double goal_tolerance_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_gate_;
  geometry_msgs::msg::PoseStamped goal_;

  std::vector<robotx_behavior_msgs::msg::TaskObject> red_buoys_array_;
  std::vector<robotx_behavior_msgs::msg::TaskObject> green_buoys_array_;

  enum class Buoy_ : short { BUOY_RED = 1, BUOY_GREEN = 2, BUOY_WHITE = 3, BUOY_BLACK = 4 };
  enum class Status_ : short { WAITING_FOR_GOAL, MOVING_TO_GOAL, AVOIDING };

protected:
  BT::NodeStatus onStart() override
  {
    const auto status_planner = getPlannerStatus();
    // try {
    //   const auto task_objects_array = getTaskObjects();
    //   if (task_objects_array) {
    //     return BT::NodeStatus::RUNNING;
    //   }
    //   return BT::NodeStatus::FAILURE;

    // } catch (const std::runtime_error & error) {
    //   RCLCPP_WARN_STREAM(get_logger(), error.what());
    //   return BT::NodeStatus::FAILURE;
    // }
    const auto task_objects_array = getTaskObjects();
    if (task_objects_array) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onRunning() override
  {
    const auto status_planner = getPlannerStatus();
    const auto pose = getCurrentPose();
    const auto task_objects_array = getTaskObjects();

    if (task_objects_array) {
      red_buoys_array_ = filter(task_objects_array.value(), static_cast<short>(Buoy_::BUOY_RED));
      green_buoys_array_ =
        filter(task_objects_array.value(), static_cast<short>(Buoy_::BUOY_GREEN));
    }

    sortBy2DDistance(red_buoys_array_, pose.value()->pose.position);
    sortBy2DDistance(green_buoys_array_, pose.value()->pose.position);

    const auto xyz = between(red_buoys_array_[0], green_buoys_array_[0], pose.value()->pose);
    get_parameter("goal_tolerance", goal_tolerance_);

    goal_.header.frame_id = "map";
    goal_.pose.position.x = xyz.position.x;
    goal_.pose.position.y = xyz.position.y;
    goal_.pose.position.z = xyz.position.z;

    goal_.pose.orientation.w = xyz.orientation.w;
    goal_.pose.orientation.x = xyz.orientation.x;
    goal_.pose.orientation.y = xyz.orientation.y;
    goal_.pose.orientation.z = xyz.orientation.z;

    if (pose) {
      distance_ = getDistance(pose.value()->pose.position, goal_.pose.position);
    }

    if (distance_ < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }

    goal_.header.stamp = get_clock()->now();
    if (status_planner.value()->status != static_cast<short>(Status_::MOVING_TO_GOAL)) {
      goal_pub_gate_->publish(goal_);
    }

    return BT::NodeStatus::RUNNING;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToGateAction)