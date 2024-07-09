// Copyright (c) 2024, OUXT-Polaris
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

namespace robotx_behavior_tree
{
class MoveToFrontPoseOfObject : public ActionROS2Node
{
public:
  MoveToFrontPoseOfObject(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    declare_parameter("goal_tolerance", 1.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_pub_front_pose_of_object_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

private:
  rclcpp::TimerBase::SharedPtr update_position_timer_;
  float distance_;
  double goal_tolerance_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_front_pose_of_object_;
  std::vector<robotx_behavior_msgs::msg::TaskObject> target_objects_array_;

  enum class Buoy : short {
    BUOY_RED = robotx_behavior_msgs::msg::TaskObject::BUOY_RED,
    BUOY_GREEN = robotx_behavior_msgs::msg::TaskObject::BUOY_GREEN,
    BUOY_WHITE = robotx_behavior_msgs::msg::TaskObject::BUOY_WHITE,
    BUOY_BLACK = robotx_behavior_msgs::msg::TaskObject::BUOY_BLACK
  };
  enum class Status : short {
    WAITING_FOR_GOAL = hermite_path_msgs::msg::PlannerStatus::WAITING_FOR_GOAL,
    MOVING_TO_GOAL = hermite_path_msgs::msg::PlannerStatus::MOVING_TO_GOAL,
    AVOIDING = hermite_path_msgs::msg::PlannerStatus::MOVING_TO_GOAL
  };

protected:
  BT::NodeStatus onStart() override
  {
    const auto status_planner = getPlannerStatus();
    const auto task_objects_array = getTaskObjects();
    if (task_objects_array) {
      RCLCPP_INFO(get_logger(), "get task_objects");
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
      target_objects_array_ = filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_RED));
    }

    sortBy2DDistance(target_objects_array_, pose.value()->pose.position);
    if (target_objects_array_.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    const auto xyz = getFrontPoseOfObject(target_objects_array_[0], 5.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_.header.frame_id = "map";
    if (xyz) {
      goal_.pose.position.x = xyz.value().position.x;
      goal_.pose.position.y = xyz.value().position.y;
      goal_.pose.position.z = xyz.value().position.z;
      goal_.pose.orientation.w = xyz.value().orientation.w;
      goal_.pose.orientation.x = xyz.value().orientation.x;
      goal_.pose.orientation.y = xyz.value().orientation.y;
      goal_.pose.orientation.z = xyz.value().orientation.z;
    }
    goal_.header.stamp = get_clock()->now();
    if (status_planner.value()->status == static_cast<short>(Status::WAITING_FOR_GOAL)) {
      goal_pub_front_pose_of_object_->publish(goal_);
    }
    distance_ = getDistance(pose.value()->pose.position, goal_.pose.position);

    if (distance_ < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToFrontPoseOfObject)
