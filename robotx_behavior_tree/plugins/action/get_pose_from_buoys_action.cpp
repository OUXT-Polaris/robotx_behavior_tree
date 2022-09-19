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

namespace robotx_behavior_tree
{
class GetPoseFromBuoysAction : public ActionROS2Node
{
public:
  GetPoseFromBuoysAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config), buffer_(get_clock()), listener_(buffer_)
  {
    declare_parameter("goal_tolerance", 1.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_pub_gate_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(),
      {BT::OutputPort<geometry_msgs::msg::Pose>("pose")});
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
    const auto pose = getCurrentPose();
    const auto task_objects_array = getTaskObjects();

    if (task_objects_array) {
      red_buoys_array_ = filter(task_objects_array.value(), static_cast<short>(Buoy_::BUOY_RED));
      green_buoys_array_ =
        filter(task_objects_array.value(), static_cast<short>(Buoy_::BUOY_GREEN));

        sortBy2DDistance(red_buoys_array_, pose.value()->pose.position);
        sortBy2DDistance(green_buoys_array_, pose.value()->pose.position);

        const auto xyz = between(red_buoys_array_[0], green_buoys_array_[0], pose.value()->pose);

        setOutput("pose", xyz);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, GetPoseFromBuoysAction)