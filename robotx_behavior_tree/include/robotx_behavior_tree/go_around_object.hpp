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
#include <iostream>
#include <memory>
#include <optional>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hermite_path_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_msgs/msg/task_object.hpp"
#include "robotx_behavior_tree/action_node.hpp"

namespace robotx_behavior_tree
{
class GoAroundObject : public ActionROS2Node
{
public:
  GoAroundObject(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    declare_parameter("goal_tolerance", 0.5);
    get_parameter("goal_tolerance", goal_tolerance_);
    declare_parameter("bouy_distance", 5.9);
    get_parameter("bouy_distance", bouy_distance_);
    declare_parameter("split_angle_deg_", 90.0);
    get_parameter("split_angle_deg_", split_angle_deg_);
    declare_parameter("abs_angle_threshold_deg", 90.0);
    get_parameter("abs_angle_threshold_deg", abs_angle_threshold_deg_);
    goal_pub_front_pose_of_object_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
    start_complete_pub_ =
      this->create_publisher<std_msgs::msg::String>("/go_around_object/started", 1);
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(),
      {BT::InputPort<std::string>("object_type"), BT::InputPort<std::string>("turning_direction"),
       BT::InputPort<double>("orbit_angle")});
  }

  double getGoalTolerance() const { return goal_tolerance_; }

private:
  rclcpp::TimerBase::SharedPtr update_position_timer_;
  double goal_tolerance_;
  double bouy_distance_;
  double split_angle_deg_;
  double waypoint_angle_deg_;
  double abs_angle_threshold_deg_;
  double referece_position_rad_ = 0.0;
  double accumulated_movement_rad_ = 0.0;
  bool is_first_waypoint_ = true;
  bool is_first_reference_position_ = true;
  bool is_final_waypoint_ = false;
  TurningDirection turning_direction_ = TurningDirection::COUNTER_CLOCKWISE;
  geometry_msgs::msg::PoseStamped target_pose_;
  std::vector<robotx_behavior_msgs::msg::TaskObject> target_objects_array_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_front_pose_of_object_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_complete_pub_;
  std::optional<std::shared_ptr<geometry_msgs::msg::PoseStamped_<std::allocator<void> > > >
    current_pose_ = getCurrentPose();

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

  enum class BehaviorState : bool { FIRST, SUBSEQUENT };

  double getDeltaTurningAngle();

  void publishWaypointPose(const std::optional<geometry_msgs::msg::Pose> & waypoint_pose);

  std::optional<std::monostate> updateTurningDirection();

  void updateWaypointAngleDeg(const BehaviorState BehaviorState);

  std::optional<std::monostate> updateTargetObjectsArray();

protected:
  BT::NodeStatus publishTargetPose(const BehaviorState BehaviorState);

  BT::NodeStatus onStart() override { return publishTargetPose(BehaviorState::FIRST); }

  BT::NodeStatus onRunning() override { return publishTargetPose(BehaviorState::SUBSEQUENT); }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, GoAroundObject)
