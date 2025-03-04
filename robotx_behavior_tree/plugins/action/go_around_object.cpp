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
  }
  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(),
      {BT::InputPort<std::string>("object_type"), BT::InputPort<std::string>("turning_direction"),
       BT::InputPort<double>("orbit_angle")});
  }

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

  double getDeltaTurningAngle()
  {
    double delta_turning_angle_rad = 0.0;
    if (is_first_waypoint_) {
      delta_turning_angle_rad = 0.0;
    } else {
      if (is_first_reference_position_) {
        referece_position_rad_ = getRelativeAngle(
          target_objects_array_[0].x, target_objects_array_[0].y,
          current_pose_.value()->pose.position.x, current_pose_.value()->pose.position.y, 0.1);
        is_first_reference_position_ = false;
      }
      auto current_position_rad = getRelativeAngle(
        target_objects_array_[0].x, target_objects_array_[0].y,
        current_pose_.value()->pose.position.x, current_pose_.value()->pose.position.y, 0.1);
      auto interim_delta_turning_angle_rad = current_position_rad - referece_position_rad_;
      if (abs(interim_delta_turning_angle_rad) < M_PI) {
        delta_turning_angle_rad = interim_delta_turning_angle_rad;
      } else {
        delta_turning_angle_rad = interim_delta_turning_angle_rad -
                                  copysign(1.0, interim_delta_turning_angle_rad) * 2 * M_PI;
      }
      RCLCPP_INFO(get_logger(), "target_objects_array_[0].x: %f", target_objects_array_[0].x);
      RCLCPP_INFO(get_logger(), "target_objects_array_[0].y: %f", target_objects_array_[0].y);
      RCLCPP_INFO(
        get_logger(), "current_pose_.value()->pose.position.x: %f",
        current_pose_.value()->pose.position.x);
      RCLCPP_INFO(
        get_logger(), "current_pose_.value()->pose.position.y: %f",
        current_pose_.value()->pose.position.y);
      RCLCPP_INFO(get_logger(), "current_position_rad: %f", current_position_rad);
      RCLCPP_INFO(get_logger(), "referece_position_rad_: %f", referece_position_rad_);
    }
    return delta_turning_angle_rad;
  }

  void publish_waypoint_pose(const std::optional<geometry_msgs::msg::Pose> & waypoint_pose)
  {
    if (waypoint_pose) {
      target_pose_.header.frame_id = "map";
      target_pose_.pose.position.x = waypoint_pose.value().position.x;
      target_pose_.pose.position.y = waypoint_pose.value().position.y;
      target_pose_.pose.position.z = waypoint_pose.value().position.z;
      target_pose_.pose.orientation.w = waypoint_pose.value().orientation.w;
      target_pose_.pose.orientation.x = waypoint_pose.value().orientation.x;
      target_pose_.pose.orientation.y = waypoint_pose.value().orientation.y;
      target_pose_.pose.orientation.z = waypoint_pose.value().orientation.z;
    }
    target_pose_.header.stamp = get_clock()->now();
    auto target_waypoint_distance =
      getDistance(current_pose_.value()->pose.position, target_pose_.pose.position);
    if (target_waypoint_distance > goal_tolerance_) {
      goal_pub_front_pose_of_object_->publish(target_pose_);
    }
  }

  std::optional<std::monostate> updateTurningDirection()
  {
    const auto interim_turning_direction = this->getInput<std::string>("turning_direction");
    if (!interim_turning_direction) {
      return std::nullopt;
    } else {
      if (interim_turning_direction.value() == "clockwise") {
        turning_direction_ = TurningDirection::CLOCKWISE;
      } else if (interim_turning_direction.value() == "counter_clockwise") {
        turning_direction_ = TurningDirection::COUNTER_CLOCKWISE;
      } else {
        return std::nullopt;
      }
    }
    return std::monostate();
  }

  void updateWaypointAngleDeg(const BehaviorState BehaviorState)
  {
    const auto abs_angle_threshold_rad = abs_angle_threshold_deg_ * M_PI / 180.0;
    auto delta_turning_angle_rad = getDeltaTurningAngle();
    if (!is_first_reference_position_ && abs(delta_turning_angle_rad) > abs_angle_threshold_rad) {
      accumulated_movement_rad_ += delta_turning_angle_rad;
      referece_position_rad_ = getRelativeAngle(
        target_objects_array_[0].x, target_objects_array_[0].y,
        current_pose_.value()->pose.position.x, current_pose_.value()->pose.position.y, 0.1);
      delta_turning_angle_rad = getDeltaTurningAngle();
    }
    auto total_turning_angle_rad = accumulated_movement_rad_ + delta_turning_angle_rad;
    auto total_turning_angle_deg = 180.0 * total_turning_angle_rad / M_PI;
    const auto target_orbit_angle = this->getInput<double>("orbit_angle").value();
    is_final_waypoint_ =
      !(BehaviorState == BehaviorState::FIRST) &&
      abs(total_turning_angle_deg) > abs(target_orbit_angle) - split_angle_deg_;
    if (is_final_waypoint_) {
      waypoint_angle_deg_ = abs(target_orbit_angle) - abs(total_turning_angle_deg);
    } else {
      waypoint_angle_deg_ = split_angle_deg_;
    }
    RCLCPP_INFO(get_logger(), "abs_angle_threshold_deg: %f", abs_angle_threshold_deg_);
    RCLCPP_INFO(get_logger(), "delta_turning_angle_deg: %f", 180.0 * delta_turning_angle_rad / M_PI);
    RCLCPP_INFO(get_logger(), "total_turning_angle_deg: %f", total_turning_angle_deg);
    RCLCPP_INFO(get_logger(), "waypoint_angle_deg_: %f", waypoint_angle_deg_);
  }

  std::optional<std::monostate> updateTargetObjectsArray()
  {
    const auto object_type = this->getInput<std::string>("object_type");
    auto target_bouy = static_cast<short>(Buoy::BUOY_RED);
    if (object_type) {
      RCLCPP_INFO(get_logger(), "object_type: %s", object_type.value().c_str());
      if (object_type.value() == "red_bouy") {
        target_bouy = static_cast<short>(Buoy::BUOY_RED);
      } else if (object_type.value() == "green_bouy") {
        target_bouy = static_cast<short>(Buoy::BUOY_GREEN);
      } else if (object_type.value() == "white_bouy") {
        target_bouy = static_cast<short>(Buoy::BUOY_WHITE);
      } else if (object_type.value() == "black_bouy") {
        target_bouy = static_cast<short>(Buoy::BUOY_BLACK);
      } else {
        return std::nullopt;
      }
    }
    const auto task_objects_array = getTaskObjects();
    if (task_objects_array) {
      target_objects_array_ = filter(task_objects_array.value(), target_bouy);
    } else {
      RCLCPP_INFO(get_logger(), "Not Found task_objects");
      return std::nullopt;
    }
    sortBy2DDistance(target_objects_array_, current_pose_.value()->pose.position);
    if (target_objects_array_.empty()) {
      return std::nullopt;
    }
    return std::monostate();
  }

protected:
  BT::NodeStatus publish_target_pose(const BehaviorState BehaviorState)
  {
    if (BehaviorState == BehaviorState::FIRST) {
      is_first_waypoint_ = true;
      is_first_reference_position_ = true;
      accumulated_movement_rad_ = 0.0;
    }
    current_pose_ = getCurrentPose();
    if (!current_pose_) {
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "WAITING_FOR_GOAL: 0 / MOVING_TO_GOAL: 1 / AVOIDING: 2");
    RCLCPP_INFO(get_logger(), "Planner Status: %d", getPlannerStatus().value()->status);

    if (!updateTurningDirection().has_value()) {
      return BT::NodeStatus::FAILURE;
    }
    if (!updateTargetObjectsArray().has_value()) {
      return BT::NodeStatus::FAILURE;
    }
    updateWaypointAngleDeg(BehaviorState);

    auto waypoint_angle_tolerance_deg = 10.0;
    if (is_final_waypoint_ && (waypoint_angle_deg_ < waypoint_angle_tolerance_deg)) {
      return BT::NodeStatus::SUCCESS;
    }

    const auto target_waypoint_distance =
      getDistance(current_pose_.value()->pose.position, target_pose_.pose.position);
    auto waypoint_pose = getTurningWaypointPoseOfObject(
      target_objects_array_[0], bouy_distance_, turning_direction_, waypoint_angle_deg_);
    if (BehaviorState == BehaviorState::FIRST) {
      publish_waypoint_pose(waypoint_pose);
      RCLCPP_INFO(get_logger(), "published waypoint pose");
    }else if (target_waypoint_distance < goal_tolerance_) {
      publish_waypoint_pose(waypoint_pose);
      RCLCPP_INFO(get_logger(), "published waypoint pose");
      is_first_waypoint_ = false;
    }
    RCLCPP_INFO(get_logger(), "is_first_waypoint_: %d", is_first_waypoint_);
    RCLCPP_INFO(get_logger(), "target_waypoint_distance: %f", target_waypoint_distance);
    RCLCPP_INFO(get_logger(), "goal_tolerance_: %f", goal_tolerance_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onStart() override { return publish_target_pose(BehaviorState::FIRST); }

  BT::NodeStatus onRunning() override { return publish_target_pose(BehaviorState::SUBSEQUENT); }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, GoAroundObject)
