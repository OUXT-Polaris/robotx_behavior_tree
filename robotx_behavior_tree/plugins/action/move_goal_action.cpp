// Copyright (c) 2021, OUXT-Polaris
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

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_tree/action_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace robotx_behavior_tree
{
class MoveGoalAction : public ActionROS2Node
{
public:
  MoveGoalAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config), buffer_(get_clock()), listener_(buffer_)
  {
    declare_parameter("goal_tolerance", "0.5");
    get_parameter("goal_tolerance", goal_tolerance_);
    declare_parameter("goal_angle_tolerance", 0.05);
    get_parameter("goal_angle_tolerance", goal_angle_tolerance_);
    goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }

  const std::optional<geometry_msgs::msg::Pose> getCurrentPose()
  {
    try {
      auto transform_stamped =
        buffer_.lookupTransform("map", "base_link", rclcpp::Time(0), tf2::durationFromSec(1.0));
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform_stamped.transform.translation.x;
      pose.position.y = transform_stamped.transform.translation.y;
      pose.position.z = transform_stamped.transform.translation.z;
      pose.orientation = transform_stamped.transform.rotation;
      return pose;
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      return std::nullopt;
    }
    return std::nullopt;
  }

protected:
  BT::NodeStatus tick() override
  {
    auto goal = this->getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!has_goal_published) {
      if (goal) {
        goal_pub_->publish(goal.value());
        RCLCPP_INFO(
          get_logger(), "MoveGoalAction : published goal [%f, %f]", goal.value().pose.position.x,
          goal.value().pose.position.y);
      } else {
        RCLCPP_WARN(get_logger(), "MoveGoalAction : Faild to pushish goal");
      }
      has_goal_published = true;
    }

    auto pose = getCurrentPose();
    get_parameter("goal_tolerance", goal_tolerance_);
    if (pose) {
      double dist = getDistance(pose.value(), goal->pose);
      double angle_dist = getAngleDiff(pose.value(), goal->pose);
      if (dist < goal_tolerance_ && angle_dist < goal_angle_tolerance_) {
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  double getDistance(const geometry_msgs::msg::Pose pose1, const geometry_msgs::msg::Pose pose2)
  {
    auto dx = pose1.position.x - pose2.position.x;
    auto dy = pose1.position.y - pose2.position.y;
    auto dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double getAngleDiff(const geometry_msgs::msg::Pose pose1, const geometry_msgs::msg::Pose pose2)
  {
    auto transform1 = convertToTF2(pose1);
    auto transform2 = convertToTF2(pose2);
    auto diff = transform2.inverse() * transform1;
    return diff.getRotation().getAngle();
  }

  tf2::Transform convertToTF2(const geometry_msgs::msg::Pose pose)
  {
    geometry_msgs::msg::Transform transform_msg;
    transform_msg.translation.x = pose.position.x;
    transform_msg.translation.y = pose.position.y;
    transform_msg.translation.z = pose.position.z;
    transform_msg.rotation = pose.orientation;

    tf2::Transform transform;
    tf2::convert(transform_msg, transform);
    return transform;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  bool has_goal_published = false;
  double goal_tolerance_;
  double goal_angle_tolerance_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveGoalAction)
