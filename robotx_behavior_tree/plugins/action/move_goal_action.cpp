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

#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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
  : ActionROS2Node(name, config)
  {
    declare_parameter("goal_tolerance", 1.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    declare_parameter("goal_angle_tolerance", 0.08);
    get_parameter("goal_angle_tolerance", goal_angle_tolerance_);
    goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(),
      {BT::InputPort<double>("goal_x"), BT::InputPort<double>("goal_y"),
       BT::InputPort<double>("goal_theta")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    auto goal_x = this->getInput<double>("goal_x");
    auto goal_y = this->getInput<double>("goal_y");
    auto goal_theta = this->getInput<double>("goal_theta");
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.0, 0.0, goal_theta.value());

    if (goal_x && goal_y && goal_theta) {
      goal.header.frame_id = "map";
      goal.pose.position.x = goal_x.value();
      goal.pose.position.y = goal_y.value();
      goal.pose.position.z = 0.0;

      goal.pose.orientation.w = tf_quat.w();
      goal.pose.orientation.x = tf_quat.x();
      goal.pose.orientation.y = tf_quat.y();
      goal.pose.orientation.z = tf_quat.z();

      goal_pub_->publish(goal);
      RCLCPP_INFO(
        get_logger(), "MoveGoalAction : published goal [%f, %f, %f]", goal_x.value(),
        goal_y.value(), goal_theta.value());
      return BT::NodeStatus::RUNNING;
    } else {
      RCLCPP_WARN(get_logger(), "MoveGoalAction : Faild to pushish goal");
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onRunning() override
  {
    auto pose = getCurrentPose();
    get_parameter("goal_tolerance", goal_tolerance_);
    if (pose) {
      dist = getDistance(pose.value()->pose, goal.pose);
      angle_dist = getAngleDiff(pose.value()->pose, goal.pose);
    }
    if (dist < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "MoveGoalAction : SUCCESS");
      return BT::NodeStatus::SUCCESS;
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
  double goal_tolerance_;
  double goal_angle_tolerance_;
  double dist;
  double angle_dist;
  geometry_msgs::msg::PoseStamped goal;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveGoalAction)
