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

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <typeinfo>

#include "geometry_msgs/msg/pose_stamped.hpp"
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
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", rclcpp::QoS(1).reliable().transient_local());
  }

protected:
  BT::NodeStatus onStart() override
  {
    std::vector<robotx_behavior_msgs::msg::TaskObject> green_buoy;
    std::vector<robotx_behavior_msgs::msg::TaskObject> red_buoy;
    
    try {
      const auto task_objects_array = getTaskObjects();
      // return BT::NodeStatus::FAILURE;
      // for (size_t i = 0; i < task_objects_array->task_objects.size(); i++) {
      //   if (task_objects_array->task_objects[i].object_kind == 1) {
      //     green_buoy.push_back(task_objects_array->task_objects[i]);
      //   } 
      //   if (task_objects_array->task_objects[i].object_kind == 2) {
      //     red_buoy.push_back(task_objects_array->task_objects[i]);
      //   }
      // }
     // RCLCPP_INFO(get_logger(), "hogehogehoge : %d", task_objects_array->task_objects.size());

      // buoy1_x = green_buoy[0].x;
      // buoy1_y = green_buoy[0].y;
      // buoy2_x = red_buoy[0].x;
      // buoy2_y = red_buoy[0].y;

      //for debug
      buoy1_x = 10.0;
      buoy1_y = 0.0;
      buoy2_x = 10.0;
      buoy2_y = 10.0;

      

      goal_x = (buoy1_x + buoy2_x) / 2.0;
      goal_y = (buoy1_y + buoy2_y) / 2.0;
      goal_theta = 0.0;

      RCLCPP_INFO(get_logger(), "goal_x : %f", goal_x);
      RCLCPP_INFO(get_logger(), "goal_y : %f", goal_y);

      goal.header.frame_id = "map";
      goal.pose.position.x = goal_x;
      goal.pose.position.y = goal_y;
      goal.pose.position.z = 0.0;

      goal.pose.orientation.w = 0.0;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;

      

      goal_pub_gate_->publish(goal);

      return BT::NodeStatus::RUNNING;

      // if (goal_x && goal_y && goal_theta) {
      //   goal.header.frame_id = "map";
      //   goal.pose.position.x = goal_x;
      //   goal.pose.position.y = goal_y;
      //   goal.pose.position.z = 0.0;

      //   goal.pose.orientation.w = 0.0;
      //   goal.pose.orientation.x = 0.0;
      //   goal.pose.orientation.y = 0.0;
      //   goal.pose.orientation.z = 0.0;

      //   goal_pub_gate_->publish(goal);

      //   RCLCPP_INFO(get_logger(), "Published! goal to through the gate");
      //   return BT::NodeStatus::RUNNING;
      // }else{
      //   return BT::NodeStatus::FAILURE;
      // }

    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_STREAM(get_logger(), error.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  BT::NodeStatus onRunning() override
  {
    // auto pose = getCurrentPose();
    // get_parameter("goal_tolerance", goal_tolerance_);
    // if (pose) {
    //   distance = getDistance(pose.value(), goal.pose);
    // }
    // if (distance < goal_tolerance_) {
    //   RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
    //   return BT::NodeStatus::SUCCESS;
    // }
    // return BT::NodeStatus::RUNNING;
    RCLCPP_INFO(get_logger(), "BBBBBBBBBB");
    //return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }

  double getDistance(const geometry_msgs::msg::Pose pose1, const geometry_msgs::msg::Pose pose2)
  {
    auto dx = pose1.position.x - pose2.position.x;
    auto dy = pose1.position.y - pose2.position.y;
    auto dz = pose1.position.z - pose2.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
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

  float buoy1_x;
  float buoy1_y;
  float buoy2_x;
  float buoy2_y;
  float goal_x;
  float goal_y;
  float goal_theta;

  float distance;
  double goal_tolerance_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_gate_;
  geometry_msgs::msg::PoseStamped goal;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToGateAction)