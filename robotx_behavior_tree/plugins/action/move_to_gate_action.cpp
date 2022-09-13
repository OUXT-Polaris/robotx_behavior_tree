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
  float red_buoy_x = 0.0;
  float red_buoy_y = 0.0;
  float green_buoy_x = 0.0;
  float green_buoy_y = 0.0;
  float goal_x;
  float goal_y;
  float goal_theta;

  float distance_;
  double goal_tolerance_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_gate_;
  geometry_msgs::msg::PoseStamped goal_;

protected:
  BT::NodeStatus onStart() override
  {
    const auto status_planner = getPlannerStatus();

    if (status_planner) {
      RCLCPP_INFO(get_logger(), "status : %d", status_planner.value()->status);
    }

    try {
      const auto task_objects_array = getTaskObjects();
      if (task_objects_array) {
        RCLCPP_INFO(
          get_logger(), "task object size : %ld", task_objects_array->task_objects.size());
      }

      // return BT::NodeStatus::FAILURE;
      goal_.header.frame_id = "map";
      goal_.pose.position.x = goal_x;
      goal_.pose.position.y = goal_y;
      goal_.pose.position.z = 0.0;

      goal_.pose.orientation.w = 1.0;
      goal_.pose.orientation.x = 0.0;
      goal_.pose.orientation.y = 0.0;
      goal_.pose.orientation.z = 0.0;

      return BT::NodeStatus::RUNNING;

    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_STREAM(get_logger(), error.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  BT::NodeStatus onRunning() override
  {
    const auto status_planner = getPlannerStatus();
    const auto pose = getCurrentPose();
    get_parameter("goal_tolerance", goal_tolerance_);

    if (pose) {
      distance_ = getDistance(pose.value(), goal_.pose);
    }

    if (distance_ < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(get_logger(), "distance from goal: %f", distance_);

    goal_.header.stamp = get_clock()->now();
    if (status_planner.value()->status != 1) {
      goal_pub_gate_->publish(goal_);
      RCLCPP_INFO(get_logger(), "status1 : %d", status_planner.value()->status);
    }

    RCLCPP_INFO(get_logger(), "status : %d", status_planner.value()->status);

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
      RCLCPP_INFO(get_logger(), "position_x : [%f]", pose.position.x);
      RCLCPP_INFO(get_logger(), "position_y : [%f]", pose.position.y);
      RCLCPP_INFO(get_logger(), "position_z : [%f]", pose.position.z);
      return pose;
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      return std::nullopt;
    }
    return std::nullopt;
  }

  float calculateDistance(float x, float y)
  {
    auto pose = getCurrentPose();
    auto dx = x - static_cast<float>(pose->position.x);
    auto dy = y - static_cast<float>(pose->position.y);

    return std::sqrt(dx * dx + dy * dy);
  }

  const std::optional<std::vector<float>> GetGenterGate(robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr task_objects_array)
  {
    std::vector<robotx_behavior_msgs::msg::TaskObject> green_buoys_array;
    std::vector<robotx_behavior_msgs::msg::TaskObject> red_buoys_array;

    std::vector<float> goal_points;

    for (size_t i = 0; i < task_objects_array->task_objects.size(); i++) {
        if (task_objects_array->task_objects[i].object_kind == 1) {
          red_buoys_array.push_back(task_objects_array->task_objects[i]);
        }
        if (task_objects_array->task_objects[i].object_kind == 2) {
          green_buoys_array.push_back(task_objects_array->task_objects[i]);
        }
      }

      RCLCPP_INFO(get_logger(), "red_buoys_array_size : [%ld]", red_buoys_array.size());
      RCLCPP_INFO(get_logger(), "green_buoys_array_size : [%ld]", green_buoys_array.size());

      for (size_t i = 0; i < red_buoys_array.size(); i++) {
        RCLCPP_INFO(get_logger(), "red_buoys_array_x : [%f]", red_buoys_array[i].x);
        RCLCPP_INFO(get_logger(), "red_buoys_array_y : [%f]", red_buoys_array[i].y);
        //RCLCPP_INFO(get_logger(), "red_buoys_array_z : [%f]", red_buoys_array[i].z);
        distance_ = calculateDistance(red_buoys_array[i].x, red_buoys_array[i].y);

        if (red_buoy_x == 0.0 && i == 0) {
          red_buoy_x = red_buoys_array[i].x;
        }
        if (red_buoy_x > red_buoys_array[i].x) {
          red_buoy_x = red_buoys_array[i].x;
        }

        if (red_buoy_y == 0.0 && i == 0) {
          red_buoy_y = red_buoys_array[i].y;
        }
        if (red_buoy_y > red_buoys_array[i].y) {
          red_buoy_y = red_buoys_array[i].y;
        }
      }

      for (size_t i = 0; i < green_buoys_array.size(); i++) {
        if (green_buoy_x == 0.0 && i == 0) {
          green_buoy_x = green_buoys_array[i].x;
        }
        if (green_buoy_x > green_buoys_array[i].x) {
          green_buoy_x = green_buoys_array[i].x;
        }

        if (green_buoy_y == 0.0 && i == 0) {
          green_buoy_y = green_buoys_array[i].y;
        }
        if (green_buoy_y > green_buoys_array[i].y) {
          green_buoy_y = green_buoys_array[i].y;
        }
      }

      goal_x = (red_buoy_x + green_buoy_x) / 2.0;
      goal_y = (red_buoy_y + green_buoy_y) / 2.0;
      goal_theta = 0.0;

      goal_points = {goal_x, goal_y, goal_theta};

      return goal_points;
    
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToGateAction)