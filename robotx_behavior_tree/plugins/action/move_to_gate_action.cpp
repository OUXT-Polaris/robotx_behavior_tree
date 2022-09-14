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
  float distance_;
  double goal_tolerance_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_gate_;
  geometry_msgs::msg::PoseStamped goal_;

  enum class Buoy : short { BUOY_RED = 1, BUOY_GREEN = 2, BUOY_WHITE = 3, BUOY_BLACK = 4 };

protected:
  BT::NodeStatus onStart() override
  {
    const auto status_planner = getPlannerStatus();
    try {
      const auto task_objects_array = getTaskObjects();
      if (task_objects_array) {
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::FAILURE;

    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_STREAM(get_logger(), error.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onRunning() override
  {
    const auto status_planner = getPlannerStatus();
    const auto pose = getCurrentPose();
    const auto task_objects_array = getTaskObjects();
    const auto xyz = GetCenterGate(task_objects_array);
    get_parameter("goal_tolerance", goal_tolerance_);

    if (xyz) {
      goal_.header.frame_id = "map";
      goal_.pose.position.x = xyz->position.x;
      goal_.pose.position.y = xyz->position.y;
      goal_.pose.position.z = 0.0;

      goal_.pose.orientation.w = xyz->orientation.w;
      goal_.pose.orientation.x = xyz->orientation.x;
      goal_.pose.orientation.y = xyz->orientation.y;
      goal_.pose.orientation.z = xyz->orientation.z;
    }

    if (pose) {
      distance_ = getDistance(pose.value(), goal_.pose);
    }

    if (distance_ < goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }

    goal_.header.stamp = get_clock()->now();
    if (status_planner.value()->status != 1) {
      goal_pub_gate_->publish(goal_);
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

  float calculateDistance(float x, float y)
  {
    auto pose = getCurrentPose();
    auto dx = x - static_cast<float>(pose->position.x);
    auto dy = y - static_cast<float>(pose->position.y);

    return std::sqrt(dx * dx + dy * dy);
  }

  const std::optional<geometry_msgs::msg::Pose> GetCenterGate(
    robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr task_objects_array)
  {
    std::vector<robotx_behavior_msgs::msg::TaskObject> green_buoys_array;
    std::vector<robotx_behavior_msgs::msg::TaskObject> red_buoys_array;
    geometry_msgs::msg::Pose goal_coordinate;
    float x;
    float y;
    int cnt = 0;
    float red_buoy_x;
    float red_buoy_y;
    float green_buoy_x;
    float green_buoy_y;

    if (task_objects_array) {
      for (size_t i = 0; i < task_objects_array->task_objects.size(); i++) {
        if (task_objects_array->task_objects[i].object_kind == static_cast<int>(Buoy::BUOY_RED)) {
          red_buoys_array.push_back(task_objects_array->task_objects[i]);
        }
        if (task_objects_array->task_objects[i].object_kind == static_cast<int>(Buoy::BUOY_GREEN)) {
          green_buoys_array.push_back(task_objects_array->task_objects[i]);
        }
      }

      for (const auto & e : red_buoys_array) {
        if (cnt == 0) {
          red_buoy_x = e.x;
        }
        if (red_buoy_x > e.x) {
          red_buoy_x = e.x;
        }

        if (cnt == 0) {
          red_buoy_y = e.y;
        }
        if (red_buoy_y > e.y) {
          red_buoy_y = e.y;
        }
        cnt++;
      }

      cnt = 0;

      for (const auto & e : green_buoys_array) {
        if (cnt == 0) {
          green_buoy_x = e.x;
        }
        if (green_buoy_x > e.x) {
          green_buoy_x = e.x;
        }

        if (cnt == 0) {
          green_buoy_y = e.y;
        }
        if (red_buoy_y > e.y) {
          green_buoy_y = e.y;
        }
        cnt++;
      }

      x = (red_buoy_x + green_buoy_x) / 2.0;
      y = (red_buoy_y + green_buoy_y) / 2.0;

      goal_coordinate.position.x = x;
      goal_coordinate.position.y = y;
      goal_coordinate.position.z = 0.0;
      goal_coordinate.orientation.w = 1.0;
      goal_coordinate.orientation.x = 0.0;
      goal_coordinate.orientation.y = 0.0;
      goal_coordinate.orientation.z = 0.0;

      return goal_coordinate;
    }
    return std::nullopt;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToGateAction)