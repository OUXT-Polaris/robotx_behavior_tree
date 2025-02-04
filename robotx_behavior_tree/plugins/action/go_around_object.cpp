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
    declare_parameter("goal_tolerance", 5.0);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_pub_front_pose_of_object_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }
  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(), {BT::InputPort<std::string>("object_type")});
  }

private:
  rclcpp::TimerBase::SharedPtr update_position_timer_;
  float goal_distance_;
  double goal_tolerance_;
  // static int count = 0;
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
  // BT::NodeStatus dispQWR() 
  // {
  //   RCLCPP_INFO(get_logger(), "super qwerty: %d", 39);
  //   return BT::NodeStatus::FAILURE;
  //   // return 0;
  // }

  BT::NodeStatus dispQWR() 
  {
    const auto status_planner = getPlannerStatus();
    RCLCPP_INFO(get_logger(), "status_planner.value()->status: %d", status_planner.value()->status);
    const auto pose = getCurrentPose();
    const auto task_objects_array = getTaskObjects();
    auto object_type = this->getInput<std::string>("object_type");

    if (task_objects_array) {
      if (object_type.value() == "red_bouy") {
        target_objects_array_ =
          filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_RED));
      } else if (object_type.value() == "green_bouy") {
        target_objects_array_ =
          filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_GREEN));
      } else if (object_type.value() == "white_bouy") {
        target_objects_array_ =
          filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_WHITE));
      } else if (object_type.value() == "black_bouy") {
        target_objects_array_ =
          filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_BLACK));
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
    sortBy2DDistance(target_objects_array_, pose.value()->pose.position);
    if (target_objects_array_.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    auto bouy_distance = 5.5;
    int goal_position_num = 0;
    const auto goal_pose = getAroundPoseOfObject(target_objects_array_[0], bouy_distance, goal_position_num);
    get_parameter("goal_tolerance", goal_tolerance_);
    goal_.header.frame_id = "map";
    if (goal_pose) {
      goal_.pose.position.x = goal_pose.value().position.x;
      goal_.pose.position.y = goal_pose.value().position.y;
      goal_.pose.position.z = goal_pose.value().position.z;
      goal_.pose.orientation.w = goal_pose.value().orientation.w;
      goal_.pose.orientation.x = goal_pose.value().orientation.x;
      goal_.pose.orientation.y = goal_pose.value().orientation.y;
      goal_.pose.orientation.z = goal_pose.value().orientation.z;
    }
    goal_.header.stamp = get_clock()->now();
    goal_distance_ = getDistance(pose.value()->pose.position, goal_.pose.position);
    static int count = 0;
    RCLCPP_INFO(get_logger(), "Count: %d", count);
    RCLCPP_INFO(get_logger(), "goal_distance_: %f", goal_distance_);
    RCLCPP_INFO(get_logger(), "goal_tolerance_: %f", goal_tolerance_);
    if (goal_distance_ < goal_tolerance_) {
      goal_pub_front_pose_of_object_->publish(goal_);
    }
    if (count == 0) {
      goal_pub_front_pose_of_object_->publish(goal_);
      count++;
    }
  }

  BT::NodeStatus onStart() override
  {
    const auto status_planner = getPlannerStatus();
    RCLCPP_INFO(get_logger(), "status_planner.value()->status: %d", status_planner.value()->status);
    RCLCPP_INFO(get_logger(), "qwerty: %d", status_planner.value()->status);
    const auto task_objects_array = getTaskObjects();
    if (task_objects_array) {
      RCLCPP_INFO(get_logger(), "get task_objects");
      return BT::NodeStatus::RUNNING;
    } else {
      RCLCPP_INFO(get_logger(), "Not Found task_objects");
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onRunning() override
  {
    // const auto result_dispQWR = dispQWR();
    // if (result_dispQWR == BT::NodeStatus::FAILURE) {
    //   RCLCPP_INFO(get_logger(), "Throgh Goal : FAILURE");
    //   return BT::NodeStatus::FAILURE;
    // } else if (result_dispQWR == BT::NodeStatus::SUCCESS) {
    //   RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
    //   return BT::NodeStatus::SUCCESS;
    // // } 

    const auto result_dispQWR = dispQWR();
    if (result_dispQWR == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : FAILURE");
      return BT::NodeStatus::FAILURE;
    } else if (result_dispQWR == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    } 

    // const auto status_planner = getPlannerStatus();
    // RCLCPP_INFO(get_logger(), "status_planner.value()->status: %d", status_planner.value()->status);
    // const auto pose = getCurrentPose();
    // const auto task_objects_array = getTaskObjects();
    // auto object_type = this->getInput<std::string>("object_type");

    // if (task_objects_array) {
    //   if (object_type.value() == "red_bouy") {
    //     target_objects_array_ =
    //       filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_RED));
    //   } else if (object_type.value() == "green_bouy") {
    //     target_objects_array_ =
    //       filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_GREEN));
    //   } else if (object_type.value() == "white_bouy") {
    //     target_objects_array_ =
    //       filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_WHITE));
    //   } else if (object_type.value() == "black_bouy") {
    //     target_objects_array_ =
    //       filter(task_objects_array.value(), static_cast<short>(Buoy::BUOY_BLACK));
    //   } else {
    //     return BT::NodeStatus::FAILURE;
    //   }
    // }
    // sortBy2DDistance(target_objects_array_, pose.value()->pose.position);
    // if (target_objects_array_.empty()) {
    //   return BT::NodeStatus::FAILURE;
    // }

    // auto bouy_distance = 5.5;
    // int goal_position_num = 0;
    // const auto goal_pose = getAroundPoseOfObject(target_objects_array_[0], bouy_distance, goal_position_num);
    // get_parameter("goal_tolerance", goal_tolerance_);
    // goal_.header.frame_id = "map";
    // if (goal_pose) {
    //   goal_.pose.position.x = goal_pose.value().position.x;
    //   goal_.pose.position.y = goal_pose.value().position.y;
    //   goal_.pose.position.z = goal_pose.value().position.z;
    //   goal_.pose.orientation.w = goal_pose.value().orientation.w;
    //   goal_.pose.orientation.x = goal_pose.value().orientation.x;
    //   goal_.pose.orientation.y = goal_pose.value().orientation.y;
    //   goal_.pose.orientation.z = goal_pose.value().orientation.z;
    // }
    // goal_.header.stamp = get_clock()->now();
    // goal_distance_ = getDistance(pose.value()->pose.position, goal_.pose.position);
    // static int count = 0;
    // RCLCPP_INFO(get_logger(), "Count: %d", count);
    // RCLCPP_INFO(get_logger(), "goal_distance_: %f", goal_distance_);
    // RCLCPP_INFO(get_logger(), "goal_tolerance_: %f", goal_tolerance_);
    // if (goal_distance_ < goal_tolerance_) {
    //   goal_pub_front_pose_of_object_->publish(goal_);
    // }
    // if (count == 0) {
    //   goal_pub_front_pose_of_object_->publish(goal_);
    //   count++;
    // }
    if (false) {
      RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, GoAroundObject)
