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


#ifndef ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_
#define ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hermite_path_msgs/msg/planner_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotx_behavior_msgs/msg/task_objects_array_stamped.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace robotx_behavior_tree
{
class ActionNode : public BT::StatefulActionNode
{
}}