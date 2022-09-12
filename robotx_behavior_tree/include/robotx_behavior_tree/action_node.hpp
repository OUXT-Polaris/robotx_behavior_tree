// Copyright (c) 2020, OUXT-Polaris
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

#include <hermite_path_msgs/msg/planner_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotx_behavior_msgs/msg/task_objects_array_stamped.hpp>
#include <stdexcept>
#include <string>

namespace robotx_behavior_tree
{
class ActionNode : public BT::StatefulActionNode
{
public:
  ActionNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    setRegistrationID(name);
  }

protected:
  void onHalted() override {}
  std::string name;
};

class ActionROS2Node : public BT::StatefulActionNode, public rclcpp::Node
{
public:
  ActionROS2Node(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), rclcpp::Node(name, rclcpp::NodeOptions())
  {
    setRegistrationID(name);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr>("task_objects"),
      BT::InputPort<hermite_path_msgs::msg::PlannerStatus::SharedPtr>("planner_status")};
  }
  static BT::PortsList appendPorts(const BT::PortsList & ports1, const BT::PortsList & ports2)
  {
    BT::PortsList ports = {};
    for (const auto & port : ports1) {
      ports.emplace(port.first, port.second);
    }
    for (const auto & port : ports2) {
      ports.emplace(port.first, port.second);
    }
    return ports;
  }

protected:
  void onHalted() override {}
  std::string name;
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr getTaskObjects() const
  {
    const auto ret =
      this->getInput<robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr>("task_objects");
    if (ret) {
      return ret.value();
    } else {
      throw std::runtime_error("Task objects were not subscribed.");
    }
  }

#define DEFINE_GET_INPUT(NAME, TYPE, BLACKBOARD_KEY) \
  std::optional<TYPE> get##NAME() const              \
  {                                                  \
    const auto ret = getInput<TYPE>(BLACKBOARD_KEY); \
    if (ret) {                                       \
      return ret.value();                            \
    }                                                \
    return std::nullopt;                             \
  }

  DEFINE_GET_INPUT(
    PlannerStatus, hermite_path_msgs::msg::PlannerStatus::SharedPtr, "planner_status");
#undef DEFINE_GET_INPUT
};
}  // namespace robotx_behavior_tree

#endif  // ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_
