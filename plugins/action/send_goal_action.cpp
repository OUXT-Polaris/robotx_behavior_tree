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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_tree/action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robotx_behavior_tree
{
class SendGoalAction : public ActionROS2Node
{
public:
  SendGoalAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  :  ActionROS2Node(name, config)
  {
    goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }

protected:
  BT::NodeStatus tick() override
  {
    auto goal = this->getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (goal) {
      goal_pub_->publish(goal.value());
    }

    return BT::NodeStatus::SUCCESS;
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, SendGoalAction)
