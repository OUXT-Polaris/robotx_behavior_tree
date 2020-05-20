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

#include <string>
#include <memory>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotx_waypoint_msgs/action/way_point.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace robotx_behavior_tree
{
using WayPoint = robotx_waypoint_msgs::action::WayPoint;
class GoAction : public nav2_behavior_tree::BtActionNode<robotx_waypoint_msgs::action::WayPoint>
{
public:
  GoAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : nav2_behavior_tree::BtActionNode<robotx_waypoint_msgs::action::WayPoint>(name, "waypoint",
      config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Pose>("target_pose")};
  }

protected:
  void on_tick() override
  {
    if (!getInput("target_pose", goal_.pose)) {
      RCLCPP_ERROR(node_->get_logger(), "target_pose is not provided");
    }
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, GoAction)
