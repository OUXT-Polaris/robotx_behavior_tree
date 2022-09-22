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

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <memory>
#include <robotx_behavior_tree/action_node.hpp>
#include <robotx_msgs/msg/autonomous_maritime_system_status.hpp>
#include <string>

namespace robotx_behavior_tree
{
class ConfigureControllerAction : public ActionROS2Node
{
public:
  ConfigureControllerAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config), requested_mode_(0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(ActionROS2Node::providedPorts(), {BT::InputPort<uint8_t>("mode")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    auto mode = this->getInput<uint8_t>("mode");
    if (!mode) {
      return BT::NodeStatus::FAILURE;
    }
    switch (mode.value()) {
      case robotx_msgs::msg::AutonomousMaritimeSystemStatus::REMOTE_OPERATED:
        break;
      case robotx_msgs::msg::AutonomousMaritimeSystemStatus::AUTONOMOUS:
        break;
      case robotx_msgs::msg::AutonomousMaritimeSystemStatus::KILLED:
        break;
      default:
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::FAILURE; }

private:
  uint8_t requested_mode_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  void switchController(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, ConfigureControllerAction)
