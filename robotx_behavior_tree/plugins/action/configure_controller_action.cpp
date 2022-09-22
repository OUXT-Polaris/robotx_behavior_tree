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
#include <string>

namespace robotx_behavior_tree
{
class ConfigureControllerAction : public ActionROS2Node
{
public:
  ConfigureControllerAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(ActionROS2Node::providedPorts(), {BT::InputPort<uint8_t>("mode")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    this->getInput<uint8_t>("mode");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::FAILURE; }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, ConfigureControllerAction)
