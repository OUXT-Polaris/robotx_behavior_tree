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

#include "robotx_behavior_tree/action_node.hpp"

namespace robotx_behavior_tree
{
class GoAction : public ActionNode
{
public:
  GoAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  :  ActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("test_input")};
  }

protected:
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::RUNNING;
  }
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, GoAction)
