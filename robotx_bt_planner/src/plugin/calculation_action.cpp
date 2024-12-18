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

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>
#include <string>

namespace robotx_bt_planner
{
class ActionNode : public BT::SyncActionNode
{
public:
  ActionNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    setRegistrationID(name);
  }

protected:
  std::string name;
};
class CalculationAction : public ActionNode
{
public:
  CalculationAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("eval")}; }

protected:
  BT::NodeStatus tick() override
  {
    auto eval = getInput<std::string>("eval");
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace robotx_bt_planner

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_bt_planner, CalculationAction)
