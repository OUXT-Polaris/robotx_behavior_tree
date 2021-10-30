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
#ifndef BEHAVIOR_TREE_ACTION_BUILDER__ACTION_NODE_HPP_
#define BEHAVIOR_TREE_ACTION_BUILDER__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <string>

namespace behavior_tree_action_builder
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
}  // namespace behavior_tree_action_builder

#endif  // BEHAVIOR_TREE_ACTION_BUILDER__ACTION_NODE_HPP_
