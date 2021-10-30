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

#ifndef BEHAVIOR_TREE_ACTION_BUILDER__REGISTER_NODES_HPP_
#define BEHAVIOR_TREE_ACTION_BUILDER__REGISTER_NODES_HPP_
namespace register_nodes
{
class NodeBuilder
{
public:
  template <class TNodeClass>
  static auto build()
  {
    return [](
             const std::string & name,  // NOLINT
             const BT::NodeConfiguration & config) {
      return std::make_unique<TNodeClass>(name, config);  // NOLINT
    };
  }
};
}  // namespace register_nodes

#define REGISTER_NODES(node_namespace, class_name)                                    \
  BT_REGISTER_NODES(factory)                                                          \
  {                                                                                   \
    factory.registerBuilder<node_namespace::class_name>(                              \
      #class_name, register_nodes::NodeBuilder::build<node_namespace::class_name>()); \
  }

#endif  // BEHAVIOR_TREE_ACTION_BUILDER__REGISTER_NODES_HPP_
