// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
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


/*
 * Note
 *
 * This header file is based on https://github.com/BehaviorTree/BehaviorTree.ROS/blob/master/include/behaviortree_ros/bt_action_node.h
 * So, this will merge into BehaviorTree.ROS when porting for ROS2 is completed
 *
 */
#ifndef ROBOTX_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define ROBOTX_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_


#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace robotx_behavior_tree
{
template<class ActionT>
class ActionNodeWithClient : public BT::SyncActionNode, rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  ActionNodeWithClient(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :  BT::SyncActionNode(name, config), rclcpp::Node(name, options)
  {
    setRegistrationID(name);
  }
  class NodeBuilder
  {
  public:
    template<class TNodeClass>
    static auto build()
    {
      return [](const std::string & name, // NOLINT
                const BT::NodeConfiguration & config) {
        return std::make_unique<TNodeClass>(name, config); // NOLINT
      };
    }
  };

protected:
  std::string name;
  virtual void callbackGoalResponce(std::shared_future<typename GoalHandle::SharedPtr>) = 0;
  virtual void callbackFeedback(
    typename GoalHandle::SharedPtr,
    const std::shared_ptr < typename ActionT::Feedback >) = 0;
  virtual void callbackResult(const typename GoalHandle::WrappedResult &) = 0;
};

namespace register_nodes
{

}  // namespace register_nodes

#define REGISTER_NODES(node_namespace, class_name) \
  BT_REGISTER_NODES(factory) { \
    factory.registerBuilder<node_namespace::class_name>(#class_name, \
      register_nodes::NodeBuilder::build<node_namespace::class_name>()); \
  } \
}  // namespace BT

#endif  // ROBOTX_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
