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

#include <memory>
#include <string>
// some type of msgs
#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_tree/action_node.hpp"
#include "robotx_behavior_msgs/msg/task_objects_array.hpp"

namespace robotx_behavior_tree
{
class SubscribeObjectsAction : public ActionROS2Node
{
public:
  SubscribeObjectsAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    subscription_ = this->create_subscription<robotx_behavor_msgs::msg::TaskObjectsArray>(
    "/TaskObjectsArray", 2, std::bind(&SubscribeObjectsAction::topic_callback, this, std::placeholders::_1));
  }


  static BT::PortsList providedPorts() { return {BT::OutputPort<robotx_behavor_msgs::msg::TaskObjectsArray>("task_objects")}; }

protected:
  BT::NodeStatus tick() override
  {
    
    return BT::NodeStatus::RUNNING;
  }

  void topic_callback(const robotx_behavor_msgs::msg::TaskObjectsArray::SharedPtr msg)
  { 
    task_objects_ = msg; 
    setOutput<robotx_behavor_msgs::msg::TaskObjectsArray>("task_objects", task_objects_);
  };



  rclcpp::Subscription<robotx_behavor_msgs::msg::TaskObjectsArray>::SharedPtr subscription_;
  robotx_behavor_msgs::msg::TaskObjectsArray task_objects_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, SubscribeObjectsAction)
