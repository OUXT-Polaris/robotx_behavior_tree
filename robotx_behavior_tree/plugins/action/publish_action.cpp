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
#include "std_msgs/msg/empty.hpp"

namespace robotx_behavior_tree
{
class PubSomeAction : public ActionROS2Node
{
public:
  PubSomeAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    declare_parameter("Topic", "/command");
    get_parameter("Topic", topic_);
    pub_some_ = this->create_publisher<std_msgs::msg::Empty>(topic_, 1);  // CHANGE when you use
  }

protected:
  BT::NodeStatus tick() override
  {
    auto input = this->getInput<std_msgs::msg::Empty>("input_");  // CHANGE when you use
    if (!has_published) {
      if (input) {
        pub_some_->publish(input.value());
        has_published = true;
        RCLCPP_INFO(get_logger(), "PubSomeAction : maybe published ");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(get_logger(), "PubSomeAction : Faild to pushish");
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_some_;  // CHANGE when you use
  bool has_published = false;
  std::string topic_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, PubSomeAction)
