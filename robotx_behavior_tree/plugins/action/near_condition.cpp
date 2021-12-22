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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace robotx_behavior_tree
{
class NearCondition : public BT::ConditionNode
{
public:
    NearCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
      topic_ = this->getInput<std::string>("topic");
      target_ = this->getInput<geometry_msgs::msg::Point>("target");
      threshold_ = this->getInput<double>("threshold");
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Point>("target"), BT::InputPort<double>("threshold"),  BT::InputPort<std::string>("topic")};
  }
protected:
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::RUNNING;
  }
  std::string topic_;
  double threshold_;
  geometry_msgs::msg::Point target_;
};

}  // namespace robotx_behavior_tree

