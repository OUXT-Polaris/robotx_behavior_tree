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

#include <unistd.h>

#include <chrono>
#include <memory>
#include <robotx_behavior_tree/action_node.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robotx_behavior_tree
{
class WaitAction : public ActionROS2Node
{
public:
  WaitAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    /*
    declare_parameter("wait_time", 5000.0);  //double millisecond
    get_parameter("wait_time", wait_time_);
    */
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(ActionROS2Node::providedPorts(), {BT::InputPort<double>("wait_time")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    auto wait_time = this->getInput<double>("wait_time");
    start_time = get_clock()->now();
    if (wait_time) {
      RCLCPP_INFO(get_logger(), "WaitAction : waiting %f sec", wait_time.value());
      threshold_duration = wait_time.value();
    } else {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    elapsed = (get_clock()->now() - start_time).seconds();
    if (elapsed >= threshold_duration) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  double wait_time_;
  double elapsed;
  rclcpp::Time start_time;
  double threshold_duration;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, WaitAction)
