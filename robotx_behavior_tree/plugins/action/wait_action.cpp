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
#include "robotx_behavior_tree/action_node.hpp"

namespace robotx_behavior_tree
{
class WaitAction : public ActionROS2Node
{
public:
  WaitAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    declare_parameter("wait_time", 5000.0);  //double millisecond
    get_parameter("wait_time", wait_time_);
  }

protected:
  BT::NodeStatus tick() override
  {
    if (!isSetWaitTime)
      if (wait_time_) {
        RCLCPP_INFO(get_logger(), "WaitAction : waiting %f ms", wait_time_);
      } else {
        RCLCPP_WARN(get_logger(), "WaitAction : Faild to get wait_time. Force to wait 5000.0ms");
        wait_time_ = 5000.0;
      }
    start = std::chrono::system_clock::now();
    isSetWaitTime = true;
  }

  end = std::chrono::system_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  RCLCPP_INFO(get_logger(), "WaitAction : %f millisecond passed", elapsed);

  if (elapsed >= wait_time_) {
    RCLCPP_INFO(get_logger(), "WaitAction : SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

double wait_time_;
double elapsed;
bool isSetWaitTime = false;
std::chrono::system_clock::time_point start, end;
};  // namespace robotx_behavior_tree
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, WaitAction)
