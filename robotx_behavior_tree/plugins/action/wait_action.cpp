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
#include <chrono>

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
    declare_parameter("wait_time", "5.0");
    get_parameter("wait_time", wait_time_);
  }

protected:
  BT::NodeStatus tick() override
  {
    using namespace std::chrono_literals;
    rclcpp::WallRate loop_rate(1s);
    count = 0;

    if (wait_time_) {
        RCLCPP_INFO(
            get_logger(), "WaitAction : waiting %d second", wait_time_);
    } 
    else {
        RCLCPP_WARN(get_logger(), "WaitAction : Faild to get wait_time. Force to wait 5s");
        wait_time_ = 5;
    }

    while (count != wait_time_)
    {
        loop_rate.sleep();
        count += 1;
        RCLCPP_INFO(
            get_logger(), "WaitAction : %d second passed", count);
        if (count == wait_time_) {
            return BT::NodeStatus::SUCCESS;
        }
        else if(count > wait_time_)
        {
         RCLCPP_WARN(
            get_logger(), "WaitAction : OverTime!! Force to wake up");  
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;  
  }

  int wait_time_;
  int count;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, WaitAction)
