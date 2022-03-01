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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_tree/action_node.hpp"

namespace robotx_behavior_tree
{
class SetTurnAction : public ActionROS2Node
{
public:
  WaitAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {OutputPort<double>("goal_x1"),     OutputPort<double>("goal_y1"),
            OutputPort<double>("goal_x2"),     OutputPort<double>("goal_y2"),
            OutputPort<double>("goal_x3"),     OutputPort<double>("goal_y3"),
            OutputPort<double>("goal_x4"),     OutputPort<double>("goal_y4"),
            OutputPort<double>("goal_x5"),     OutputPort<double>("goal_y5"),
            BT::InputPort<double>("radius"),   BT::InputPort<int>("dir"),
            BT::InputPort<double>("pose_x"),   BT::InputPort<int>("pose_y"),
            BT::InputPort<double>("pose_v_x"), BT::InputPort<int>("pose_v_y")};
  }

protected:
  BT::NodeStatus tick() override
  {
    if (!isSet) {
      auto radius = this->getInput<double>("rasius");
      auto dir = this->getInput<int>("turn");
      auto pose_x = this->getInput<double>("rasius");
      auto pose_y = this->getInput<double>("turn");
      auto pose_v_x = this->getInput<double>("rasius");
      auto pose_v_y = this->getInput<double>("turn");

      radius_ = radius.value();
      dir_ = dir.value();
      pose_x_ = pose_x.value();
      pose_y_ = pose_y.value();
      pose_v_x_ = pose_v_x.value();
      pose_v_y_ = pose_v_y.value();

      isSet = true;
    }

    if (!success) {
      thetap_ = std::atan2(pose_v_y_ - pose_y_, pose_v_x_ - pose_x_);

      for (int i = 0; i < 5; i++) {
            goal[i] = std::cos(-thetap_)*(pose_v_x_ - radius_*std::cos((36+72*i)*3.14/180) - std::sin(-thetap_)*(pose_v_y_ - radius_*dir_*std::sin((36+72*i)*3.14/180);
            goal[i+1] = std::sin(-thetap_)*(pose_v_x_ - radius_*std::cos((36+72*i)*3.14/180) + std::cos(-thetap_)*(pose_v_y_ - radius_*dir_*std::sin((36+72*i)*3.14/180);
      }
      setOutput<double>("goal_x1", goal[0]);
      setOutput<double>("goal_y1", goal[1]);
      setOutput<double>("goal_x2", goal[2]);
      setOutput<double>("goal_y2", goal[3]);
      setOutput<double>("goal_x3", goal[4]);
      setOutput<double>("goal_y3", goal[5]);
      setOutput<double>("goal_x4", goal[6]);
      setOutput<double>("goal_y4", goal[7]);
      setOutput<double>("goal_x5", goal[8]);
      setOutput<double>("goal_y5", goal[9]);

      RCLCPP_INFO(get_logger(), "SetTurnAction : SUCCESS");
      success = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<double> goal(10);
  double radius_, pose_x_, pose_y_, pose_v_x_, pose_v_y_, thetap_;
  int dir_;
  bool isSet = false;
  bool success = false;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, SetTurnAction)
