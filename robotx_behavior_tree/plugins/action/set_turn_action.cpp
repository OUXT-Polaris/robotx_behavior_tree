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
#include <robotx_behavior_tree/action_node.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robotx_behavior_tree
{
class SetTurnAction : public ActionROS2Node
{
  enum class TurnDirection { LEFT, RIGHT };

public:
  SetTurnAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(
      ActionROS2Node::providedPorts(),
      {BT::OutputPort<double>("goal_x1"),     BT::OutputPort<double>("goal_y1"),
       BT::OutputPort<double>("goal_theta1"), BT::OutputPort<double>("goal_x2"),
       BT::OutputPort<double>("goal_y2"),     BT::OutputPort<double>("goal_theta2"),
       BT::OutputPort<double>("goal_x3"),     BT::OutputPort<double>("goal_y3"),
       BT::OutputPort<double>("goal_theta3"), BT::OutputPort<double>("goal_x4"),
       BT::OutputPort<double>("goal_y4"),     BT::OutputPort<double>("goal_theta4"),
       BT::OutputPort<double>("goal_x5"),     BT::OutputPort<double>("goal_y5"),
       BT::OutputPort<double>("goal_theta5"), BT::OutputPort<double>("goal_x6"),
       BT::OutputPort<double>("goal_y6"),     BT::OutputPort<double>("goal_theta6"),
       BT::OutputPort<double>("goal_x7"),     BT::OutputPort<double>("goal_y7"),
       BT::OutputPort<double>("goal_theta7"), BT::OutputPort<double>("goal_x8"),
       BT::OutputPort<double>("goal_y8"),     BT::OutputPort<double>("goal_theta8"),
       BT::InputPort<double>("radius"),       BT::InputPort<std::string>("dir"),
       BT::InputPort<double>("pose_x"),       BT::InputPort<int>("pose_y"),
       BT::InputPort<double>("pose_v_x"),     BT::InputPort<int>("pose_v_y")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    auto dir = this->getInput<std::string>("dir");
    if (dir) {
      if (dir.value() == "left") {
        turn_direction_ = TurnDirection::LEFT;
        RCLCPP_INFO(get_logger(), "SetTurnAction : turning left");
      } else if (dir.value() == "right") {
        turn_direction_ = TurnDirection::RIGHT;
        RCLCPP_INFO(get_logger(), "SetTurnAction : turning right");
      } else {
        RCLCPP_ERROR(get_logger(), "SetTurnAction : invalid dir");
        return BT::NodeStatus::FAILURE;
      }
    }
    auto radius = this->getInput<double>("radius");
    auto pose_x = this->getInput<double>("pose_x");
    auto pose_y = this->getInput<double>("pose_y");
    auto pose_v_x = this->getInput<double>("pose_v_x");
    auto pose_v_y = this->getInput<double>("pose_v_y");
    if (radius && pose_x && pose_y && pose_v_x && pose_v_y) {
      radius_ = radius.value();
      pose_x_ = pose_x.value();
      pose_y_ = pose_y.value();
      pose_v_x_ = pose_v_x.value();
      pose_v_y_ = pose_v_y.value();
    } else {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    int N = 8;  // vertex
    thetap_ = std::atan2(pose_v_y_ - pose_y_, pose_v_x_ - pose_x_);
    std::vector<double> goalx(N);
    std::vector<double> goaly(N);
    std::vector<double> goaltheta(N);
    double o = 360 / N;
    for (int i = 0; i < N; i++) {
      goalx[i] =
        std::cos(-thetap_) * (pose_v_x_ - radius_ * std::cos((o / 2 + o * i) * 3.14 / 180)) -
        std::sin(-thetap_) * (pose_v_y_ - radius_ * dir_ * std::sin((o / 2 + o * i) * 3.14 / 180));
      goaly[i] =
        std::sin(-thetap_) * (pose_v_x_ - radius_ * std::cos((o / 2 + o * i) * 3.14 / 180)) +
        std::cos(-thetap_) * (pose_v_y_ - radius_ * dir_ * std::sin((o / 2 + o * i) * 3.14 / 180));
    }
    for (int j = 0; j < (N - 1); j++) {
      goaltheta[j] = std::atan2(goaly[j + 1] - goaly[j], goalx[j + 1] - goalx[j]);
    }
    goaltheta[N - 1] = goaltheta[N - 2];

    setOutput<double>("goal_x1", goalx[0]);
    setOutput<double>("goal_y1", goaly[0]);
    setOutput<double>("goal_theta1", goaltheta[0]);

    setOutput<double>("goal_x2", goalx[1]);
    setOutput<double>("goal_y2", goaly[1]);
    setOutput<double>("goal_theta2", goaltheta[1]);

    setOutput<double>("goal_x3", goalx[2]);
    setOutput<double>("goal_y3", goaly[2]);
    setOutput<double>("goal_theta3", goaltheta[2]);

    setOutput<double>("goal_x4", goalx[3]);
    setOutput<double>("goal_y4", goaly[3]);
    setOutput<double>("goal_theta4", goaltheta[3]);

    setOutput<double>("goal_x5", goalx[4]);
    setOutput<double>("goal_y5", goaly[4]);
    setOutput<double>("goal_theta5", goaltheta[4]);

    setOutput<double>("goal_x6", goalx[5]);
    setOutput<double>("goal_y6", goaly[5]);
    setOutput<double>("goal_theta6", goaltheta[5]);

    setOutput<double>("goal_x7", goalx[6]);
    setOutput<double>("goal_y7", goaly[6]);
    setOutput<double>("goal_theta7", goaltheta[6]);

    setOutput<double>("goal_x8", goalx[7]);
    setOutput<double>("goal_y8", goaly[7]);
    setOutput<double>("goal_theta8", goaltheta[7]);

    RCLCPP_INFO(get_logger(), "SetTurnAction : SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

private:
  double radius_, pose_x_, pose_y_, pose_v_x_, pose_v_y_, thetap_;
  int dir_;
  TurnDirection turn_direction_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, SetTurnAction)
