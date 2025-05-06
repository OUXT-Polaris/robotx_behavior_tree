// Copyright (c) 2023 OUXT Polaris
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

/**
 * @file test_go_around_object.cpp
 * @author Kento Hirogaki hkt8g2r6kin@gmail.com
 * @brief test code for GO Around Object
 * @version 0.1
 * @date 2025-03-18
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <gtest/gtest.h>
#include <robotx_behavior_tree/action_node.hpp>
#include "robotx_behavior_tree/go_around_object.hpp"
TEST(TestSuite, testCase1)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("object_type", std::string("red_bouy"));
  blackboard->set("turning_direction", std::string("clockwise"));
  blackboard->set("orbit_angle", 180.0);
  BT::NodeConfiguration config;
  config.blackboard = blackboard;
  robotx_behavior_tree::GoAroundObject action("go_around", config);
  ASSERT_EQ(BT::NodeStatus::SUCCESS, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(action.getGoalTolerance(), 0.5);
  // ASSERT_EQ(action.onStart(), BT::NodeStatus::RUNNING);
  // EXPECT_EQ(true, false);
}
/**
 * @brief Run all the tests that were declared with TEST()
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}