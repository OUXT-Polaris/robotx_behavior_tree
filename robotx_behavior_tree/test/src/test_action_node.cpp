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
 * @file test_action_node.cpp
 * @author Kento Hirogaki hkt8g2r6kin@gmail.com
 * @brief test code for Action Node
 * @version 0.1
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>

#include <robotx_behavior_tree/action_node.hpp>

TEST(TestSuite, testCase1)
{
  robotx_behavior_msgs::msg::TaskObject object;
  // robotx_behavior_tree::ActionROS2Node *testtest();
  object.x = 0;
  object.y = 0;

  // object->x = 0;
  // object->y = 0;
  // robotx_behavior_tree::ActionROS2Node::getFrontPoseForWaypoint(object, 2.0);

  // robotx_behavior_tree::getFrontPoseForWaypoint(object, 2.0);
  // robotx_behavior_tree::ActionROS2Node::getPoint(object);
  EXPECT_EQ(true, true);
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