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
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "robotx_behavior_tree/go_around_object.hpp"
class SubscriberTestNode : public rclcpp::Node
{
public:
  SubscriberTestNode(const std::string & node_name)
  : Node(node_name), received_msg_(nullptr), msg_received_flag_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/go_around_object/started", 10,
      std::bind(&SubscriberTestNode::callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "SubscriberTestNode created, waiting for messages on /go_around_object/started");
  }
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message");
    received_msg_ = msg;
    msg_received_flag_ = true;
  }
  bool hasReceivedMessage() const { return msg_received_flag_; }
  std_msgs::msg::String::SharedPtr getReceivedMessage() const { return received_msg_; }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std_msgs::msg::String::SharedPtr received_msg_;
  bool msg_received_flag_;
};

TEST(TestSuite, testCase1)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SubscriberTestNode>("test_subscriber_node");
  const auto timeout = std::chrono::seconds(25);
  const auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !node->hasReceivedMessage() &&
         (std::chrono::steady_clock::now() - start_time) < timeout) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();
  ASSERT_TRUE(node->hasReceivedMessage())
    << "Timeout: No message received on /go_around_object/started within " << timeout.count()
    << " seconds.";
  auto received_msg = node->getReceivedMessage();
  ASSERT_NE(received_msg, nullptr);
  EXPECT_EQ(received_msg->data, "GoAroundObject node started successfully!");
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