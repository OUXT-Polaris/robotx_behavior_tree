// Copyright (c) 2019, OUXT-Polaris
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

#include <glog/logging.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robotx_bt_planner/bt_planner_component.hpp>

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotx_bt_planner::BTPlannerComponent>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
