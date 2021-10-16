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

#ifndef ROBOTX_BT_PLANNER__DESCRIPTOR__DATA_STRUCTURES_HPP_
#define ROBOTX_BT_PLANNER__DESCRIPTOR__DATA_STRUCTURES_HPP_

#include <string>
#include <vector>

namespace robotx_bt_planner
{
struct BlackBoard
{
  std::string input;
  std::string eval;
};

struct Behavior
{
  std::string description;
  std::vector<BlackBoard> blackboard;
};
struct Position
{
  double x, y, z;
};
struct Quaternion
{
  double x, y, z, w;
};
struct Pose
{
  Position position;
  Quaternion orientation;
};
struct Pose2D
{
  double x, y, theta;
};

struct Object
{
  int uuid;
  std::vector<std::string> attributes;
  Pose pose;
};

struct Format
{
  Behavior behavior;
  std::vector<Object> objects;
};
}  // namespace robotx_bt_planner
#endif  // ROBOTX_BT_PLANNER__DESCRIPTOR__DATA_STRUCTURES_HPP_
