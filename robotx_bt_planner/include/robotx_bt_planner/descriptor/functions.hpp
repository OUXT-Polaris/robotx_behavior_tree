// Copyright (c) 2020, OUXT-Polaris
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

#ifndef ROBOTX_BT_PLANNER__DESCRIPTOR__FUNCTIONS_HPP_
#define ROBOTX_BT_PLANNER__DESCRIPTOR__FUNCTIONS_HPP

#include "sol/sol.hpp"

void addPresetFunctions(sol::state & lua)
{
  lua["add"] = [](int a, int b) -> int { return a + b; };
}
#endif  // ROBOTX_BT_PLANNER__DESCRIPTOR__FUNCTIONS_HPP
