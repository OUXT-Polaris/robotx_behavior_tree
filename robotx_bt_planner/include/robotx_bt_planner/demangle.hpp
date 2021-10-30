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

#ifndef ROBOTX_BT_PLANNER__DEMANGLE_HPP_
#define ROBOTX_BT_PLANNER__DEMANGLE_HPP_

#include <cxxabi.h>

#include <cstdlib>
#include <memory>
#include <string>

struct free_delete
{
  template <typename T>
  void operator()(T * ptr) const noexcept
  {
    std::free(ptr);
  }
};

std::string demangle(std::type_info const & ti)
{
  int status = 0;
  std::unique_ptr<char, free_delete> ptr(abi::__cxa_demangle(ti.name(), nullptr, nullptr, &status));

  if (!ptr) {
    switch (status) {
      case -1:
        return "Failed to allocate memory";
      case -2:
        return "Got an invalid name";
      case -3:
        return "Got invalid arguments";
      default:
        return "Shouldn't reach here";
    }
  }

  std::string result(ptr.get());

  return result;
}

#endif  // ROBOTX_BT_PLANNER__DEMANGLE_HPP_
