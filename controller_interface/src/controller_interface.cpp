// Copyright 2017 Open Source Robotics Foundation, Inc.
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

// 控制器接口（ControllerInterface）实现文件
// 本文件实现了标准控制器接口，是 ros2_control 中控制器的基类。
// 主要功能：提供控制器与硬件接口交互的标准框架，默认不可链式调用。

#include "controller_interface/controller_interface.hpp"

#include <vector>

namespace controller_interface
{
ControllerInterface::ControllerInterface() : ControllerInterfaceBase() {}

bool ControllerInterface::is_chainable() const { return false; }

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ControllerInterface::export_state_interfaces()
{
  return {};
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ControllerInterface::export_reference_interfaces()
{
  return {};
}

bool ControllerInterface::set_chained_mode(bool /*chained_mode*/) { return false; }

bool ControllerInterface::is_in_chained_mode() const { return false; }

}  // namespace controller_interface
