// Copyright 2020 - 2021 ros2_control Development Team
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

// 系统（System）头文件
// 定义了系统组件类，是对 SystemInterface 的包装
// 继承自 HardwareComponent，提供系统级硬件的生命周期管理

#ifndef HARDWARE_INTERFACE__SYSTEM_HPP_
#define HARDWARE_INTERFACE__SYSTEM_HPP_

#include "hardware_interface/hardware_component.hpp"
#include "hardware_interface/system_interface.hpp"

namespace hardware_interface
{
using System = HardwareComponent;
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_HPP_
