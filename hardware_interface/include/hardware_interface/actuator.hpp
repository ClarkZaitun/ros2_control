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

// 执行器（Actuator）头文件
// 定义了执行器组件类，是对 ActuatorInterface 的包装
// 继承自 HardwareComponent，提供执行器专用的生命周期管理

#ifndef HARDWARE_INTERFACE__ACTUATOR_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_component.hpp"

namespace hardware_interface
{
using Actuator = HardwareComponent;
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_HPP_
