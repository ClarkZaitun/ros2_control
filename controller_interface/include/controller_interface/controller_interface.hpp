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

// 控制器接口（ControllerInterface）头文件
// 定义了标准控制器接口类，是 ros2_control 中所有控制器的基类
// 提供控制器与硬件命令/状态接口交互的标准框架

#ifndef CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/handle.hpp"

namespace controller_interface
{
class ControllerInterface : public controller_interface::ControllerInterfaceBase
{
public:
  ControllerInterface();

  virtual ~ControllerInterface() = default;

  /**
   * Controller is not chainable.
   *
   * \returns false.
   */
  bool is_chainable() const final;

  /**
   * A non-chainable controller doesn't export any state interfaces.
   *
   * \returns empty list.
   */
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> export_state_interfaces() final;

  /**
   * Controller has no reference interfaces.
   *
   * \returns empty list.
   */
  std::vector<hardware_interface::CommandInterface::SharedPtr> export_reference_interfaces() final;

  /**
   * Controller is not chainable, therefore no chained mode can be set.
   *
   * \returns false.
   */
  bool set_chained_mode(bool chained_mode) final;

  /**
   * Controller can not be in chained mode.
   *
   * \returns false.
   */
  bool is_in_chained_mode() const final;
};

using ControllerInterfaceSharedPtr = std::shared_ptr<ControllerInterface>;

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_
