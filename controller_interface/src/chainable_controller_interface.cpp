// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

// 可链式控制器接口（ChainableControllerInterface）实现文件
// 本文件实现了可链式调用的控制器接口，允许控制器导出状态和参考接口供其他控制器使用。
// 主要功能：控制器链式调用、导出状态/参考接口、管理链式控制器之间的数据传递。

#include "controller_interface/chainable_controller_interface.hpp"

#include <fmt/compile.h>

#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
// 可链式控制器构造函数
// 初始化链式模式标志为 false
ChainableControllerInterface::ChainableControllerInterface() : ControllerInterfaceBase() {}

// 可链式控制器始终返回 true
bool ChainableControllerInterface::is_chainable() const { return true; }

// 链式控制器更新函数
// 与标准控制器不同，链式控制器在更新后需要：
// 1. 执行控制算法计算
// 2. 更新导出的参考接口值（供后续控制器使用）
// 3. 更新导出的状态接口值（供其他控制器读取）
return_type ChainableControllerInterface::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return_type ret = return_type::ERROR;

  if (!is_in_chained_mode())
  {
    ret = update_reference_from_subscribers(time, period);
    if (ret != return_type::OK)
    {
      return ret;
    }
  }

  ret = update_and_write_commands(time, period);

  return ret;
}

// 导出链式控制器的状态接口
// 调用 on_export_state_interfaces_list() 获取接口描述
// 为每个接口创建 StateInterface 并注册到接口映射中
// 其他控制器可以通过这些接口读取本控制器的内部状态
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ChainableControllerInterface::export_state_interfaces()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto state_interfaces = on_export_state_interfaces();
#pragma GCC diagnostic pop
  const auto state_interfaces_list = on_export_state_interfaces_list();
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_ptrs_vec;
  state_interfaces_ptrs_vec.reserve(state_interfaces.size() + state_interfaces_list.size());
  ordered_exported_state_interfaces_.reserve(
    state_interfaces.size() + state_interfaces_list.size());
  exported_state_interface_names_.reserve(state_interfaces.size() + state_interfaces_list.size());
  exported_state_interfaces_.clear();
  exported_state_interface_names_.clear();
  ordered_exported_state_interfaces_.clear();

  // check if the names of the controller state interfaces begin with the controller's name
  for (const auto & interface : state_interfaces)
  {
    if (interface.get_prefix_name().find(get_node()->get_name()) != 0)
    {
      const std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for state interfaces. No state interface will be exported. "
          "Please correct and recompile the controller with name '{}' and try again."),
        interface.get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }
    auto state_interface = std::make_shared<hardware_interface::StateInterface>(interface);
    const auto interface_name = state_interface->get_name();
    auto [it, succ] = exported_state_interfaces_.insert({interface_name, state_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert StateInterface<{}> into exported_state_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      exported_state_interfaces_.clear();
      exported_state_interface_names_.clear();
      state_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_state_interfaces_, state_interface);
    ros2_control::add_item(exported_state_interface_names_, interface_name);
    state_interfaces_ptrs_vec.push_back(
      std::const_pointer_cast<const hardware_interface::StateInterface>(state_interface));
  }

  // New API
  for (const auto & interface_ptr : state_interfaces_list)
  {
    if (interface_ptr->get_prefix_name().find(get_node()->get_name()) != 0)
    {
      const std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for state interfaces. No state interface will be exported. "
          "Please correct and recompile the controller with name '{}' and try again."),
        interface_ptr->get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }
    const auto interface_name = interface_ptr->get_name();
    auto [it, succ] = exported_state_interfaces_.insert({interface_name, interface_ptr});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert StateInterface<{}> into exported_state_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      exported_state_interfaces_.clear();
      exported_state_interface_names_.clear();
      state_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_state_interfaces_, interface_ptr);
    ros2_control::add_item(exported_state_interface_names_, interface_name);
    state_interfaces_ptrs_vec.push_back(interface_ptr);
    ;
  }

  const auto total_state_interfaces = state_interfaces.size() + state_interfaces_list.size();
  if (exported_state_interfaces_.size() != total_state_interfaces)
  {
    std::string error_msg = fmt::format(
      FMT_COMPILE(
        "The internal storage for state interface ptrs 'exported_state_interfaces_' variable has "
        "size '{}', but it is expected to have the size '{}' equal to the number of exported "
        "reference interfaces. Please correct and recompile the controller with name '{}' and try "
        "again."),
      exported_state_interfaces_.size(), total_state_interfaces, get_node()->get_name());
    throw std::runtime_error(error_msg);
  }

  return state_interfaces_ptrs_vec;
}

// 导出链式控制器的参考接口
// 调用 on_export_reference_interfaces_list() 获取接口描述
// 为每个接口创建 CommandInterface 并注册到接口映射中
// 前一个控制器可以通过这些接口写入参考值给本控制器
std::vector<hardware_interface::CommandInterface::SharedPtr>
ChainableControllerInterface::export_reference_interfaces()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto reference_interfaces = on_export_reference_interfaces();
#pragma GCC diagnostic pop
  const auto reference_interfaces_list = on_export_reference_interfaces_list();
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces_ptrs_vec;
  reference_interfaces_ptrs_vec.reserve(
    reference_interfaces.size() + reference_interfaces_list.size());
  exported_reference_interface_names_.reserve(
    reference_interfaces.size() + reference_interfaces_list.size());
  ordered_exported_reference_interfaces_.reserve(
    reference_interfaces.size() + reference_interfaces_list.size());
  exported_reference_interfaces_.clear();
  exported_reference_interface_names_.clear();
  ordered_exported_reference_interfaces_.clear();

  // BEGIN (Handle export change): for backward compatibility
  // check if the "reference_interfaces_" variable is resized to number of interfaces
  if (reference_interfaces_.size() != reference_interfaces.size())
  {
    std::string error_msg = fmt::format(
      FMT_COMPILE(
        "The internal storage for reference values 'reference_interfaces_' variable has size '{}', "
        "but it is expected to have the size '{}' equal to the number of exported reference "
        "interfaces. Please correct and recompile the controller with name '{}' and try again."),
      reference_interfaces_.size(), reference_interfaces.size(), get_node()->get_name());
    throw std::runtime_error(error_msg);
  }
  // END

  // check if the names of the reference interfaces begin with the controller's name
  for (auto & interface : reference_interfaces)
  {
    if (interface.get_prefix_name().find(get_node()->get_name()) != 0)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for reference interfaces. Please correct and recompile the "
          "controller with name '{}' and try again."),
        interface.get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }

    hardware_interface::CommandInterface::SharedPtr reference_interface =
      std::make_shared<hardware_interface::CommandInterface>(std::move(interface));
    const auto interface_name = reference_interface->get_name();
    // check the exported interface name is unique
    auto [it, succ] = exported_reference_interfaces_.insert({interface_name, reference_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert Reference interface<{}> into reference_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      reference_interfaces_.clear();
      exported_reference_interface_names_.clear();
      reference_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_reference_interfaces_, reference_interface);
    ros2_control::add_item(exported_reference_interface_names_, interface_name);
    reference_interfaces_ptrs_vec.push_back(reference_interface);
  }

  // new API
  for (const auto & interface_ptr : reference_interfaces_list)
  {
    if (interface_ptr->get_prefix_name().find(get_node()->get_name()) != 0)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for reference interfaces. Please correct and recompile the "
          "controller with name '{}' and try again."),
        interface_ptr->get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }

    const auto interface_name = interface_ptr->get_name();
    // check the exported interface name is unique
    auto [it, succ] = exported_reference_interfaces_.insert({interface_name, interface_ptr});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert Reference interface<{}> into reference_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      reference_interfaces_.clear();
      exported_reference_interface_names_.clear();
      reference_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_reference_interfaces_, interface_ptr);
    ros2_control::add_item(exported_reference_interface_names_, interface_name);
    reference_interfaces_ptrs_vec.push_back(interface_ptr);
  }

  const auto total_ref_interfaces = reference_interfaces.size() + reference_interfaces_list.size();
  if (exported_reference_interfaces_.size() != total_ref_interfaces)
  {
    std::string error_msg = fmt::format(
      FMT_COMPILE(
        "The internal storage for exported reference ptrs 'exported_reference_interfaces_' "
        "variable has size '{}', but it is expected to have the size '{}' equal to the number of "
        "exported reference interfaces. Please correct and recompile the controller with name '{}' "
        "and try again."),
      exported_reference_interfaces_.size(), total_ref_interfaces, get_node()->get_name());
    throw std::runtime_error(error_msg);
  }

  return reference_interfaces_ptrs_vec;
}

// 设置链式调用模式
// 当 chained_mode=true 时，控制器的参考值来自前一个控制器的输出
// 调用 on_set_chained_mode() 通知子类链式模式变化
bool ChainableControllerInterface::set_chained_mode(bool chained_mode)
{
  bool result = false;

  if (get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    result = on_set_chained_mode(chained_mode);

    if (result)
    {
      in_chained_mode_ = chained_mode;
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Can not change controller's chained mode because it is no in '%s' state. "
      "Current state is '%s'.",
      hardware_interface::lifecycle_state_names::UNCONFIGURED,
      get_lifecycle_state().label().c_str());
  }

  return result;
}

// 检查控制器是否处于链式调用模式
bool ChainableControllerInterface::is_in_chained_mode() const { return in_chained_mode_; }

// 链式模式变化回调（虚函数）
// 子类可重写此方法在链式模式变化时执行特定操作
// 默认实现返回 true（接受模式变化）
bool ChainableControllerInterface::on_set_chained_mode(bool /*chained_mode*/) { return true; }

// 导出状态接口回调（虚函数）
// 子类必须重写此方法声明要导出的状态接口
// 默认实现返回空列表
std::vector<hardware_interface::StateInterface>
ChainableControllerInterface::on_export_state_interfaces()
{
  state_interfaces_values_.resize(exported_state_interface_names_.size(), 0.0);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i]);
  }
  return state_interfaces;
}

// 导出状态接口列表回调（虚函数）
// 返回状态接口描述列表
std::vector<hardware_interface::StateInterface::SharedPtr>
ChainableControllerInterface::on_export_state_interfaces_list()
{
  // return empty vector by default.
  return {};
}

// 导出参考接口回调（虚函数）
// 子类必须重写此方法声明要导出的参考接口
// 默认实现返回空列表
std::vector<hardware_interface::CommandInterface>
ChainableControllerInterface::on_export_reference_interfaces()
{
  reference_interfaces_.resize(exported_reference_interface_names_.size(), 0.0);
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  for (size_t i = 0; i < exported_reference_interface_names_.size(); ++i)
  {
    reference_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), exported_reference_interface_names_[i], &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

// 导出参考接口列表回调（虚函数）
// 返回参考接口描述列表
std::vector<hardware_interface::CommandInterface::SharedPtr>
ChainableControllerInterface::on_export_reference_interfaces_list()
{
  // return empty vector by default.
  return {};
}

}  // namespace controller_interface
