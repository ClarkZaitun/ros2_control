// Copyright 2025 ros2_control Development Team
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

// 硬件组件（HardwareComponent）实现文件
// 本文件实现了硬件组件的生命周期管理，包括初始化、配置、激活、停用、清理、关闭和错误恢复等状态转换。
// HardwareComponent 是对 HardwareComponentInterface 的包装类，采用 Pimpl 模式，
// 将实际的硬件接口实现委托给 impl_ 对象，同时提供互斥锁保护和统计信息收集功能。

#include "hardware_interface/hardware_component.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lifecycle_helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 硬件组件构造函数
// 接受一个硬件组件接口的唯一指针，通过移动语义存储到 impl_ 中
// impl_ 是 Pimpl 模式，将实现细节隐藏在 HardwareComponentInterface 中
HardwareComponent::HardwareComponent(std::unique_ptr<HardwareComponentInterface> impl)
: impl_(std::move(impl))
{
}

// 移动构造函数：从另一个 HardwareComponent 对象移动资源
// 注意：需要锁定源对象的互斥锁以确保线程安全
HardwareComponent::HardwareComponent(HardwareComponent && other) noexcept
{
  std::lock_guard<std::recursive_mutex> lock(other.component_mutex_);
  impl_ = std::move(other.impl_);
  last_read_cycle_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  last_write_cycle_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
}

// 初始化硬件组件
// 调用 HardwareComponentInterface::init() 执行实际的初始化逻辑
// 初始化内容包括：读取硬件参数、分配内存、设置初始状态等
// 成功初始化后硬件进入 UNCONFIGURED 状态
const rclcpp_lifecycle::State & HardwareComponent::initialize(
  const hardware_interface::HardwareComponentParams & params)
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN)
  {
    switch (impl_->init(params))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 配置硬件组件
// 执行状态转换：UNCONFIGURED → INACTIVE
// 调用 HardwareComponentInterface::on_configure()，硬件在此阶段导出状态和命令接口
// 配置成功后启用内省（introspection）功能
const rclcpp_lifecycle::State & HardwareComponent::configure()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    impl_->pause_async_operations();
    switch (impl_->on_configure(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(error());
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 清理硬件组件
// 执行状态转换：INACTIVE → UNCONFIGURED
// 调用 HardwareComponentInterface::on_cleanup()，释放配置期间分配的资源
// 清理完成后禁用内省功能
const rclcpp_lifecycle::State & HardwareComponent::cleanup()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  impl_->enable_introspection(false);
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    impl_->pause_async_operations();
    switch (impl_->on_cleanup(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(error());
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 关停硬件组件
// 执行状态转换：INACTIVE → FINALIZED
// 调用 HardwareComponentInterface::on_shutdown()
const rclcpp_lifecycle::State & HardwareComponent::shutdown()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  impl_->enable_introspection(false);
  if (
    impl_->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN &&
    impl_->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    impl_->pause_async_operations();
    switch (impl_->on_shutdown(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(error());
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 激活硬件组件
// 执行状态转换：INACTIVE → ACTIVE
// 调用 HardwareComponentInterface::on_activate()，启动硬件通信
// 激活成功后，硬件的状态和命令接口可供控制器使用
// 同时启动异步操作线程
const rclcpp_lifecycle::State & HardwareComponent::activate()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  last_read_cycle_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  last_write_cycle_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  read_statistics_.reset_statistics();
  if (impl_->get_hardware_info().type != "sensor")
  {
    write_statistics_.reset_statistics();
  }
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    impl_->pause_async_operations();
    impl_->prepare_for_activation();
    switch (impl_->on_activate(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->enable_introspection(true);
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(error());
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 停用硬件组件
// 执行状态转换：ACTIVE → INACTIVE
// 调用 HardwareComponentInterface::on_deactivate()，停止硬件通信
// 同时暂停异步操作线程
const rclcpp_lifecycle::State & HardwareComponent::deactivate()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  impl_->enable_introspection(false);
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    impl_->pause_async_operations();
    switch (impl_->on_deactivate(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE));
        break;
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(error());
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 将硬件组件置为错误状态
// 从任意状态转换到错误状态
// 调用 HardwareComponentInterface::on_error()，尝试恢复硬件
// 如果恢复成功返回 UNCONFIGURED，否则返回 ERROR
const rclcpp_lifecycle::State & HardwareComponent::error()
{
  std::unique_lock<std::recursive_mutex> lock(component_mutex_);
  impl_->enable_introspection(false);
  if (
    impl_->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN &&
    impl_->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    impl_->pause_async_operations();
    switch (impl_->on_error(impl_->get_lifecycle_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_lifecycle_state(
          rclcpp_lifecycle::State(
            lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
    }
  }
  return impl_->get_lifecycle_state();
}

// 导出状态接口：提供只读的硬件状态数据给控制器使用
// 兼容旧的 export_state_interfaces() 和新的 on_export_state_interfaces() 两种方式
std::vector<StateInterface::ConstSharedPtr> HardwareComponent::export_state_interfaces()
{
// BEGIN (Handle export change): for backward compatibility, can be removed if
// export_command_interfaces() method is removed
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<StateInterface> interfaces = impl_->export_state_interfaces();
#pragma GCC diagnostic pop
  // END: for backward compatibility

  // If no StateInterfaces has been exported, this could mean:
  // a) there is nothing to export -> on_export_state_interfaces() does return nothing as well
  // b) default implementation for export_state_interfaces() is used -> new functionality ->
  // Framework exports and creates everything
  if (interfaces.empty())
  {
    return impl_->on_export_state_interfaces();
  }

  // BEGIN (Handle export change): for backward compatibility, can be removed if
  // export_command_interfaces() method is removed
  std::vector<StateInterface::ConstSharedPtr> interface_ptrs;
  interface_ptrs.reserve(interfaces.size());
  for (auto const & interface : interfaces)
  {
    interface_ptrs.emplace_back(std::make_shared<const StateInterface>(interface));
  }
  return interface_ptrs;
  // END: for backward compatibility
}

// 导出命令接口：提供可写的硬件命令数据给控制器使用
// 兼容旧的 export_command_interfaces() 和新的 on_export_command_interfaces() 两种方式
std::vector<CommandInterface::SharedPtr> HardwareComponent::export_command_interfaces()
{
// BEGIN (Handle export change): for backward compatibility, can be removed if
// export_command_interfaces() method is removed
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<CommandInterface> interfaces = impl_->export_command_interfaces();
#pragma GCC diagnostic pop
  // END: for backward compatibility

  // If no CommandInterface has been exported, this could mean:
  // a) there is nothing to export -> on_export_command_interfaces() does return nothing as well
  // b) default implementation for export_command_interfaces() is used -> new functionality ->
  // Framework exports and creates everything
  if (interfaces.empty())
  {
    return impl_->on_export_command_interfaces();
  }
  // BEGIN (Handle export change): for backward compatibility, can be removed if
  // export_command_interfaces() method is removed
  std::vector<CommandInterface::SharedPtr> interface_ptrs;
  interface_ptrs.reserve(interfaces.size());
  for (auto & interface : interfaces)
  {
    interface_ptrs.emplace_back(std::make_shared<CommandInterface>(std::move(interface)));
  }
  return interface_ptrs;
  // END: for backward compatibility
}

// 准备命令模式切换
// 在控制器切换前调用，通知硬件即将发生的接口变化
// 调用 HardwareComponentInterface::prepare_command_mode_switch()
return_type HardwareComponent::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->prepare_command_mode_switch(start_interfaces, stop_interfaces);
}

// 执行命令模式切换
// 在控制器切换后调用，通知硬件接口变化已完成
// 调用 HardwareComponentInterface::perform_command_mode_switch()
return_type HardwareComponent::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

const std::string & HardwareComponent::get_name() const { return impl_->get_name(); }

const std::string & HardwareComponent::get_group_name() const { return impl_->get_group_name(); }

const rclcpp_lifecycle::State & HardwareComponent::get_lifecycle_state() const
{
  return impl_->get_lifecycle_state();
}

uint8_t HardwareComponent::get_lifecycle_id() const { return impl_->get_lifecycle_id(); }

const rclcpp::Time & HardwareComponent::get_last_read_time() const { return last_read_cycle_time_; }

const rclcpp::Time & HardwareComponent::get_last_write_time() const
{
  return last_write_cycle_time_;
}

const HardwareComponentStatisticsCollector & HardwareComponent::get_read_statistics() const
{
  return read_statistics_;
}

const HardwareComponentStatisticsCollector & HardwareComponent::get_write_statistics() const
{
  return write_statistics_;
}

// 读取硬件状态数据
// 从物理硬件读取传感器数据和关节状态
// 调用 HardwareComponentInterface::read()
// 使用互斥锁 (component_mutex_) 保护并发访问
// 如果读取失败（ERROR），将硬件置为错误状态
// 如果返回 DEACTIVATE，将硬件转为非活跃状态
// 同时收集执行时间和周期性的统计数据
return_type HardwareComponent::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (lifecycleStateThatRequiresNoAction(impl_->get_lifecycle_id()))
  {
    last_read_cycle_time_ = time;
    return return_type::OK;
  }
  if (
    impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    const auto trigger_result = impl_->trigger_read(time, period);
    if (trigger_result.result == return_type::ERROR)
    {
      error();
    }
    if (trigger_result.successful)
    {
      if (trigger_result.execution_time.has_value())
      {
        read_statistics_.execution_time->add_measurement(
          static_cast<double>(trigger_result.execution_time.value().count()) / 1.e3);
      }
      if (last_read_cycle_time_.get_clock_type() != RCL_CLOCK_UNINITIALIZED)
      {
        read_statistics_.periodicity->add_measurement(
          1.0 / (time - last_read_cycle_time_).seconds());
      }
      last_read_cycle_time_ = time;
    }
    return trigger_result.result;
  }
  return return_type::OK;
}

// 写入硬件命令数据
// 将控制器计算的命令值写入物理硬件
// 调用 HardwareComponentInterface::write()
// 使用互斥锁 (component_mutex_) 保护并发访问
// 如果写入失败（ERROR），将硬件置为错误状态
// 如果返回 DEACTIVATE，将硬件转为非活跃状态
// 同时收集执行时间和周期性的统计数据
return_type HardwareComponent::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (impl_->get_hardware_info().type == "sensor")
  {
    return return_type::OK;
  }

  if (lifecycleStateThatRequiresNoAction(impl_->get_lifecycle_id()))
  {
    last_write_cycle_time_ = time;
    return return_type::OK;
  }
  // only call write in the active state
  if (impl_->get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    const auto trigger_result = impl_->trigger_write(time, period);
    if (trigger_result.result == return_type::ERROR)
    {
      error();
    }
    if (trigger_result.successful)
    {
      if (trigger_result.execution_time.has_value())
      {
        write_statistics_.execution_time->add_measurement(
          static_cast<double>(trigger_result.execution_time.value().count()) / 1.e3);
      }
      if (last_write_cycle_time_.get_clock_type() != RCL_CLOCK_UNINITIALIZED)
      {
        write_statistics_.periodicity->add_measurement(
          1.0 / (time - last_write_cycle_time_).seconds());
      }
      last_write_cycle_time_ = time;
    }
    return trigger_result.result;
  }
  return return_type::OK;
}

// 获取组件互斥锁的引用，供外部进行线程同步
std::recursive_mutex & HardwareComponent::get_mutex() { return component_mutex_; }
}  // namespace hardware_interface
