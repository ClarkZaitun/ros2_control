// Copyright 2024 PAL Robotics S.L.
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

/// \author Adrià Roig Moreno

// 关节限位辅助函数实现文件
// 本文件提供了关节限位计算中使用的辅助函数，包括限位值的裁剪和边界检查等。

#include "joint_limits/joint_limits_helpers.hpp"

#include <fmt/compile.h>

#include <algorithm>
#include <cmath>

#include "rclcpp/logging.hpp"

namespace joint_limits
{
namespace internal
{
/**
 * @brief 检查限位上下限顺序，若下限大于上限则交换两者
 * 确保限位值始终满足 lower_limit <= upper_limit 的约束
 * @param lower_limit 限位下限引用，可能被修改
 * @param upper_limit 限位上限引用，可能被修改
 */
void check_and_swap_limits(double & lower_limit, double & upper_limit)
{
  if (lower_limit > upper_limit)
  {
    std::swap(lower_limit, upper_limit);
  }
}

/**
 * @brief 验证实际位置是否在关节限位范围内，若超出则记录错误并抛出异常
 * @param joint_name 关节名称，用于错误日志输出
 * @param actual_position 关节的实际位置值
 * @param limits 关节限位参数，包含位置上下限
 * @throws std::runtime_error 当实际位置超出限位范围时抛出异常
 */
void verify_actual_position_within_limits(
  const std::string & joint_name, const std::optional<double> & actual_position,
  const joint_limits::JointLimits & limits)
{
  if (actual_position.has_value() && limits.has_position_limits)
  {
    const double actual_pos = actual_position.value();
    if (
      actual_pos > (limits.max_position + internal::OUT_OF_BOUNDS_EXCEPTION_TOLERANCE) ||
      actual_pos < (limits.min_position - internal::OUT_OF_BOUNDS_EXCEPTION_TOLERANCE))
    {
      const std::string error_message = fmt::format(
        FMT_COMPILE(
          "Joint position is out of bounds for the joint : '{}' actual position: {} limits: [{}, "
          "{}]. This could be due to a hardware failure (or) the physical limits of the joint "
          "being larger than the ones defined in the URDF. Please recheck the URDF and the "
          "hardware to verify the joint limits."),
        joint_name, actual_pos, limits.min_position, limits.max_position);
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("joint_limiter_interface"), "%s", error_message.c_str());
      // Throw an exception to indicate that the joint position is out of bounds
      throw std::runtime_error(error_message);
    }
  }
}
}  // namespace internal

// 更新上一命令缓存
// 将当前期望命令值（desired）中的各控制量复制到上一命令缓存（prev_command）中
// 用于下一周期限位计算时作为参考基准
// @param desired 当前期望的关节控制接口数据
// @param prev_command 上一命令缓存，将被更新
void update_prev_command(
  const JointControlInterfacesData & desired, JointControlInterfacesData & prev_command)
{
  if (desired.has_position())
  {
    prev_command.position = desired.position;
  }
  if (desired.has_velocity())
  {
    prev_command.velocity = desired.velocity;
  }
  if (desired.has_effort())
  {
    prev_command.effort = desired.effort;
  }
  if (desired.has_acceleration())
  {
    prev_command.acceleration = desired.acceleration;
  }
  if (desired.has_jerk())
  {
    prev_command.jerk = desired.jerk;
  }
  prev_command.joint_name = desired.joint_name;
}

// 判断值是否超出指定范围（被限位）
// @param value 待检查的值
// @param min 允许的最小值
// @param max 允许的最大值
// @return true 如果值小于最小值或大于最大值，表示需要限位
bool is_limited(double value, double min, double max) { return value < min || value > max; }

// 计算位置限位边界
// 综合考虑关节的位置硬限位和速度/加速度约束，计算当前周期允许的位置命令范围
// 使用上一命令位置作为参考基准（比实际位置更少保守），避免因通信延迟导致过度限位
// @param joint_name 关节名称，用于日志输出
// @param limits 关节限位参数
// @param act_vel 实际速度值
// @param act_pos 实际位置值
// @param prev_command_pos 上一命令位置值
// @param dt 时间步长（秒）
// @return PositionLimits 包含位置下限和上限的结构体
PositionLimits compute_position_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const std::optional<double> & act_vel, const std::optional<double> & act_pos,
  const std::optional<double> & prev_command_pos, double dt)
{
  PositionLimits pos_limits(limits.min_position, limits.max_position);
  internal::verify_actual_position_within_limits(joint_name, act_pos, limits);
  if (limits.has_velocity_limits)
  {
    const double act_vel_abs = act_vel.has_value() ? std::fabs(act_vel.value()) : 0.0;
    const double delta_vel = limits.has_acceleration_limits
                               ? act_vel_abs + (limits.max_acceleration * dt)
                               : limits.max_velocity;
    const double max_vel = std::min(limits.max_velocity, delta_vel);
    const double delta_pos = max_vel * dt;
    /// @note: We use the previous command position to compute the limits here because using the
    /// actual position would be too conservative, usually there is a couple of cycles of delay
    /// between the command sent to the robot and the robot actually showing that in the state. That
    /// effectively limits the velocity with which the joint can be moved which is much lower than
    /// the actual velocity limit.
    const double position_reference = prev_command_pos.value();
    pos_limits.lower_limit = std::max(
      std::min(position_reference - delta_pos, pos_limits.upper_limit), pos_limits.lower_limit);
    pos_limits.upper_limit = std::min(
      std::max(position_reference + delta_pos, pos_limits.lower_limit), pos_limits.upper_limit);
  }
  internal::check_and_swap_limits(pos_limits.lower_limit, pos_limits.upper_limit);
  return pos_limits;
}

// 计算速度限位边界
// 综合考虑关节的速度硬限位、位置限位和加速度约束，计算当前周期允许的速度命令范围
// 当实际位置超出限位时，将速度范围限制为零以防止进一步越界
// @param joint_name 关节名称，用于日志输出
// @param limits 关节限位参数
// @param desired_vel 期望速度值
// @param act_pos 实际位置值
// @param prev_command_vel 上一命令速度值
// @param dt 时间步长（秒）
// @return VelocityLimits 包含速度下限和上限的结构体
VelocityLimits compute_velocity_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const double & desired_vel, const std::optional<double> & act_pos,
  const std::optional<double> & prev_command_vel, double dt)
{
  const double max_vel =
    limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::infinity();
  VelocityLimits vel_limits(-max_vel, max_vel);
  if (limits.has_position_limits && act_pos.has_value())
  {
    const double actual_pos = act_pos.value();
    const double max_vel_with_pos_limits = (limits.max_position - actual_pos) / dt;
    const double min_vel_with_pos_limits = (limits.min_position - actual_pos) / dt;
    vel_limits.lower_limit = std::max(min_vel_with_pos_limits, vel_limits.lower_limit);
    vel_limits.upper_limit = std::min(max_vel_with_pos_limits, vel_limits.upper_limit);

    if (actual_pos > limits.max_position || actual_pos < limits.min_position)
    {
      if (
        (actual_pos < (limits.max_position + internal::POSITION_BOUNDS_TOLERANCE) &&
         (actual_pos > limits.min_position) && desired_vel >= 0.0) ||
        (actual_pos > (limits.min_position - internal::POSITION_BOUNDS_TOLERANCE) &&
         (actual_pos < limits.max_position) && desired_vel <= 0.0))
      {
        RCLCPP_WARN_EXPRESSION(
          rclcpp::get_logger("joint_limiter_interface"),
          prev_command_vel.has_value() && prev_command_vel.value() != 0.0,
          "Joint position %.5f is out of bounds[%.5f, %.5f] for the joint and we want to move "
          "further into bounds with vel %.5f: '%s'. Joint velocity limits will be "
          "restrictred to zero.",
          actual_pos, limits.min_position, limits.max_position, desired_vel, joint_name.c_str());
        vel_limits = VelocityLimits(0.0, 0.0);
      }
      // If the joint reports a position way out of bounds, then it would mean something is
      // extremely wrong, so no velocity command should be allowed as it might damage the robot
      else if (
        (actual_pos > (limits.max_position + internal::POSITION_BOUNDS_TOLERANCE)) ||
        (actual_pos < (limits.min_position - internal::POSITION_BOUNDS_TOLERANCE)))
      {
        RCLCPP_ERROR_ONCE(
          rclcpp::get_logger("joint_limiter_interface"),
          "Joint position is out of bounds for the joint : '%s'. Joint velocity limits will be "
          "restricted to zero.",
          joint_name.c_str());
        vel_limits = VelocityLimits(0.0, 0.0);
      }
    }
  }
  if (limits.has_acceleration_limits && prev_command_vel.has_value())
  {
    const double delta_vel = limits.max_acceleration * dt;
    vel_limits.lower_limit = std::max(prev_command_vel.value() - delta_vel, vel_limits.lower_limit);
    vel_limits.upper_limit = std::min(prev_command_vel.value() + delta_vel, vel_limits.upper_limit);
  }
  internal::check_and_swap_limits(vel_limits.lower_limit, vel_limits.upper_limit);
  return vel_limits;
}

// 计算力矩（effort）限位边界
// 综合考虑关节的力矩硬限位、位置限位和速度限位，计算当前周期允许的力矩命令范围
// 当关节位于位置限位边界且向限位方向运动时，将对应方向的力矩限制为零
// 当关节速度超出速度限位时，将对应方向的力矩限制为零
// @param limits 关节限位参数
// @param act_pos 实际位置值
// @param act_vel 实际速度值
// @param dt 时间步长（秒），当前未使用
// @return EffortLimits 包含力矩下限和上限的结构体
EffortLimits compute_effort_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_pos,
  const std::optional<double> & act_vel, double /*dt*/)
{
  const double max_effort =
    limits.has_effort_limits ? limits.max_effort : std::numeric_limits<double>::infinity();
  EffortLimits eff_limits(-max_effort, max_effort);
  if (limits.has_position_limits && act_pos.has_value() && act_vel.has_value())
  {
    if ((act_pos.value() <= limits.min_position) && (act_vel.value() <= 0.0))
    {
      eff_limits.lower_limit = 0.0;
    }
    else if ((act_pos.value() >= limits.max_position) && (act_vel.value() >= 0.0))
    {
      eff_limits.upper_limit = 0.0;
    }
  }
  if (limits.has_velocity_limits && act_vel.has_value())
  {
    if (act_vel.value() < -limits.max_velocity)
    {
      eff_limits.lower_limit = 0.0;
    }
    else if (act_vel.value() > limits.max_velocity)
    {
      eff_limits.upper_limit = 0.0;
    }
  }
  internal::check_and_swap_limits(eff_limits.lower_limit, eff_limits.upper_limit);
  return eff_limits;
}

// 计算加速度限位边界
// 根据期望加速度方向和当前速度方向判断是加速还是减速
// 减速时使用减速度限位（若有），否则使用加速度限位
// @param limits 关节限位参数
// @param desired_acceleration 期望加速度值
// @param actual_velocity 实际速度值，用于判断加速/减速方向
// @return AccelerationLimits 包含加速度下限和上限的结构体
AccelerationLimits compute_acceleration_limits(
  const joint_limits::JointLimits & limits, double desired_acceleration,
  std::optional<double> actual_velocity)
{
  AccelerationLimits acc_or_dec_limits(
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  if (
    limits.has_deceleration_limits &&
    ((desired_acceleration < 0 && actual_velocity && actual_velocity.value() > 0) ||
     (desired_acceleration > 0 && actual_velocity && actual_velocity.value() < 0)))
  {
    acc_or_dec_limits.lower_limit = -limits.max_deceleration;
    acc_or_dec_limits.upper_limit = limits.max_deceleration;
  }
  else if (limits.has_acceleration_limits)
  {
    acc_or_dec_limits.lower_limit = -limits.max_acceleration;
    acc_or_dec_limits.upper_limit = limits.max_acceleration;
  }
  internal::check_and_swap_limits(acc_or_dec_limits.lower_limit, acc_or_dec_limits.upper_limit);
  return acc_or_dec_limits;
}

}  // namespace joint_limits
