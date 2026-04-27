// Copyright 2024 ros2_control Development Team
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

// 词法转换工具实现文件
// 提供与区域设置无关的字符串到数值的转换函数
// 包括：stod（字符串转double）、stof（字符串转float）、to_lower_case、parse_bool、parse_array 等
// 这些函数避免了标准库 stod/stof 受本地化设置影响的问题

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"

namespace hardware_interface
{
namespace impl
{
// 使用 std::from_chars 解析浮点数（C++17+），不受区域设置影响
template <typename FloatingPointType>
std::optional<FloatingPointType> parse_floating_point_with_from_chars(const std::string & s)
{
  const char * begin = s.data();
  const char * end = s.data() + s.size();

  if (begin == end)
  {
    return std::nullopt;
  }

  // std::from_chars for floating-point does not accept a leading '+' sign.
  if (*begin == '+')
  {
    ++begin;
    if (begin == end)
    {
      return std::nullopt;
    }
  }

  FloatingPointType result_value;
  const auto parse_result = std::from_chars(begin, end, result_value);
  if (
    parse_result.ec == std::errc() && parse_result.ptr == end &&
    std::isfinite(static_cast<double>(result_value)))
  {
    return result_value;
  }

  return std::nullopt;
}

// 字符串转 double（可选返回版本），内部使用不含区域设置的转换
std::optional<double> stod(const std::string & s)
{
#if __cplusplus < 202002L
  // convert from string using no locale
  // Impl with std::istringstream
  std::istringstream stream(s);
  stream.imbue(std::locale::classic());
  double result;
  stream >> result;
  if (stream.fail() || !stream.eof() || !std::isfinite(result))
  {
    return std::nullopt;
  }
  return result;
#else
  // Impl with std::from_chars
  return parse_floating_point_with_from_chars<double>(s);
#endif
}

// 字符串转 float（可选返回版本）
std::optional<float> stof(const std::string & s)
{
#if __cplusplus < 202002L
  // convert from string using no locale
  // Impl with std::istringstream
  std::istringstream stream(s);
  stream.imbue(std::locale::classic());
  float result;
  stream >> result;
  if (stream.fail() || !stream.eof() || !std::isfinite(result))
  {
    return std::nullopt;
  }
  return result;
#else
  // Impl with std::from_chars
  return parse_floating_point_with_from_chars<float>(s);
#endif
}

}  // namespace impl

// 字符串转 double（抛出异常版本）：转换失败时抛出 std::invalid_argument
double stod(const std::string & s)
{
  if (const auto result = impl::stod(s))
  {
    return *result;
  }
  throw std::invalid_argument("Failed converting string to real number");
}

// 字符串转 float（抛出异常版本）：转换失败时抛出 std::invalid_argument
float stof(const std::string & s)
{
  if (const auto result = impl::stof(s))
  {
    return *result;
  }
  throw std::invalid_argument("Failed converting string to float number");
}

// 将字符串转换为小写
std::string to_lower_case(const std::string & string)
{
  std::string lower_case_string = string;
  std::transform(
    lower_case_string.begin(), lower_case_string.end(), lower_case_string.begin(),
    [](unsigned char c) { return std::tolower(c); });
  return lower_case_string;
}

// 解析布尔字符串：接受 "true" 或 "false"（不区分大小写）
bool parse_bool(const std::string & bool_string)
{
  // Copy input to temp and make lowercase
  std::string temp = to_lower_case(bool_string);

  if (temp == "true")
  {
    return true;
  }
  if (temp == "false")
  {
    return false;
  }
  // If input is not "true" or "false" (any casing), throw or handle as error
  throw std::invalid_argument(
    "Input string : '" + bool_string +
    "' is not a valid boolean value. Expected 'true' or 'false'.");
}

// 解析字符串数组
std::vector<std::string> parse_string_array(const std::string & string_array_string)
{
  return parse_array<std::string>(string_array_string);
}

}  // namespace hardware_interface
