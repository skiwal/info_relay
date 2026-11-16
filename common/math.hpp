/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__COMMON__MATH_HPP
#define INFO_RELAY__COMMON__MATH_HPP

/*
 * 极少量角度工具：中转进程只需要把角度差折到最短弧。
 *
 * 依赖与边界：纯头文件、纯标准库，刻意不引入 Eigen 或 OpenCV。
 *
 * 使用注意：本文件的 π 是自有常量，不用 M_PI（后者在严格 ISO 模式下不可移植），
 *           也不用 CV_PI（会把 OpenCV 依赖带进来）。
 */

namespace info_relay
{

// 圆周率，双精度足够覆盖本项目全部角度运算
inline constexpr double kPi = 3.14159265358979323846;

// 功能：把角度折算到 (-π, π]
// 输入：
//   angle  任意弧度值，允许超出 ±2π 多个周期
// 输出：等价角度，范围 (-π, π]
// 备注：用减法循环而不是 fmod，保证边界处结果确定：π 映射为 π，-π 映射为 π
inline double limit_rad(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__MATH_HPP
