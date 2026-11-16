/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__TOOLS__CONFIG_HPP
#define INFO_RELAY__TOOLS__CONFIG_HPP

/*
 * 中转进程配置：通信链路 / 解析策略 / 跨进程主题 / 日志四段。
 *
 * 数据流：configs/info_relay.yaml → RelayConfig::load() → 各子系统按段取值。
 * 依赖与边界：只描述本进程参数，不含相机与算法配置；路径字段加载时即解析为绝对路径。
 * 使用注意：缺必需键（com_port 或 can_interface）抛 std::runtime_error。
 */

#include <cstdint>
#include <string>

#include <yaml-cpp/yaml.h>

#include "IO/inc/IO.hpp"

namespace info_relay
{

// 协议解析与下位机交互参数
struct BoardSection
{
  double bullet_speed = 28.0;             // 默认弹速，m/s
  bool use_referee_system = false;        // 是否依赖裁判系统判定开赛
  int game_start_progress_threshold = 4;  // 比赛中对应的 game_progress
  int heat_stop_margin = 10;              // 热量保护提前量
  bool imu_on_upper_axis = true;          // IMU 是否装在小 yaw 轴
};

// 跨进程通信
struct RelaySection
{
  std::string prefix = "rm_vision";  // 主题前缀，实际主题为 "<prefix>/<topic>"
  int timeout_ms = 200;              // 无上行帧判定链路失效的阈值，ms
};

// 日志
struct LogSection
{
  std::string dir;  // 空串表示使用 <home>/logs
};

// 完整配置
struct RelayConfig
{
  io::IoOptions io;             // 通信方式参数
  BoardSection board;           // 协议解析参数
  RelaySection relay;           // 跨进程参数
  LogSection log;               // 日志参数

  // 从 YAML 文件加载；文件不存在或语法错误时抛异常
  static RelayConfig load(const std::string & path);

  // 从已解析的 YAML 节点加载，供工具直接喂节点
  static RelayConfig load(const YAML::Node & yaml);
};

}  // namespace info_relay

#endif  // INFO_RELAY__TOOLS__CONFIG_HPP
