/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/serial/protocol/uplink_host.hpp"

#include <cstring>

namespace host_comm
{

template <typename T>
bool Protocol::Assign(const referee_comm::Frame & frame, T & target, size_t expected)
{
  if (expected == 0 || expected > sizeof(T) || frame.length != expected) return false;
  std::memcpy(&target, frame.data, expected);
  return true;
}

bool Protocol::OnCommand(const referee_comm::Frame & frame)
{
  if (frame.data == nullptr) return false;

  switch (frame.cmd_id) {
    case HOST_VERSION: {
      if (!Assign(frame, version, sizeof(host_version_t))) return false;
      version_valid_ = true;
      return true;
    }
    case ROBOT_STATUS:
      return Assign(frame, robot, sizeof(host_robot_status_t));
    case CHASSIS_STATUS:
      return Assign(frame, chassis, sizeof(host_chassis_status_t));
    case GIMBAL_STATUS: {
      if (!Assign(frame, gimbal, sizeof(host_gimbal_status_t))) return false;
      gimbal_count_++;
      return true;
    }
    case SHOOT_STATUS:
      return Assign(frame, shoot, sizeof(host_shoot_status_t));
    case HOST_LOG:
      return Assign(frame, log, sizeof(host_log_t));
    default:
      /* 未知或已废弃命令：静默忽略，不影响字节流同步 */
      return false;
  }
}

}  // namespace host_comm
