/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__CAN__PROTOCOL__UPLINK_HPP
#define INFO_RELAY__IO__CAN__PROTOCOL__UPLINK_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace info_relay::io::can::protocol
{

// CAN 上行读数据结构体：分片重组后的完整字节流
struct CanUplinkMessage
{
  uint8_t seq = 0;                       // 来源报文序号
  std::array<uint8_t, 320> payload{};    // 最大帧长与 RM 线格式一致
  size_t size = 0;                       // 有效字节数
};

}  // namespace info_relay::io::can::protocol

#endif  // INFO_RELAY__IO__CAN__PROTOCOL__UPLINK_HPP
