/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__CAN__PROTOCOL__CRC_HPP
#define INFO_RELAY__IO__CAN__PROTOCOL__CRC_HPP

#include <cstddef>
#include <cstdint>

namespace info_relay::io::can::protocol
{

// CAN 分片校验：CRC-8（多项式 0x07，初值 0）
uint8_t crc8(const uint8_t * data, size_t size);

}  // namespace info_relay::io::can::protocol

#endif  // INFO_RELAY__IO__CAN__PROTOCOL__CRC_HPP
