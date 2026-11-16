/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/can/protocol/crc.hpp"

namespace info_relay::io::can::protocol
{

uint8_t crc8(const uint8_t * data, size_t size)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace info_relay::io::can::protocol
