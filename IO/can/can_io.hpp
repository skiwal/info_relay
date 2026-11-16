/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__CAN__CAN_IO_HPP
#define INFO_RELAY__IO__CAN__CAN_IO_HPP

#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include "IO/can/protocol/frame.hpp"
#include "IO/inc/IO.hpp"

namespace info_relay::io
{

// SocketCAN 实现：把字节流分片成标准 CAN 帧，反向重组后交给统一 IO 接口
class CanIo final : public IO
{
public:
  ~CanIo() override;

  bool init(const IoOptions & options) override;
  bool open() override;
  size_t read(uint8_t * buffer, size_t size, int timeout_ms) override;
  size_t write(const uint8_t * data, size_t size) override;
  void close() override;
  std::string describe() const override;

private:
  bool consume_frame(const can::protocol::CanFrame & frame);

  IoOptions options_;
  int fd_ = -1;
  uint8_t tx_seq_ = 0;
  uint8_t last_rx_seq_ = 0xFF;
  std::vector<uint8_t> rx_bytes_;
  std::map<uint8_t, std::map<uint8_t, can::protocol::CanChunk>> pending_;
  mutable std::mutex rx_mutex_;
  mutable std::mutex tx_mutex_;
  bool opened_ = false;
};

}  // namespace info_relay::io

#endif  // INFO_RELAY__IO__CAN__CAN_IO_HPP
