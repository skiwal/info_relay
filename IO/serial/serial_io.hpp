/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__SERIAL__SERIAL_IO_HPP
#define INFO_RELAY__IO__SERIAL__SERIAL_IO_HPP

#include <mutex>
#include <string>

#include "IO/inc/IO.hpp"
#include "serial/serial.h"

namespace info_relay::io
{

// 真实串口实现：把 3rdparty/serial 包装成统一的 IO 接口
class SerialIo final : public IO
{
public:
  ~SerialIo() override;

  bool init(const IoOptions & options) override;
  bool open() override;
  size_t read(uint8_t * buffer, size_t size, int timeout_ms) override;
  size_t write(const uint8_t * data, size_t size) override;
  void close() override;
  std::string describe() const override;

private:
  IoOptions options_;
  serial::Serial serial_;
  mutable std::mutex read_mutex_;
  mutable std::mutex write_mutex_;
  bool opened_ = false;
};

}  // namespace info_relay::io

#endif  // INFO_RELAY__IO__SERIAL__SERIAL_IO_HPP
