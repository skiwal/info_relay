/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/inc/IO.hpp"

#include "IO/can/can_io.hpp"
#include "IO/serial/serial_io.hpp"

namespace info_relay::io
{

IO::Ptr make_io(const IoOptions & options)
{
  switch (options.type) {
    case IoType::Serial: {
      auto impl = std::make_unique<SerialIo>();
      return impl->init(options) ? std::move(impl) : nullptr;
    }
    case IoType::Can: {
      auto impl = std::make_unique<CanIo>();
      return impl->init(options) ? std::move(impl) : nullptr;
    }
  }
  return nullptr;
}

}  // namespace info_relay::io
