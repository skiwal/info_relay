/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "link/protocol/link.hpp"

namespace info_relay
{

bool MinipcLink::OnCommand(const referee_comm::Frame & frame)
{
  // 号段 0x05xx 是 MCU 上行 host 帧，其余（含裁判 0x0xxx 与下行 0x04xx）走赛季协议。
  const uint16_t segment = static_cast<uint16_t>(frame.cmd_id & 0xFF00);
  const bool handled = (segment == 0x0500) ? host.OnCommand(frame) : referee.OnCommand(frame);

  // 只有被识别并成功解出的帧才回调，避免未知命令反复触发上层逻辑。
  if (handled && on_frame) on_frame(frame.cmd_id);
  return handled;
}

}  // namespace info_relay
