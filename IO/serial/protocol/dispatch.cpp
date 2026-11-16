/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/serial/protocol/dispatch.hpp"

#include "tools/comm_warning.hpp"

namespace info_relay
{

bool MinipcLink::OnCommand(const referee_comm::Frame & frame)
{
  // 下行帧正常只由本端发送；若线缆回环或下位机回显，按新 ID 解包并记录。
  if (frame.cmd_id == kCmdAimDownlink) {
    AimFields fields;
    if (!decode_aim_payload(frame.data, frame.length, fields)) {
      comm_warnings().report(
        CommWarningKind::LengthMismatch, frame.cmd_id, static_cast<uint32_t>(kAimDataSize),
        frame.length, "downlink_aim");
      return false;
    }
    last_aim_downlink = fields;
    return true;
  }
  if (frame.cmd_id == kCmdNavDownlink) {
    NavFields fields;
    if (!decode_nav_payload(frame.data, frame.length, fields)) {
      comm_warnings().report(
        CommWarningKind::LengthMismatch, frame.cmd_id, static_cast<uint32_t>(kNavDataSize),
        frame.length, "downlink_nav");
      return false;
    }
    last_nav_downlink = fields;
    return true;
  }

  // 号段 0x05xx 是 MCU 上行 host 帧，其余裁判 0x0xxx 走赛季协议。
  const uint16_t segment = static_cast<uint16_t>(frame.cmd_id & 0xFF00);
  const bool handled = (segment == 0x0500) ? host.OnCommand(frame) : referee.OnCommand(frame);

  // 只有被识别并成功解出的上行帧才回调，避免下行回显触发“有新 IMU”判断。
  if (handled && on_frame) on_frame(frame.cmd_id);
  return handled;
}

}  // namespace info_relay
