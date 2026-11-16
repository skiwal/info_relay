/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__SERIAL__PROTOCOL__DISPATCH_HPP
#define INFO_RELAY__IO__SERIAL__PROTOCOL__DISPATCH_HPP

#include <cstdint>
#include <functional>
#include <optional>

#include "IO/serial/protocol/downlink.hpp"
#include "IO/serial/protocol/uplink_host.hpp"
#include "IO/serial/protocol/frame.hpp"
#include "IO/serial/protocol/uplink_referee.hpp"

namespace info_relay
{

/**
 * @brief 上位机侧链路解包：按 cmd_id 号段把帧分给对应解码器。
 * @details 下位机把裁判转发帧与自己的 host 帧放在同一条链路上，号段约定：
 *   裁判 0x0001~0x0A06（原样转发）  下行 0x0400/0x0401  上行 0x05xx
 * 0x05xx 交给 host_comm；0x0400/0x0401 交给下行解码器（正常只在本端发送，
 * 若线缆回环或下位机回显则记录最近一次解包结果）；其余交给赛季协议。
 * 并发：OnCommand 在接收线程中调用，on_frame 也在同一线程内同步回调。
 */
class MinipcLink final : public referee_comm::Protocol
{
public:
  bool OnCommand(const referee_comm::Frame & frame) override;

  /** @brief MCU 上行帧（0x0500~0x0505）。 */
  host_comm::Protocol host;
  /** @brief 下位机转发的裁判系统帧（按赛季解析）。 */
  referee_comm::v2027::Protocol referee;

  /** @brief 最近一次回显/回环收到的下行自瞄帧，未收到时为空。 */
  std::optional<AimFields> last_aim_downlink;
  /** @brief 最近一次回显/回环收到的下行导航帧，未收到时为空。 */
  std::optional<NavFields> last_nav_downlink;

  /**
   * @brief 每解出一帧就回调一次，参数为刚处理完的 cmd_id。
   * @note 指向的载荷在 host / referee 里，回调内直接读即可。
   */
  std::function<void(uint16_t cmd_id)> on_frame;

  /** @brief 云台构型（0 无，1 gimbal，2 gimbal_DoubleYaw，3 gimbal_Fold），来自 0x0500。 */
  uint8_t gimbal_type() const { return host.version.gimbal_type; }
};

}  // namespace info_relay

#endif  // INFO_RELAY__IO__SERIAL__PROTOCOL__DISPATCH_HPP
