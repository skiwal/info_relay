/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__RELAY_HPP
#define INFO_RELAY__RELAY_HPP

/*
 * relay 主循环：把串口解析结果按主题发布到 iceoryx2，把视觉下发的指令写回串口。
 *
 * 数据流：SerialBoard（接收线程解析）→ 本循环按节拍取样 → iceoryx2 主题；
 *         视觉侧 AimCmd → 本循环取出 → info_relay::Command → SerialBoard::send → 串口。
 *
 * 线程：全程在调用线程内运行，不额外起线程；SerialBoard 自己的接收线程负责收字节。
 *
 * 注意：本循环只做转发与节流，不做任何坐标变换、滤波或决策。
 */

#include <cstdint>
#include <functional>
#include <string>

#include "link/serial_link.hpp"
#include "msg/relay_msgs.hpp"

namespace info_relay
{

// 运行期参数，全部来自配置文件
struct Options
{
  std::string prefix = msg::kDefaultPrefix;  // 主题名前缀，实际主题为 "<prefix>/<topic>"
  int timeout_ms = 200;                      // 超过该时长无上行帧即判链路失效，ms
  uint8_t source_kind = 0;                   // 数据源：0 real / 1 fake / 2 replay
};

// 功能：运行 relay 主循环，直到 should_exit 返回 true
// 输入：
//   opt          运行期参数
//   board        已打开传输层的串口链路对象，调用方保证其生命周期覆盖整个循环
//   should_exit  退出判定回调，每个节拍调用一次；返回 true 时正常退出
// 输出：进程退出码，0 表示正常退出，非 0 表示 iceoryx2 初始化失败
// 备注：阻塞直到退出；内部按 500Hz 节拍运行，Imu 按"有新帧才发"节流
int run(const Options & opt, info_relay::SerialBoard & board, const std::function<bool()> & should_exit);

}  // namespace info_relay

#endif  // INFO_RELAY__RELAY_HPP
