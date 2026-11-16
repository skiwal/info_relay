/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__SRC__RELAY_HPP
#define INFO_RELAY__SRC__RELAY_HPP

/*
 * relay 主循环：启动通信链路，把解析结果按主题发布到 iceoryx2，
 * 并把外部写入的指令通过同一通信链路发回下位机。
 *
 * 数据流：CommBoard 接收线程解析 → 本循环按节拍取样 → iceoryx2 主题；
 *         外部进程写入 AimCmd/NavCmd → 本循环取出 → CommBoard::send → IO::write。
 *
 * 主题对任何有需求的进程开放，不区分导航与自瞄；本循环只做转发与节流。
 */

#include <functional>

#include "tools/config.hpp"

namespace info_relay
{

// 功能：运行 relay 主循环，直到 should_exit 返回 true
// 输入：
//   cfg          完整配置，含通信链路与 iceoryx2 参数
//   should_exit  退出判定回调，每个节拍调用一次
// 输出：进程退出码，0 表示正常退出，非 0 表示初始化失败
int run(const RelayConfig & cfg, const std::function<bool()> & should_exit);

}  // namespace info_relay

#endif  // INFO_RELAY__SRC__RELAY_HPP
