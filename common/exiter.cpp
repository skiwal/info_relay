/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

/*
 * 退出信号的实现：把 SIGINT/SIGTERM 写进一个原子标志。
 *
 * 注意：契约见 exiter.hpp，本文件只写实现要点。
 */
#include "common/exiter.hpp"

#include <atomic>
#include <csignal>
#include <stdexcept>

namespace info_relay
{
namespace
{
std::atomic<bool> exit_flag{false};  // 退出标志，信号处理器写、任意线程读
bool exiter_inited = false;          // 是否已安装处理器，防止重复构造
}  // namespace

Exiter::Exiter()
{
  if (exiter_inited) throw std::runtime_error("Multiple Exiter instances!");
  // 处理器里只允许写标志：日志、内存分配在信号上下文里都是不安全的
  std::signal(SIGINT, [](int) { exit_flag = true; });
  std::signal(SIGTERM, [](int) { exit_flag = true; });
  exiter_inited = true;
}

bool Exiter::exit() const { return exit_flag; }

bool exiting() { return exit_flag; }

void request_exit() { exit_flag = true; }

}  // namespace info_relay
