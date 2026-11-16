/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__TOOLS__EXITER_HPP
#define INFO_RELAY__TOOLS__EXITER_HPP

/*
 * 退出信号：把 SIGINT/SIGTERM 收敛成一个可轮询的布尔标志。
 *
 * 数据流：构造 Exiter → 安装信号处理器 → 主循环每拍查 exit()；
 *         Ctrl+C 或 kill 之后主循环正常走完清理流程再退出。
 *
 * 依赖与边界：只用 <csignal> 与一个原子标志，不涉及任何线程或回调；
 *             Exiter 应在主循环启动前构造，以便尽早接管 SIGINT/SIGTERM。
 *
 * 使用注意：全进程只允许一个实例，重复构造抛异常；
 *           信号处理器里只做赋值，不做日志与其他系统调用。
 */

namespace info_relay
{

// 功能：查询是否已请求退出
// 输入：无
// 输出：true 表示收到过退出信号或有人显式请求退出
// 备注：可在任意线程调用，无锁
bool exiting();

// 功能：显式请求退出
// 输入：无
// 输出：无
// 备注：用于让阻塞等待的线程尽快返回，避免退出时 join 卡死
void request_exit();

class Exiter
{
public:
  // 功能：安装 SIGINT/SIGTERM 处理器
  // 输入：无
  // 输出：无
  // 备注：全进程只允许一个实例，重复构造抛 std::runtime_error
  Exiter();

  // 功能：查询退出标志
  // 输入：无
  // 输出：true 表示已请求退出
  // 备注：等价于全局 exiting()
  bool exit() const;
};

}  // namespace info_relay

#endif  // INFO_RELAY__TOOLS__EXITER_HPP
