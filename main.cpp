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
 * info_relay 进程入口：读配置 → 初始化日志 → 打开串口链路 → 运行转发主循环。
 *
 * 数据流：配置文件 → RelayConfig → init_logging() → SerialBoard 打开传输层并起接收线程
 *         → run() 把解析结果发布到 iceoryx2，同时把视觉指令写回串口。
 *
 * 线程：本文件只在主线程执行；串口接收线程由 SerialBoard 内部创建。
 *
 * 依赖与边界：不引用任何外部工程的源码或产物；配置、日志、录制路径全部由
 *             paths::home() 派生，默认落在本仓库内。
 *
 * 注意：1) 本进程是整机唯一打开串口的程序，视觉进程不得再打开同一设备；
 *       2) init_logging() 必须早于 SerialBoard 构造，后者构造期就会写日志；
 *       3) 未给命令行参数时使用 <repo>/configs/info_relay.yaml。
 */
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

#include "common/exiter.hpp"
#include "common/logger.hpp"
#include "common/paths.hpp"
#include "config.hpp"
#include "link/serial_link.hpp"
#include "relay.hpp"

#ifdef INFO_RELAY_WITH_ROS2
#include <rclcpp/rclcpp.hpp>

#include "bridge/ros2/ros2.hpp"
#endif

int main(int argc, char * argv[])
{
  const std::string config_path =
    argc >= 2 ? std::string(argv[1]) : info_relay::paths::default_config().string();

  info_relay::Options opt;
  try {
    const auto cfg = info_relay::RelayConfig::load(config_path);

    // 日志目录必须在任何一次写日志之前定下来：SerialBoard 构造期就会打日志
    info_relay::init_logging(
      cfg.log.dir.empty() ? info_relay::paths::logs() : std::filesystem::path(cfg.log.dir));

    opt.prefix = cfg.relay.prefix;
    opt.timeout_ms = cfg.relay.timeout_ms;
    // 数据源类型只用于 Health 主题里说明链路形态，取值与 SourceMode 的枚举顺序一致
    opt.source_kind = static_cast<uint8_t>(cfg.serial.mode);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[info_relay] 启动失败：%s\n", e.what());
    return 1;
  }

  info_relay::logger()->info(
    "[info_relay] 配置 {}，仓库根 {}", config_path, info_relay::paths::home().string());

  // 串口链路：构造即打开传输层并启动接收线程；打不开时内部直接报错退出，
  // 因为本进程存在的唯一目的就是持有串口，没有串口就无事可做。
  info_relay::SerialBoard board(config_path);

#ifdef INFO_RELAY_WITH_ROS2
  // 导航与裁判桥接：导航速度收到即下发串口；云台关节与裁判状态由发布节点对外发布。
  // 顺序要求：ros2 必须先于 Exiter 构造——rclcpp::init 会接管 SIGINT 处理，
  // 后安装的 handler 才能让 Ctrl+C 正常置位退出标志。
  info_relay::ROS2 ros2([&board](const info_relay::NavCommand & cmd) { board.send(cmd); });
  board.set_gimbal_joint_callback(ros2.gimbal_joint_sender());
  ros2.set_referee_source([&board] { return board.referee_state(); });

  info_relay::Exiter exiter;
  return info_relay::run(opt, board, [&exiter] { return exiter.exit() || !rclcpp::ok(); });
#else
  info_relay::Exiter exiter;
  return info_relay::run(opt, board, [&exiter] { return exiter.exit(); });
#endif
}
